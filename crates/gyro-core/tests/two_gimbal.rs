use gyro_core::*;

fn random(seed: &mut u64, extent: f64) -> f64 {
    *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
    (2.0 * ((*seed >> 11) as f64 / ((1_u64 << 53) as f64)) - 1.0) * extent
}

/// Closed fixed-housing equations (18)-(20), including axial torque h_dot.
fn reference(config: SimulationConfig, state: JointState, drive: DriveState) -> [f64; 3] {
    let (a, [b1, b2, b3]) = match config.preset {
        Preset::Attitude => (config.gimbal_inertia[0], config.inner_gimbal_inertia),
        Preset::Directional => (
            config.gimbal_inertia[2],
            [
                config.inner_gimbal_inertia[2],
                config.inner_gimbal_inertia[0],
                config.inner_gimbal_inertia[1],
            ],
        ),
        _ => unreachable!(),
    };
    let [alpha, beta, _] = state.angles;
    let [u, v, spin] = state.rates;
    let (s, c) = beta.sin_cos();
    let d = a + (b1 + config.rotor.transverse) * c * c + b3 * s * s;
    let dp = 2.0 * (b3 - b1 - config.rotor.transverse) * s * c;
    let h = config.rotor.axial * (spin + u * s);
    let qspin = (if drive.mode == DriveMode::Powered {
        (config.motor.gain * (drive.target_spin - spin))
            .clamp(-config.motor.torque_limit, config.motor.torque_limit)
    } else {
        0.0
    }) - config.motor.spin_damping * spin;
    let qa = -config.spring_stiffness * alpha - config.gimbal_damping * u;
    let qb = -config.inner_spring_stiffness * beta - config.inner_gimbal_damping * v;
    let aa = (qa - dp * u * v - h * c * v - qspin * s) / d;
    let ab = (qb + 0.5 * dp * u * u + h * c * u) / (b2 + config.rotor.transverse);
    [aa, ab, qspin / config.rotor.axial - aa * s - u * v * c]
}

#[test]
fn fixed_housing_matches_closed_equations_at_and_near_alignment() {
    let mut seed = 20261024;
    for preset in [Preset::Attitude, Preset::Directional] {
        let mut config = SimulationConfig {
            gimbal_inertia: [2e-5, 3e-5, 4e-5],
            inner_gimbal_inertia: [4e-5, 5e-5, 6e-5],
            spring_stiffness: 0.003,
            inner_spring_stiffness: 0.002,
            gimbal_damping: 0.0001,
            inner_gimbal_damping: 0.0002,
            ..SimulationConfig::for_preset(preset)
        };
        config.motor.spin_damping = config.rotor.axial / 30.0;
        let model = Model::new(config).unwrap();
        let mut worst: f64 = 0.0;
        for case in 0..1000 {
            let state = JointState {
                angles: [
                    random(&mut seed, 3.0),
                    if case % 10 == 0 {
                        std::f64::consts::FRAC_PI_2
                    } else if case % 10 == 1 {
                        std::f64::consts::FRAC_PI_2 - 1e-8
                    } else {
                        random(&mut seed, 3.0)
                    },
                    random(&mut seed, 10.0),
                ],
                rates: [
                    random(&mut seed, 3.0),
                    random(&mut seed, 3.0),
                    if case % 13 == 0 {
                        0.0
                    } else {
                        random(&mut seed, 1500.0)
                    },
                ],
            };
            let drive = DriveState {
                mode: if case % 3 == 0 {
                    DriveMode::Coast
                } else {
                    DriveMode::Powered
                },
                target_spin: random(&mut seed, 1500.0),
            };
            let value = model
                .evaluate(state, HousingSample::default(), drive)
                .unwrap();
            for (a, b) in value
                .accelerations
                .into_iter()
                .zip(reference(config, state, drive))
            {
                let error = (a - b).abs() / (1.0 + a.abs().max(b.abs()));
                worst = worst.max(error);
                assert!(error <= 1e-9, "{preset:?} case {case}, error={error:e}");
            }
            if case % 10 <= 1 {
                assert!(value.diagnostics.near_axis_alignment);
                assert!(!value.diagnostics.poorly_conditioned);
            }
            let mut shifted = state;
            shifted.angles[2] += 123456.7;
            let other = model
                .evaluate(shifted, HousingSample::default(), drive)
                .unwrap();
            assert_eq!(value.accelerations, other.accelerations);
            assert_eq!(value.diagnostics, other.diagnostics);
        }
        eprintln!("{preset:?}: 1000 fixed-housing references, max scaled error {worst:.3e}");
    }
}

fn conservative(preset: Preset, dt: f64) -> Simulation {
    Simulation::new(
        SimulationConfig {
            step_seconds: dt,
            ..SimulationConfig::for_preset(preset)
        },
        InitialState {
            joints: JointState {
                angles: [0.1, 0.2, 0.0],
                rates: [0.5, 0.2, rpm_to_rad_s(12000.0)],
            },
            drive: DriveState {
                mode: DriveMode::Coast,
                ..DriveState::default()
            },
            ..InitialState::for_preset(preset)
        },
    )
    .unwrap()
}

#[test]
fn sixty_second_conservation_uses_dynamic_energy_not_large_spin_energy() {
    for preset in [Preset::Attitude, Preset::Directional] {
        let config = SimulationConfig::for_preset(preset);
        let mut sim = conservative(preset, config.step_seconds);
        let start = sim.snapshot().diagnostics;
        let dynamic_energy =
            start.total_energy() - start.axial_momentum.powi(2) / (2.0 * config.rotor.axial);
        let mut errors = [0.0_f64; 3];
        for _ in 0..120 {
            let d = sim
                .advance_steps((0.5 / config.step_seconds) as u32)
                .unwrap()
                .current
                .diagnostics;
            errors[0] = errors[0].max((d.total_energy() - start.total_energy()).abs());
            errors[1] = errors[1].max((d.axial_momentum - start.axial_momentum).abs());
            errors[2] =
                errors[2].max((d.generalized_momenta[0] - start.generalized_momenta[0]).abs());
        }
        eprintln!(
            "{preset:?} 60 s: max |dE|={:.3e} J (dynamic E={dynamic_energy:.3e} J), |dh|={:.3e}, |dp_alpha|={:.3e}",
            errors[0], errors[1], errors[2]
        );
        assert!(errors[0] < 1e-10 + 1e-4 * dynamic_energy);
        assert!(errors[1] < 1e-6 * start.axial_momentum.abs());
        assert!(errors[2] < 1e-6 * start.generalized_momenta[0].abs());
        assert_eq!(sim.snapshot().work, Work::default());
    }
}

#[test]
fn two_gimbal_rk4_has_fourth_order_convergence() {
    for preset in [Preset::Attitude, Preset::Directional] {
        let states = [8192.0, 16384.0, 32768.0].map(|hz| {
            let mut sim = conservative(preset, 1.0 / hz);
            sim.advance_steps((hz / 8.0) as u32).unwrap().current.joints
        });
        let distance = |a: JointState, b: JointState| {
            (0..2)
                .map(|i| {
                    (a.angles[i] - b.angles[i]).powi(2)
                        + ((a.rates[i] - b.rates[i]) / 1000.0).powi(2)
                })
                .sum::<f64>()
                .sqrt()
        };
        let ratio = distance(states[0], states[1]) / distance(states[1], states[2]);
        eprintln!("{preset:?} RK4 step-halving ratio {ratio:.6}");
        assert!((12.0..20.0).contains(&ratio));
    }
}

#[test]
fn small_motion_has_predicted_nutation_frequency() {
    for preset in [Preset::Attitude, Preset::Directional] {
        let config = SimulationConfig::for_preset(preset);
        let initial = InitialState {
            joints: JointState {
                angles: [0.0; 3],
                rates: [1e-3, 0.0, rpm_to_rad_s(12000.0)],
            },
            drive: DriveState {
                mode: DriveMode::Coast,
                ..DriveState::default()
            },
            ..InitialState::for_preset(preset)
        };
        let mut sim = Simulation::new(config, initial).unwrap();
        let expected = config.rotor.axial * initial.joints.rates[2]
            / ((4e-5 + config.rotor.transverse) * (2e-5 + config.rotor.transverse)).sqrt();
        let mut before = sim.snapshot();
        let mut crossings = Vec::new();
        for _ in 0..4096 {
            let after = sim.step().unwrap();
            if before.joints.rates[0] > 0.0 && after.joints.rates[0] <= 0.0 {
                let fraction =
                    before.joints.rates[0] / (before.joints.rates[0] - after.joints.rates[0]);
                crossings.push(before.time_seconds + fraction * config.step_seconds);
            }
            before = after;
        }
        let measured = std::f64::consts::TAU * ((crossings.len() - 1) as f64)
            / (crossings.last().unwrap() - crossings[0]);
        eprintln!("{preset:?} nutation: measured {measured:.6}, expected {expected:.6} rad/s");
        assert!((measured - expected).abs() / expected < 1e-5);
    }
}

#[test]
fn conditioning_and_physical_alignment_are_distinct_warnings() {
    let preset = Preset::Attitude;
    let initial = InitialState {
        joints: JointState {
            angles: [0.0, std::f64::consts::FRAC_PI_2, 0.0],
            ..JointState::for_preset(preset)
        },
        ..InitialState::for_preset(preset)
    };
    let config = SimulationConfig::for_preset(preset);
    let normal = Simulation::new(config, initial).unwrap().snapshot();
    assert!(normal.diagnostics.near_axis_alignment);
    assert!(!normal.diagnostics.poorly_conditioned);
    let tiny_gimbals = Simulation::new(
        SimulationConfig {
            gimbal_inertia: [1e-16; 3],
            inner_gimbal_inertia: [1e-16; 3],
            ..config
        },
        initial,
    )
    .unwrap()
    .snapshot();
    assert!(tiny_gimbals.diagnostics.near_axis_alignment);
    assert!(tiny_gimbals.diagnostics.poorly_conditioned);
    assert!(tiny_gimbals.diagnostics.mass_condition.is_finite());
}

#[test]
fn default_steps_resolve_ten_second_scripted_maneuvers() {
    for preset in [Preset::TurnIndicator, Preset::Attitude, Preset::Directional] {
        let motions = [
            HousingMotion::YawRate(YawRateProfile::default()),
            HousingMotion::Excursion(ExcursionProfile {
                axis: Axis::Pitch,
                ..ExcursionProfile::default()
            }),
            HousingMotion::Excursion(ExcursionProfile::default()),
        ];
        for motion in motions {
            let config = SimulationConfig::for_preset(preset);
            // Excite the internal motion; a neutral chain under a single-axis
            // rotation can be an exact, deceptively easy tracking solution.
            let mut initial = InitialState {
                housing: motion,
                ..InitialState::for_preset(preset)
            };
            initial.joints.angles[0] = 0.1;
            initial.joints.rates[0] = 0.1;
            if preset != Preset::TurnIndicator {
                initial.joints.angles[1] = 0.2;
                initial.joints.rates[1] = -0.15;
            }
            let mut coarse = Simulation::new(config, initial).unwrap();
            let mut fine = Simulation::new(
                SimulationConfig {
                    step_seconds: config.step_seconds / 2.0,
                    ..config
                },
                initial,
            )
            .unwrap();
            let mut angle_error = 0.0_f64;
            let mut rate_error = 0.0_f64;
            for _ in 0..2560 {
                let a = coarse
                    .advance_steps((1.0 / 256.0 / config.step_seconds) as u32)
                    .unwrap()
                    .current;
                let b = fine
                    .advance_steps((1.0 / 128.0 / config.step_seconds) as u32)
                    .unwrap()
                    .current;
                for i in 0..preset.spin_index() {
                    angle_error = angle_error.max((a.joints.angles[i] - b.joints.angles[i]).abs());
                    rate_error = rate_error.max((a.joints.rates[i] - b.joints.rates[i]).abs());
                }
                let spindle_difference = (0..3)
                    .map(|i| {
                        (a.diagnostics.spindle_direction[i] - b.diagnostics.spindle_direction[i])
                            .powi(2)
                    })
                    .sum::<f64>()
                    .sqrt();
                angle_error = angle_error.max(2.0 * (spindle_difference / 2.0).min(1.0).asin());
                assert!(a.energy_balance_residual.abs() < 1e-7);
            }
            eprintln!(
                "{preset:?} {motion:?}: 10 s dt vs dt/2, max pose {angle_error:.3e} rad, rate {rate_error:.3e} rad/s"
            );
            assert!(angle_error < 1e-4 && rate_error < 1e-3);
        }
    }
}
