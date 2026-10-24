//! Independent reference equations and physical invariants from the two design documents.

use gyro_core::*;

fn close(actual: f64, expected: f64, abs: f64, rel: f64) {
    assert!(
        (actual - expected).abs() <= abs + rel * actual.abs().max(expected.abs()),
        "actual={actual:.16e}, expected={expected:.16e}, difference={:.3e}",
        (actual - expected).abs()
    );
}

fn advance(sim: &mut Simulation, duration: f64) -> Snapshot {
    let steps = duration / sim.config().step_seconds;
    close(steps, steps.round(), 1e-8, 0.0);
    sim.advance_steps(steps.round() as u32).unwrap().current
}

/// Lagrangian reference evaluated in gimbal axes, independently of spatial Jacobians.
/// For housing rate [a,b0,c0], its gimbal-resolved transverse components are
/// b=b0*cos(theta)+c0*sin(theta), c=-b0*sin(theta)+c0*cos(theta).
fn reference(
    config: SimulationConfig,
    state: JointState,
    drive: DriveState,
    omega_body: [f64; 3],
    alpha_body: [f64; 3],
) -> ([f64; 2], f64) {
    let [theta, _, _] = state.angles;
    let [u, spin, _] = state.rates;
    let (s, c) = theta.sin_cos();
    let b = omega_body[1] * c + omega_body[2] * s;
    let z = -omega_body[1] * s + omega_body[2] * c;
    let h = config.rotor.axial * (spin + b);
    let delta = config.gimbal_inertia[1] - config.gimbal_inertia[2] - config.rotor.transverse;
    let joint_inertia = config.gimbal_inertia[0] + config.rotor.transverse;
    let torque = if drive.mode == DriveMode::Powered {
        (config.motor.gain * (drive.target_spin - spin))
            .clamp(-config.motor.torque_limit, config.motor.torque_limit)
    } else {
        0.0
    } - config.motor.spin_damping * spin;
    let theta_acc =
        (h * z + delta * b * z - config.spring_stiffness * theta - config.gimbal_damping * u)
            / joint_inertia
            - alpha_body[0];
    let spin_acc = torque / config.rotor.axial - alpha_body[1] * c - alpha_body[2] * s - z * u;
    let energy = 0.5 * joint_inertia * (omega_body[0] + u).powi(2)
        + 0.5 * config.gimbal_inertia[1] * b * b
        + 0.5 * (config.gimbal_inertia[2] + config.rotor.transverse) * z * z
        + 0.5 * config.rotor.axial * (spin + b).powi(2)
        + 0.5 * config.spring_stiffness * theta * theta;
    ([theta_acc, spin_acc], energy)
}

// Test-only independent Rodrigues rotation, also giving a world-from-body quaternion.
fn rotated(v: [f64; 3], angle: f64) -> [f64; 3] {
    let axis = [
        1.0 / 14.0_f64.sqrt(),
        2.0 / 14.0_f64.sqrt(),
        3.0 / 14.0_f64.sqrt(),
    ];
    let dot = axis.iter().zip(v).map(|(a, b)| a * b).sum::<f64>();
    let cross = [
        axis[1] * v[2] - axis[2] * v[1],
        axis[2] * v[0] - axis[0] * v[2],
        axis[0] * v[1] - axis[1] * v[0],
    ];
    std::array::from_fn(|i| {
        v[i] * angle.cos() + cross[i] * angle.sin() + axis[i] * dot * (1.0 - angle.cos())
    })
}

fn orientation(angle: f64) -> [f64; 4] {
    let s = (angle / 2.0).sin() / 14.0_f64.sqrt();
    [s, 2.0 * s, 3.0 * s, (angle / 2.0).cos()]
}

fn random(seed: &mut u64, extent: f64) -> f64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    let unit = (*seed >> 11) as f64 / ((1_u64 << 53) as f64);
    (2.0 * unit - 1.0) * extent
}

#[test]
fn general_dynamics_match_independent_lagrangian_reference() {
    let mut config = SimulationConfig {
        gimbal_inertia: [2e-5, 3e-5, 4e-5],
        ..SimulationConfig::default()
    };
    config.motor.spin_damping = config.rotor.axial / 30.0;
    let model = Model::new(config).unwrap();
    let mut seed = 20261024_u64;
    let mut worst: f64 = 0.0;
    for case in 0..1000 {
        let state = JointState {
            angles: [random(&mut seed, 3.0), random(&mut seed, 10.0), 0.0],
            rates: [
                random(&mut seed, 3.0),
                if case % 10 == 0 {
                    0.0
                } else {
                    random(&mut seed, 1500.0)
                },
                0.0,
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
        // Half the cases reproduce the exact pure-yaw equations; the rest exercise
        // arbitrary moving housings with an independent body-frame energy reference.
        let omega = if case < 500 {
            [0.0, 0.0, random(&mut seed, 3.0)]
        } else {
            std::array::from_fn(|_| random(&mut seed, 3.0))
        };
        let alpha = if case < 500 {
            [0.0, 0.0, random(&mut seed, 4.0)]
        } else {
            std::array::from_fn(|_| random(&mut seed, 4.0))
        };
        let angle = random(&mut seed, 3.0);
        let housing = HousingSample {
            orientation: orientation(angle),
            angular_velocity: rotated(omega, angle),
            angular_acceleration: rotated(alpha, angle),
        };
        let actual = model.evaluate(state, housing, drive).unwrap();
        let (expected, energy) = reference(config, state, drive, omega, alpha);
        for (a, e) in actual.accelerations.into_iter().zip(expected) {
            let error = (a - e).abs() / (1.0 + a.abs().max(e.abs()));
            worst = worst.max(error);
            assert!(error <= 1e-9, "case {case}: scaled error {error:e}");
        }
        close(actual.diagnostics.total_energy(), energy, 1e-12, 1e-12);
    }
    eprintln!("1000 acceleration references: maximum scaled error {worst:.3e}");
}

#[test]
fn spin_phase_does_not_change_axisymmetric_dynamics() {
    let model = Model::new(SimulationConfig::default()).unwrap();
    let state = JointState {
        angles: [0.7, 0.0, 0.0],
        rates: [-0.2, 1300.0, 0.0],
    };
    let housing = HousingSample {
        angular_velocity: [0.1, 0.2, 0.3],
        ..HousingSample::default()
    };
    let a = model
        .evaluate(state, housing, DriveState::default())
        .unwrap();
    let b = model
        .evaluate(
            JointState {
                angles: [0.7, 98765.4, 0.0],
                ..state
            },
            housing,
            DriveState::default(),
        )
        .unwrap();
    assert_eq!(a.accelerations, b.accelerations);
    assert_eq!(a.diagnostics, b.diagnostics);
    assert_ne!(a.component_orientations[1], b.component_orientations[1]);
}

#[test]
fn small_yaw_response_spin_reversal_lower_speed_and_zero_spin() {
    let config = SimulationConfig {
        step_seconds: 1.0 / 2048.0,
        ..SimulationConfig::default()
    };
    for rpm in [12_000.0, -12_000.0, 6000.0, 0.0] {
        let spin = rpm_to_rad_s(rpm);
        let initial = InitialState {
            joints: JointState {
                rates: [0.0, spin, 0.0],
                ..JointState::default()
            },
            drive: DriveState {
                target_spin: spin,
                ..DriveState::default()
            },
            housing: HousingMotion::YawRate(YawRateProfile {
                rate_rad_s: 0.3_f64.to_radians(),
                ..YawRateProfile::default()
            }),
            ..InitialState::default()
        };
        let mut sim = Simulation::new(config, initial).unwrap();
        let settled = advance(&mut sim, 5.0);
        let expected = config.rotor.axial * spin * 0.3_f64.to_radians() / config.spring_stiffness;
        close(settled.joints.angles[0], expected, 1e-12, 0.001);
        close(settled.energy_balance_residual, 0.0, 2e-9, 0.0);
        let stopped = advance(&mut sim, 3.0);
        close(stopped.joints.angles[0], 0.0, 1e-10, 0.0);
    }
}

#[test]
fn coast_conserves_axial_momentum_under_housing_motion() {
    let config = SimulationConfig {
        step_seconds: 1.0 / 2048.0,
        ..SimulationConfig::default()
    };
    let initial = InitialState {
        joints: JointState {
            angles: [0.2, 0.0, 0.0],
            rates: [0.1, rpm_to_rad_s(12_000.0), 0.0],
        },
        drive: DriveState {
            mode: DriveMode::Coast,
            ..DriveState::default()
        },
        housing: HousingMotion::YawRate(YawRateProfile::default()),
        ..InitialState::default()
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    let h0 = sim.snapshot().diagnostics.axial_momentum;
    let during = advance(&mut sim, 2.0);
    close(during.diagnostics.axial_momentum, h0, 1e-12, 1e-10);
    assert!(
        (during.joints.rates[1] - initial.joints.rates[1]).abs() > 1e-4,
        "constant h must not be implemented by freezing relative RPM"
    );
    let end = advance(&mut sim, 6.0);
    close(end.diagnostics.axial_momentum, h0, 1e-12, 1e-10);
    close(end.energy_balance_residual, 0.0, 5e-9, 0.0);
    assert!(end.work.dissipated > 0.0);
    assert!(end.work.housing.abs() > 1e-8);
}

#[test]
fn viscous_coast_matches_exponential_spin_down() {
    let mut config = SimulationConfig {
        step_seconds: 1.0 / 128.0,
        ..SimulationConfig::default()
    };
    config.motor.spin_damping = config.rotor.axial / 30.0;
    let initial = InitialState {
        drive: DriveState {
            mode: DriveMode::Coast,
            ..DriveState::default()
        },
        ..InitialState::default()
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    let initial_energy = sim.snapshot().diagnostics.total_energy();
    let end = advance(&mut sim, 60.0);
    close(
        end.joints.rates[1],
        initial.joints.rates[1] * (-2.0_f64).exp(),
        1e-8,
        0.0,
    );
    close(end.energy_balance_residual, 0.0, 2e-9, 0.0);
    close(
        end.work.dissipated,
        initial_energy - end.diagnostics.total_energy(),
        2e-9,
        0.0,
    );
    assert_eq!(end.work.motor, 0.0);
}

#[test]
fn unsaturated_motor_has_predicted_time_constant() {
    let config = SimulationConfig {
        step_seconds: 1.0 / 256.0,
        ..SimulationConfig::default()
    };
    let initial = InitialState {
        joints: JointState {
            rates: [0.0, DriveState::default().target_spin - 5.0, 0.0],
            ..JointState::default()
        },
        ..InitialState::default()
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    let end = advance(&mut sim, 1.0);
    close(
        end.joints.rates[1],
        initial.drive.target_spin - 5.0 * (-2.0_f64).exp(),
        1e-8,
        0.0,
    );
    close(end.energy_balance_residual, 0.0, 2e-9, 0.0);
}

#[test]
fn motor_saturation_and_reversal_preserve_continuous_state() {
    let config = SimulationConfig {
        step_seconds: 1.0 / 1024.0,
        ..SimulationConfig::default()
    };
    let initial = InitialState {
        joints: JointState {
            rates: [0.0; 3],
            ..JointState::default()
        },
        ..InitialState::default()
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    close(
        sim.snapshot().diagnostics.motor_torque,
        config.motor.torque_limit,
        0.0,
        0.0,
    );
    let end = advance(&mut sim, 0.25);
    close(
        end.joints.rates[1],
        0.25 * config.motor.torque_limit / config.rotor.axial,
        1e-10,
        0.0,
    );
    close(end.energy_balance_residual, 0.0, 1e-12, 0.0);
    let before = sim.snapshot();
    sim.set_drive_mode(DriveMode::Coast).unwrap();
    assert_eq!(sim.snapshot().joints, before.joints);
    assert_eq!(sim.snapshot().work, before.work);
    assert_eq!(sim.snapshot(), before); // Queued until the next physical step.
    assert_eq!(sim.step().unwrap().diagnostics.motor_torque, 0.0);

    let reversing = InitialState {
        joints: JointState {
            rates: [0.0, 10.0, 0.0],
            ..JointState::default()
        },
        drive: DriveState {
            target_spin: 10.0,
            ..DriveState::default()
        },
        ..InitialState::default()
    };
    sim.reset(reversing).unwrap();
    let before = sim.snapshot();
    sim.set_target_spin(-10.0).unwrap();
    assert_eq!(sim.snapshot().joints, before.joints);
    let end = advance(&mut sim, 1.0);
    assert!(end.joints.rates[1] < 0.0);
    close(
        end.joints.rates[1],
        -10.0 + 20.0 * (-2.0_f64).exp(),
        1e-9,
        0.0,
    );
}

fn oscillator(dt: f64) -> Simulation {
    let config = SimulationConfig {
        step_seconds: dt,
        gimbal_damping: 0.0,
        ..SimulationConfig::default()
    };
    let initial = InitialState {
        joints: JointState {
            angles: [0.2, 0.0, 0.0],
            ..JointState::default()
        },
        drive: DriveState {
            mode: DriveMode::Coast,
            ..DriveState::default()
        },
        ..InitialState::default()
    };
    Simulation::new(config, initial).unwrap()
}

#[test]
fn rk4_step_halving_has_fourth_order_convergence() {
    let config = oscillator(1.0 / 512.0).config();
    let w = (config.spring_stiffness / (config.gimbal_inertia[0] + config.rotor.transverse)).sqrt();
    let values = [128.0, 256.0, 512.0].map(|n| {
        let mut sim = oscillator(1.0 / n);
        advance(&mut sim, 0.25).joints
    });
    let difference = |a: JointState, b: JointState| {
        // Scale angular rate by the natural frequency to compare the two phase
        // components in the same units without favoring a near-zero component.
        (a.angles[0] - b.angles[0]).hypot((a.rates[0] - b.rates[0]) / w)
    };
    let ratio = difference(values[0], values[1]) / difference(values[1], values[2]);
    assert!((12.0..20.0).contains(&ratio), "RK4 ratio={ratio}");
    close(values[2].angles[0], 0.2 * (w * 0.25).cos(), 2e-8, 0.0);
    eprintln!("RK4 step-halving ratio: {ratio:.6}");
}

#[test]
fn conservative_energy_and_axial_momentum_survive_sixty_seconds() {
    let mut sim = oscillator(1.0 / 1024.0);
    let start = sim.snapshot();
    let end = advance(&mut sim, 60.0);
    let spin_energy = start.diagnostics.axial_momentum.powi(2) / (2.0 * sim.config().rotor.axial);
    let dynamic_energy = start.diagnostics.total_energy() - spin_energy;
    let energy_error = (end.diagnostics.total_energy() - start.diagnostics.total_energy()).abs();
    assert!(
        energy_error <= 1e-10 + 1e-4 * dynamic_energy,
        "energy drift {energy_error:e}"
    );
    close(
        end.diagnostics.axial_momentum,
        start.diagnostics.axial_momentum,
        1e-12,
        1e-10,
    );
    assert_eq!(end.work, Work::default());
    eprintln!("60 s conservative run: absolute energy drift {energy_error:.3e} J");
}

#[test]
fn housing_reaction_during_motor_drive_satisfies_power_balance() {
    let model = Model::new(SimulationConfig::default()).unwrap();
    let state = JointState {
        angles: [0.3, 1.0, 0.0],
        rates: [0.4, 100.0, 0.0],
    };
    let housing = HousingSample {
        angular_velocity: [0.3, -0.7, 0.2],
        angular_acceleration: [-0.4, 0.6, 0.1],
        ..HousingSample::default()
    };
    let drive = DriveState::default();
    let eval = model.evaluate(state, housing, drive).unwrap();
    // Differentiate independently assembled body-frame energy along the trajectory.
    let epsilon = 1e-6;
    let energy_at = |dt: f64| {
        let next_state = JointState {
            angles: std::array::from_fn(|i| state.angles[i] + state.rates[i] * dt),
            rates: std::array::from_fn(|i| state.rates[i] + eval.accelerations[i] * dt),
        };
        let next_omega = std::array::from_fn(|i| {
            housing.angular_velocity[i] + housing.angular_acceleration[i] * dt
        });
        reference(
            model.config(),
            next_state,
            drive,
            next_omega,
            housing.angular_acceleration,
        )
        .1
    };
    let energy_rate = (energy_at(epsilon) - energy_at(-epsilon)) / (2.0 * epsilon);
    let d = eval.diagnostics;
    close(
        energy_rate,
        d.housing_power + d.motor_power - d.dissipated_power,
        1e-7,
        1e-7,
    );
}
