use gyro_core::*;

fn norm(v: [f64; 3]) -> f64 {
    v.into_iter().map(|x| x * x).sum::<f64>().sqrt()
}
fn run(preset: Preset) -> Simulation {
    Simulation::new(
        SimulationConfig::for_preset(preset),
        InitialState::for_preset(preset),
    )
    .unwrap()
}
fn target() -> [f64; 4] {
    [0.17, -0.29, 0.31, 0.89]
}

#[test]
fn queued_controls_are_ordered_and_replay_is_batch_independent() {
    for preset in [Preset::TurnIndicator, Preset::Attitude, Preset::Directional] {
        let mut sim = run(preset);
        let initial = sim.snapshot();
        let records = [
            CommandRecord {
                step_index: 100,
                sequence: 1,
                command: Command::SetTargetSpin(-50.0),
            },
            CommandRecord {
                step_index: 0,
                sequence: 5,
                command: Command::SetHousingTarget(target()),
            },
            CommandRecord {
                step_index: 100,
                sequence: 0,
                command: Command::SetTargetSpin(50.0),
            },
            CommandRecord {
                step_index: 50,
                sequence: 0,
                command: Command::SetDriveMode(DriveMode::Coast),
            },
            CommandRecord {
                step_index: 200,
                sequence: 0,
                command: Command::SetDriveMode(DriveMode::Powered),
            },
            CommandRecord {
                step_index: 0,
                sequence: 2,
                command: Command::SetHousingTarget([0.3, 0.0, 0.0, 0.9]),
            },
        ];
        for record in records {
            sim.enqueue(record).unwrap();
        }
        assert_eq!(sim.snapshot(), initial);
        assert_eq!(sim.advance_steps(0).unwrap().current, initial);
        let mut singles = Simulation::from_replay(&sim.replay()).unwrap();
        sim.advance_steps(1000).unwrap();
        for _ in 0..1000 {
            singles.step().unwrap();
        }
        assert_eq!(sim.snapshot(), singles.snapshot());
        assert_eq!(sim.snapshot().drive.target_spin, -50.0);
        let event = sim.queue(Command::SetHeadingOffset(1.2)).unwrap();
        assert_eq!((event.step_index, event.sequence), (1000, 0));
        let replay = sim.replay();
        let mut restored = Simulation::from_replay(&replay).unwrap();
        assert_eq!(sim.snapshot(), restored.snapshot());
        assert_eq!(replay, restored.replay());
        assert_eq!(sim.step().unwrap(), restored.step().unwrap());
        assert_eq!(sim.snapshot().heading_offset, 1.2);
        sim.reset(replay.initial).unwrap();
        assert_eq!(sim.snapshot(), initial);
        assert!(sim.replay().commands.is_empty());
    }
}

#[test]
fn malformed_commands_and_replays_do_not_mutate_a_run() {
    let mut sim = run(Preset::Directional);
    sim.queue(Command::SetTargetSpin(100.0)).unwrap();
    let before = sim.replay();
    for command in [
        Command::SetTargetSpin(f64::NAN),
        Command::SetHeadingOffset(f64::INFINITY),
        Command::SetHousingTarget([0.0; 4]),
        Command::SetHousingTarget([f64::NAN; 4]),
        Command::StartManeuver(Maneuver::Excursion(ExcursionProfile {
            move_seconds: 0.1,
            ..ExcursionProfile::default()
        })),
    ] {
        assert!(sim.queue(command).is_err());
        assert_eq!(sim.replay(), before);
    }
    assert!(sim.enqueue(before.commands[0]).is_err());
    sim.step().unwrap();
    assert!(
        sim.enqueue(CommandRecord {
            step_index: 0,
            sequence: 9,
            command: Command::StopHousing
        })
        .is_err()
    );
    let mut invalid = sim.replay();
    invalid.version = 0;
    assert!(Simulation::from_replay(&invalid).is_err());
    invalid.version = Replay::VERSION;
    invalid.commands.push(invalid.commands[0]);
    assert!(Simulation::from_replay(&invalid).is_err());
}

#[test]
fn manual_target_is_not_a_pose_overwrite_and_work_balances() {
    for preset in [Preset::TurnIndicator, Preset::Attitude, Preset::Directional] {
        let mut sim = run(preset);
        let config = sim.config();
        let before = sim.snapshot();
        sim.queue(Command::SetHousingTarget(target())).unwrap();
        sim.set_target_spin(rpm_to_rad_s(11000.0)).unwrap();
        assert_eq!(sim.snapshot(), before);
        let first = sim.step().unwrap();
        assert!(
            norm(first.housing.angular_velocity)
                <= config.housing_controller.max_acceleration * config.step_seconds * 1.00001
        );
        assert!(
            first.housing.orientation[..3]
                .iter()
                .all(|x| x.abs() < 1e-7)
        );
        assert!(first.housing_target.is_some());
        for _ in 0..256 {
            let s = sim
                .advance_steps((4.0 / 256.0 / config.step_seconds) as u32)
                .unwrap()
                .current;
            let qnorm = s.housing.orientation.iter().map(|x| x * x).sum::<f64>();
            assert!((qnorm - 1.0).abs() < 1e-14);
            assert!(
                norm(s.housing.angular_acceleration)
                    <= config.housing_controller.max_acceleration * (1.0 + 1e-12)
            );
            assert!(!s.housing_rate_exceeded);
            assert!(
                s.energy_balance_residual.abs() < 1e-7,
                "{preset:?} residual {}",
                s.energy_balance_residual
            );
        }
        let s = sim.snapshot();
        assert!(norm(s.housing.angular_velocity) < 0.001);
        let target = s.housing_target.unwrap();
        let alignment = s
            .housing
            .orientation
            .iter()
            .zip(target)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            .abs();
        assert!(1.0 - alignment < 1e-7);
        assert!(s.work.housing.abs() > 1e-8);
        assert!(s.work.motor.abs() > 1e-6);
        assert_eq!(
            Simulation::from_replay(&sim.replay()).unwrap().snapshot(),
            s
        );
    }
}

#[test]
fn shortest_path_has_deterministic_quaternion_sign_and_pi_tie_break() {
    for q in [
        target(),
        [1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 0.0, 0.1, -0.99],
    ] {
        let mut a = run(Preset::Attitude);
        let mut b = a.clone();
        a.queue(Command::SetHousingTarget(q)).unwrap();
        b.queue(Command::SetHousingTarget(q.map(|x| -x))).unwrap();
        assert_eq!(
            a.advance_steps(1000).unwrap(),
            b.advance_steps(1000).unwrap()
        );
    }
    let mut sim = run(Preset::Directional);
    sim.queue(Command::SetHousingTarget([0.0, 0.0, 0.1, -0.99]))
        .unwrap();
    assert!(sim.step().unwrap().housing.angular_velocity[2] < 0.0); // short negative yaw, not ~349 degrees positive
}

#[test]
fn switching_from_script_to_manual_preserves_pose_and_velocity() {
    let config = SimulationConfig::for_preset(Preset::Directional);
    let initial = InitialState {
        housing: HousingMotion::YawRate(YawRateProfile::default()),
        ..InitialState::for_preset(Preset::Directional)
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    sim.advance_steps((2.0 / config.step_seconds) as u32)
        .unwrap();
    let before = sim.snapshot();
    sim.queue(Command::StopHousing).unwrap();
    assert_eq!(sim.snapshot(), before);
    let after = sim.step().unwrap();
    assert_eq!(after.housing_target, Some(before.housing.orientation));
    assert!(after.housing.angular_velocity[2] > 0.0);
    assert!(
        (after.housing.angular_velocity[2] - before.housing.angular_velocity[2]).abs()
            < config.housing_controller.max_acceleration * config.step_seconds
    );
    assert!(
        after
            .housing
            .orientation
            .iter()
            .zip(before.housing.orientation)
            .all(|(a, b)| (a - b).abs() < 0.1 * config.step_seconds)
    );
    sim.advance_steps((4.0 / config.step_seconds) as u32)
        .unwrap();
    assert!(norm(sim.snapshot().housing.angular_velocity) < 1e-5);
}

#[test]
fn failed_command_step_is_atomic_including_other_commands_and_log() {
    let config = SimulationConfig::for_preset(Preset::Attitude);
    let initial = InitialState {
        housing: HousingMotion::Manual {
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [2.0, 0.0, 0.0],
            target: [0.0, 0.0, 0.0, 1.0],
        },
        ..InitialState::for_preset(Preset::Attitude)
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    assert!(sim.snapshot().housing_rate_exceeded); // monitor actual rate; do not clamp it
    sim.queue(Command::SetTargetSpin(-10.0)).unwrap();
    sim.queue(Command::StartManeuver(Maneuver::Excursion(
        ExcursionProfile::default(),
    )))
    .unwrap();
    let before = sim.snapshot();
    let replay = sim.replay();
    for _ in 0..2 {
        assert!(sim.step().is_err());
        assert_eq!(sim.snapshot(), before);
        assert_eq!(sim.replay(), replay);
    }
    sim.reset(initial).unwrap();
    let after = sim.step().unwrap();
    assert!(
        after.housing.angular_velocity[0] < 2.0
            && after.housing.angular_velocity[0] > config.housing_controller.max_rate
    );
}

#[test]
fn numerical_failure_preserves_pending_commands_and_all_snapshot_fields() {
    let config = SimulationConfig {
        step_seconds: 1e200,
        ..SimulationConfig::default()
    };
    let initial = InitialState {
        joints: JointState::turn_indicator([0.2, 0.0], [0.0, 100.0]),
        ..InitialState::default()
    };
    let mut sim = Simulation::new(config, initial).unwrap();
    sim.set_target_spin(-20.0).unwrap();
    sim.queue(Command::SetHeadingOffset(0.3)).unwrap();
    sim.queue(Command::StopHousing).unwrap();
    let before = sim.snapshot();
    let replay = sim.replay();
    assert!(sim.step().is_err());
    assert_eq!(sim.snapshot(), before);
    assert_eq!(sim.replay(), replay);
}

#[test]
fn preset_padding_controller_and_heading_inputs_are_validated() {
    let config = SimulationConfig::default();
    let initial = InitialState::default();
    assert!(
        Simulation::new(
            config,
            InitialState {
                joints: JointState {
                    angles: [0.0, 0.0, 0.1],
                    ..initial.joints
                },
                ..initial
            }
        )
        .is_err()
    );
    assert!(
        Simulation::new(
            config,
            InitialState {
                heading_offset: f64::NAN,
                ..initial
            }
        )
        .is_err()
    );
    assert!(
        Simulation::new(
            SimulationConfig {
                housing_controller: HousingControllerConfig {
                    rate_time_constant: 0.0,
                    ..config.housing_controller
                },
                ..config
            },
            initial
        )
        .is_err()
    );
    assert!(
        Simulation::new(
            SimulationConfig {
                inner_gimbal_inertia: [1.0, 1.0, 4.0],
                ..config
            },
            initial
        )
        .is_err()
    );
    assert!(
        Simulation::new(
            SimulationConfig {
                nominal_axial_momentum: 0.0,
                ..config
            },
            initial
        )
        .is_err()
    );
    assert!(
        Simulation::new(
            SimulationConfig {
                inner_gimbal_damping: -1.0,
                ..config
            },
            initial
        )
        .is_err()
    );
    let manual = HousingMotion::Manual {
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.0; 3],
        target: [0.0; 4],
    };
    assert!(
        Simulation::new(
            config,
            InitialState {
                housing: manual,
                ..initial
            }
        )
        .is_err()
    );
    let manual = HousingMotion::Manual {
        orientation: [0.0, 0.0, 0.0, 1.0],
        angular_velocity: [0.0; 3],
        target: [0.0, 0.0, 0.0, 1.0],
    };
    assert!(manual.sample(1.0).is_err());
    let extreme = SimulationConfig {
        spring_stiffness: 1e300,
        nominal_axial_momentum: 1e-300,
        ..config
    };
    let neutral = Simulation::new(extreme, initial).unwrap();
    assert!(matches!(
        neutral.snapshot().instrument,
        InstrumentReading::TurnIndicator {
            indicated_rate: 0.0,
            ..
        }
    ));
    assert!(
        Simulation::new(
            extreme,
            InitialState {
                joints: JointState {
                    angles: [1e-100, 0.0, 0.0],
                    ..initial.joints
                },
                ..initial
            }
        )
        .is_err()
    );
}

#[test]
fn excursion_derivatives_are_consistent_at_boundaries_and_within_segments() {
    for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
        let index = match axis {
            Axis::Roll => 0,
            Axis::Pitch => 1,
            Axis::Yaw => 2,
        };
        let motion = HousingMotion::Excursion(ExcursionProfile {
            axis,
            ..ExcursionProfile::default()
        });
        for t in [0.0, 0.2, 1.0, 2.0, 3.0, 4.0, 4.3, 5.9, 6.0, 7.0] {
            let epsilon = 1e-5;
            if t == 0.0 {
                continue;
            }
            let a = motion.sample(t - epsilon).unwrap();
            let b = motion.sample(t + epsilon).unwrap();
            let center = motion.sample(t).unwrap();
            let angle = |s: HousingSample| 2.0 * s.orientation[index].atan2(s.orientation[3]);
            assert!(
                ((angle(b) - angle(a)) / (2.0 * epsilon) - center.angular_velocity[index]).abs()
                    < 1e-9
            );
            assert!(
                ((b.angular_velocity[index] - a.angular_velocity[index]) / (2.0 * epsilon)
                    - center.angular_acceleration[index])
                    .abs()
                    < 1e-5
            );
        }
        assert_eq!(motion.sample(7.0).unwrap(), HousingSample::default());
    }
}

#[test]
fn maneuver_start_is_relative_to_current_pose_and_replays() {
    let initial = InitialState {
        housing: HousingMotion::Fixed {
            orientation: [
                0.0,
                0.0,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
        },
        ..InitialState::for_preset(Preset::Attitude)
    };
    let config = SimulationConfig::for_preset(Preset::Attitude);
    let mut sim = Simulation::new(config, initial).unwrap();
    sim.advance_steps(20).unwrap();
    sim.queue(Command::StartManeuver(Maneuver::Excursion(
        ExcursionProfile::default(),
    )))
    .unwrap();
    let s = sim
        .advance_steps((1.0 / config.step_seconds) as u32)
        .unwrap()
        .current;
    assert!(s.housing.angular_velocity[0].abs() < 1e-12);
    assert!(s.housing.angular_velocity[1] > 0.0); // local roll became world +y
    assert_eq!(
        Simulation::from_replay(&sim.replay()).unwrap().snapshot(),
        s
    );
}

#[test]
fn instrument_signs_offsets_and_undefined_bank_are_explicit() {
    for spin in [rpm_to_rad_s(12000.0), -rpm_to_rad_s(12000.0), 0.0] {
        let initial = InitialState {
            joints: JointState {
                angles: [-0.2, -0.1, 0.0],
                rates: [0.0, 0.0, spin],
            },
            ..InitialState::for_preset(Preset::Attitude)
        };
        let config = SimulationConfig::for_preset(Preset::Attitude);
        let sim = Simulation::new(config, initial).unwrap();
        match sim.snapshot().instrument {
            InstrumentReading::Attitude {
                pitch,
                bank: Some(bank),
            } => {
                assert!((pitch - 0.1).abs() < 1e-12);
                assert!((bank - 0.2).abs() < 1e-12);
            }
            reading => panic!("unexpected {reading:?}"),
        }
        let sim = Simulation::new(
            config,
            InitialState {
                joints: JointState {
                    angles: [0.0, std::f64::consts::FRAC_PI_2, 0.0],
                    ..initial.joints
                },
                ..initial
            },
        )
        .unwrap();
        assert!(matches!(
            sim.snapshot().instrument,
            InstrumentReading::Attitude { bank: None, .. }
        ));
    }
    let mut a = run(Preset::Directional);
    let mut b = a.clone();
    a.queue(Command::SetHeadingOffset(-0.2)).unwrap();
    let x = a.step().unwrap();
    let y = b.step().unwrap();
    assert_eq!(x.joints, y.joints);
    assert_eq!(x.diagnostics, y.diagnostics);
    assert_eq!(
        x.instrument,
        InstrumentReading::Directional {
            heading: std::f64::consts::TAU - 0.2
        }
    );
    for preset in [Preset::Directional, Preset::Attitude] {
        let config = SimulationConfig::for_preset(preset);
        let initial = InitialState {
            housing: if preset == Preset::Directional {
                HousingMotion::YawRate(YawRateProfile::default())
            } else {
                HousingMotion::Excursion(ExcursionProfile::default())
            },
            ..InitialState::for_preset(preset)
        };
        let mut sim = Simulation::new(config, initial).unwrap();
        let s = sim
            .advance_steps((2.0 / config.step_seconds) as u32)
            .unwrap()
            .current;
        match s.instrument {
            InstrumentReading::Directional { heading } => {
                assert!((heading.to_degrees() - 4.5).abs() < 0.01)
            }
            InstrumentReading::Attitude {
                bank: Some(bank), ..
            } => assert!((bank.to_degrees() - 20.0).abs() < 0.01),
            reading => panic!("unexpected {reading:?}"),
        }
    }
}
