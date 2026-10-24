use gyro_core::*;

#[test]
fn housing_profile_has_consistent_pose_rate_and_acceleration() {
    let profile = YawRateProfile::default();
    let motion = HousingMotion::YawRate(profile);
    let dt = 1e-5;
    for t in [0.2, 0.7, 2.0, 5.2, 5.9, 7.0] {
        let before = motion.sample(t - dt).unwrap();
        let at = motion.sample(t).unwrap();
        let after = motion.sample(t + dt).unwrap();
        let yaw = |s: HousingSample| 2.0 * s.orientation[2].atan2(s.orientation[3]);
        assert!(((yaw(after) - yaw(before)) / (2.0 * dt) - at.angular_velocity[2]).abs() < 1e-9);
        assert!(
            ((after.angular_velocity[2] - before.angular_velocity[2]) / (2.0 * dt)
                - at.angular_acceleration[2])
                .abs()
                < 1e-9
        );
    }
    for t in [1.0, 5.0, 6.0] {
        let before = motion.sample(t - 1e-8).unwrap();
        let after = motion.sample(t + 1e-8).unwrap();
        for (a, b) in before.orientation.iter().zip(after.orientation) {
            assert!((a - b).abs() < 1e-8);
        }
        for (a, b) in before.angular_velocity.iter().zip(after.angular_velocity) {
            assert!((a - b).abs() < 1e-8);
        }
        for (a, b) in before
            .angular_acceleration
            .iter()
            .zip(after.angular_acceleration)
        {
            assert!((a - b).abs() < 1e-8);
        }
    }
    let end = motion.sample(7.0).unwrap();
    let yaw = 2.0 * end.orientation[2].atan2(end.orientation[3]);
    assert!(
        (yaw - profile.rate_rad_s * (profile.ramp_seconds + profile.hold_seconds)).abs() < 1e-12
    );
    assert_eq!(end.angular_velocity, [0.0; 3]);
}

#[test]
fn batching_is_deterministic_and_reset_restores_initial_conditions() {
    let config = SimulationConfig::default();
    let initial = InitialState {
        housing: HousingMotion::YawRate(YawRateProfile::default()),
        ..InitialState::default()
    };
    let mut single = Simulation::new(config, initial).unwrap();
    let mut batch = single.clone();
    let original = single.snapshot();
    assert_eq!(single.snapshot(), original); // Reading state advances no time.
    let result = batch.advance_steps(100).unwrap();
    for _ in 0..99 {
        single.step().unwrap();
    }
    assert_eq!(result.previous, single.snapshot());
    assert_eq!(result.current, single.step().unwrap());
    let before = batch.snapshot();
    let zero = batch.advance_steps(0).unwrap();
    assert_eq!(zero.previous, before);
    assert_eq!(zero.current, before);
    batch.set_target_spin(-100.0).unwrap();
    batch.set_drive_mode(DriveMode::Coast).unwrap();
    assert_eq!(batch.reset(initial).unwrap(), original);
}

#[test]
fn invalid_configuration_and_input_are_rejected() {
    let valid = SimulationConfig::default();
    for config in [
        SimulationConfig {
            step_seconds: 0.0,
            ..valid
        },
        SimulationConfig {
            step_seconds: f64::NAN,
            ..valid
        },
        SimulationConfig {
            gimbal_inertia: [-1.0, 1.0, 1.0],
            ..valid
        },
        SimulationConfig {
            gimbal_inertia: [4.0, 1.0, 1.0],
            ..valid
        },
        SimulationConfig {
            rotor: RotorInertia {
                axial: 4.0,
                transverse: 1.0,
            },
            ..valid
        },
        SimulationConfig {
            gimbal_damping: -0.1,
            ..valid
        },
        SimulationConfig {
            motor: MotorConfig {
                spin_damping: -1.0,
                ..valid.motor
            },
            ..valid
        },
    ] {
        assert!(matches!(
            Simulation::new(config, InitialState::default()),
            Err(SimulationError::InvalidConfiguration(_))
        ));
    }
    for initial in [
        InitialState {
            joints: JointState {
                angles: [f64::NAN, 0.0],
                ..JointState::default()
            },
            ..InitialState::default()
        },
        InitialState {
            drive: DriveState {
                target_spin: f64::INFINITY,
                ..DriveState::default()
            },
            ..InitialState::default()
        },
        InitialState {
            housing: HousingMotion::Fixed {
                orientation: [0.0; 4],
            },
            ..InitialState::default()
        },
        InitialState {
            housing: HousingMotion::YawRate(YawRateProfile {
                ramp_seconds: -1.0,
                ..YawRateProfile::default()
            }),
            ..InitialState::default()
        },
        InitialState {
            housing: HousingMotion::YawRate(YawRateProfile {
                ramp_seconds: 0.1,
                ..YawRateProfile::default()
            }),
            ..InitialState::default()
        },
        InitialState {
            housing: HousingMotion::YawRate(YawRateProfile {
                ramp_seconds: 1e-20,
                ..YawRateProfile::default()
            }),
            ..InitialState::default()
        },
        InitialState {
            housing: HousingMotion::YawRate(YawRateProfile {
                rate_rad_s: 1e308,
                ..YawRateProfile::default()
            }),
            ..InitialState::default()
        },
    ] {
        assert!(Simulation::new(valid, initial).is_err());
    }
}

#[test]
fn quaternion_normalization_is_finite_and_respects_xyzw_order() {
    let initial = InitialState {
        housing: HousingMotion::Fixed {
            orientation: [0.0, 0.0, 1e300, 1e300],
        },
        ..InitialState::default()
    };
    let sim = Simulation::new(SimulationConfig::default(), initial).unwrap();
    let snapshot = sim.snapshot();
    assert!((snapshot.housing.orientation[2] - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-15);
    let spindle = snapshot.diagnostics.spindle_direction;
    assert!((spindle[0] + 1.0).abs() < 1e-14); // +y spindle rotated +90 degrees about z points -x.
    assert!(spindle[1].abs() < 1e-14 && spindle[2].abs() < 1e-14);
}

#[test]
fn failed_updates_and_failed_steps_leave_state_unchanged() {
    let mut sim = Simulation::new(SimulationConfig::default(), InitialState::default()).unwrap();
    sim.advance_steps(10).unwrap();
    let before = sim.snapshot();
    assert!(sim.set_target_spin(f64::NAN).is_err());
    assert_eq!(sim.snapshot(), before);
    let invalid = InitialState {
        joints: JointState {
            rates: [0.0, f64::INFINITY],
            ..JointState::default()
        },
        ..InitialState::default()
    };
    assert!(sim.reset(invalid).is_err());
    assert_eq!(sim.snapshot(), before);

    let config = SimulationConfig {
        step_seconds: 1e200,
        ..SimulationConfig::default()
    };
    let initial = InitialState {
        joints: JointState {
            angles: [0.2, 0.0],
            ..JointState::default()
        },
        ..InitialState::default()
    };
    let mut overflowing = Simulation::new(config, initial).unwrap();
    let before = overflowing.snapshot();
    assert!(matches!(
        overflowing.step(),
        Err(SimulationError::NumericalFailure { step: Some(0), .. })
    ));
    assert_eq!(overflowing.snapshot(), before);
}
