mod support;
use gyro_core::*;
use support::*;

#[test]
fn all_presets_match_independent_energy_derivatives_with_moving_housing() {
    let mut seed = 2026102402;
    for preset in [Preset::TurnIndicator, Preset::Attitude, Preset::Directional] {
        let mut config = SimulationConfig {
            gimbal_inertia: [2e-5, 3e-5, 4e-5],
            inner_gimbal_inertia: [3e-5, 4e-5, 5e-5],
            spring_stiffness: 0.003,
            inner_spring_stiffness: 0.002,
            gimbal_damping: 0.0001,
            inner_gimbal_damping: 0.0002,
            ..SimulationConfig::for_preset(preset)
        };
        config.motor.spin_damping = config.rotor.axial / 30.0;
        let model = Model::new(config).unwrap();
        let mut worst: f64 = 0.0;
        for case in 0..500 {
            let mut state = JointState::for_preset(preset);
            for i in 0..preset.joint_count() {
                state.angles[i] = random(&mut seed, 3.0);
                state.rates[i] = random(
                    &mut seed,
                    if i == preset.spin_index() {
                        1500.0
                    } else {
                        3.0
                    },
                );
            }
            if case % 10 == 0 {
                state.rates[preset.spin_index()] = 0.0;
            }
            if preset != Preset::TurnIndicator && case % 11 == 0 {
                state.angles[1] = std::f64::consts::FRAC_PI_2;
            }
            let omega = std::array::from_fn(|_| random(&mut seed, 3.0));
            let alpha = std::array::from_fn(|_| random(&mut seed, 4.0));
            let angle = random(&mut seed, 3.0);
            let housing = HousingSample {
                orientation: orientation(angle),
                angular_velocity: rotated(omega, angle),
                angular_acceleration: rotated(alpha, angle),
            };
            let drive = DriveState {
                mode: if case % 3 == 0 {
                    DriveMode::Coast
                } else {
                    DriveMode::Powered
                },
                target_spin: random(&mut seed, 1500.0),
            };
            let actual = model.evaluate(state, housing, drive).unwrap();
            let (expected, energy) = energy_reference(config, state, drive, omega, alpha);
            for (a, b) in actual
                .accelerations
                .into_iter()
                .zip(expected)
                .take(preset.joint_count())
            {
                let error = (a - b).abs() / (1.0 + a.abs().max(b.abs()));
                worst = worst.max(error);
                assert!(
                    error <= 1e-9,
                    "{preset:?}, case {case}, error {error:e}, actual {a}, expected {b}"
                );
            }
            close(actual.diagnostics.total_energy(), energy, 1e-12, 1e-12);
        }
        eprintln!(
            "{preset:?}: 500 moving-housing energy references, maximum scaled error {worst:.3e}"
        );
    }
}
