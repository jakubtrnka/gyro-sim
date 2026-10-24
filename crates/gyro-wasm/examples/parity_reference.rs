//! Native fixtures for the exact same serialized configuration and commands used by Node/WASM.
use gyro_core::*;
use serde::Serialize;
use std::io::{self, Write};

#[derive(Serialize)]
struct Fixture {
    name: String,
    replay: Replay,
    samples: Vec<Snapshot>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut fixtures = Vec::new();
    for preset in [Preset::Attitude, Preset::Directional, Preset::TurnIndicator] {
        for spin in [12_000.0, -12_000.0, 0.0] {
            let config = SimulationConfig::for_preset(preset);
            let mut initial = InitialState::for_preset(preset);
            initial.joints.angles[0] = 0.1;
            initial.joints.rates[preset.spin_index()] = rpm_to_rad_s(spin);
            initial.drive.target_spin = rpm_to_rad_s(spin);
            if preset != Preset::TurnIndicator {
                initial.joints.angles[1] = 0.2;
            }
            let mut sim = Simulation::new(config, initial)?;
            sim.queue(Command::SetHousingTarget([0.17, -0.29, 0.31, 0.89]))?;
            for (time, command) in [
                (2.0, Command::SetDriveMode(DriveMode::Coast)),
                (4.0, Command::SetHousingTarget([-0.1, 0.2, 0.15, 0.95])),
                (6.0, Command::SetDriveMode(DriveMode::Powered)),
                (6.0, Command::SetTargetSpin(rpm_to_rad_s(-spin))),
                (8.0, Command::SetHeadingOffset(0.4)),
                (10.0, Command::StopHousing), // pending at exported end step
            ]
            .into_iter()
            .enumerate()
            .map(|(sequence, (time, command))| (time, (sequence as u32, command)))
            {
                sim.enqueue(CommandRecord {
                    step_index: (time / config.step_seconds) as u64,
                    sequence: command.0,
                    command: command.1,
                })?;
            }
            let mut samples = vec![sim.snapshot()];
            for _ in 0..20 {
                samples.push(
                    sim.advance_steps((0.5 / config.step_seconds) as u32)?
                        .current,
                );
            }
            fixtures.push(Fixture {
                name: format!("{preset:?}/{spin}rpm"),
                replay: sim.replay(),
                samples,
            });
        }
    }
    let stdout = io::stdout();
    let mut output = io::BufWriter::new(stdout.lock());
    serde_json::to_writer(&mut output, &fixtures)?;
    output.flush()?;
    Ok(())
}
