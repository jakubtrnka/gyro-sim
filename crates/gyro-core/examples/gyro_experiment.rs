//! Exercise every preset, queued housing commands, instrument readings and replay.

use gyro_core::*;
use std::error::Error;
use std::io::{self, Write};

fn main() -> Result<(), Box<dyn Error>> {
    let mut preset = Preset::Attitude;
    let mut motion = String::from("roll");
    let mut rpm = 12000.0;
    let mut mode = DriveMode::Powered;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--preset" => {
                preset = match args.next().ok_or("missing preset")?.as_str() {
                    "attitude" => Preset::Attitude,
                    "directional" => Preset::Directional,
                    "turn" => Preset::TurnIndicator,
                    _ => return Err("preset must be attitude, directional or turn".into()),
                }
            }
            "--motion" => motion = args.next().ok_or("missing motion")?,
            "--rpm" => rpm = args.next().ok_or("missing RPM")?.parse()?,
            "--coast" => mode = DriveMode::Coast,
            "--help" | "-h" => {
                println!(
                    "gyro_experiment [--preset attitude|directional|turn] [--motion roll|pitch|yaw|manual] [--rpm 12000] [--coast]\nPrints eight simulated seconds as CSV and checks native replay. Defaults: attitude, roll, +12000 RPM, powered.\nManual requests a mixed-axis target, then a second target at 4 s. Release leaves the last target in place."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown option {arg}; use --help").into()),
        }
    }
    let config = SimulationConfig::for_preset(preset);
    let mut initial = InitialState::for_preset(preset);
    initial.joints.rates[preset.spin_index()] = rpm_to_rad_s(rpm);
    initial.drive = DriveState {
        mode,
        target_spin: rpm_to_rad_s(rpm),
    };
    let mut sim = Simulation::new(config, initial)?;
    match motion.as_str() {
        "roll" | "pitch" => {
            sim.queue(Command::StartManeuver(Maneuver::Excursion(
                ExcursionProfile {
                    axis: if motion == "roll" {
                        Axis::Roll
                    } else {
                        Axis::Pitch
                    },
                    ..ExcursionProfile::default()
                },
            )))?;
        }
        "yaw" => {
            sim.queue(Command::StartManeuver(Maneuver::Yaw {
                rate_rad_s: 3.0_f64.to_radians(),
                ramp_seconds: 1.0,
                hold_seconds: 4.0,
            }))?;
        }
        "manual" => {
            sim.queue(Command::SetHousingTarget([0.17, -0.29, 0.31, 0.89]))?;
            sim.enqueue(CommandRecord {
                step_index: (4.0 / config.step_seconds) as u64,
                sequence: 0,
                command: Command::SetHousingTarget([-0.1, 0.2, 0.15, 0.95]),
            })?;
        }
        _ => return Err("motion must be roll, pitch, yaw or manual".into()),
    }
    let stdout = io::stdout();
    let mut out = io::BufWriter::new(stdout.lock());
    writeln!(
        out,
        "time_s,outer_angle_deg,inner_angle_deg,relative_rpm,pitch_deg,bank_deg,heading_deg,indicated_turn_deg_s,energy_balance_error_j,axis_alignment,mass_condition"
    )?;
    for sample in 0..=16 {
        let s = if sample == 0 {
            sim.snapshot()
        } else {
            sim.advance_steps((0.5 / config.step_seconds) as u32)?
                .current
        };
        let (pitch, bank, heading, turn) = match s.instrument {
            InstrumentReading::Attitude { pitch, bank } => (Some(pitch), bank, None, None),
            InstrumentReading::Directional { heading } => (None, None, Some(heading), None),
            InstrumentReading::TurnIndicator { indicated_rate, .. } => {
                (None, None, None, Some(indicated_rate))
            }
        };
        let degrees = |v: Option<f64>| {
            v.map(|v| format!("{:.6}", v.to_degrees()))
                .unwrap_or_default()
        };
        writeln!(
            out,
            "{:.3},{:.6},{},{:.6},{},{},{},{},{:.6e},{},{:.6}",
            s.time_seconds,
            s.joints.angles[0].to_degrees(),
            degrees((preset != Preset::TurnIndicator).then_some(s.joints.angles[1])),
            rad_s_to_rpm(s.joints.rates[preset.spin_index()]),
            degrees(pitch),
            degrees(bank),
            degrees(heading),
            degrees(turn),
            s.energy_balance_residual,
            s.diagnostics.near_axis_alignment,
            s.diagnostics.mass_condition
        )?;
    }
    out.flush()?;
    let replay = sim.replay();
    let reproduced = Simulation::from_replay(&replay)?;
    if reproduced.snapshot() != sim.snapshot() {
        return Err("replay snapshot mismatch".into());
    }
    eprintln!(
        "Native replay matched all final snapshot fields ({} steps, {} commands).",
        replay.end_step,
        replay.commands.len()
    );
    Ok(())
}
