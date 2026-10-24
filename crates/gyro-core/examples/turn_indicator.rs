//! Run a deterministic housing-yaw experiment and print CSV to stdout.

use gyro_core::*;
use std::error::Error;
use std::io::{self, Write};

fn main() -> Result<(), Box<dyn Error>> {
    let mut config = SimulationConfig::default();
    let mut rpm = 12_000.0;
    let mut yaw_rate = 3.0_f64;
    let mut mode = DriveMode::Powered;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--rate-deg-s" => yaw_rate = args.next().ok_or("missing yaw rate")?.parse()?,
            "--rpm" => rpm = args.next().ok_or("missing RPM")?.parse()?,
            "--coast" => mode = DriveMode::Coast,
            "--loss-seconds" => {
                let seconds: f64 = args.next().ok_or("missing loss time constant")?.parse()?;
                if !seconds.is_finite() || seconds <= 0.0 {
                    return Err("loss time constant must be finite and positive".into());
                }
                config.motor.spin_damping = config.rotor.axial / seconds;
            }
            "--help" | "-h" => {
                println!(
                    "turn_indicator [--rate-deg-s 3] [--rpm 12000] [--coast] [--loss-seconds 30]\n\
                    Prints an 8 s experiment at 0.5 s intervals: 1 s yaw-rate ramp, 4 s hold, 1 s ramp down, 2 s settling.\n\
                    RPM sets initial and target relative spin; coast disables the motor. All dynamics use SI units."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown option {arg}; use --help").into()),
        }
    }
    let initial = InitialState {
        joints: JointState {
            rates: [0.0, rpm_to_rad_s(rpm), 0.0],
            ..JointState::default()
        },
        drive: DriveState {
            mode,
            target_spin: rpm_to_rad_s(rpm),
        },
        housing: HousingMotion::YawRate(YawRateProfile {
            rate_rad_s: yaw_rate.to_radians(),
            ..YawRateProfile::default()
        }),
        ..InitialState::default()
    };
    let mut sim = Simulation::new(config, initial)?;
    let stdout = io::stdout();
    let mut out = io::BufWriter::new(stdout.lock());
    writeln!(
        out,
        "time_s,housing_yaw_rate_deg_s,gimbal_angle_deg,relative_rpm,axial_momentum_nms,energy_balance_error_j"
    )?;
    for sample in 0..=16 {
        let s = if sample == 0 {
            sim.snapshot()
        } else {
            sim.advance_steps(8192)?.current
        };
        writeln!(
            out,
            "{:.3},{:.6},{:.9},{:.6},{:.12},{:.6e}",
            s.time_seconds,
            s.housing.angular_velocity[2].to_degrees(),
            s.joints.angles[0].to_degrees(),
            rad_s_to_rpm(s.joints.rates[1]),
            s.diagnostics.axial_momentum,
            s.energy_balance_residual
        )?;
    }
    out.flush()?;
    Ok(())
}
