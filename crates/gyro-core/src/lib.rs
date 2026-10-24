//! Native, deterministic gyroscope dynamics with no browser or clock dependency.
//!
//! This first implementation supports the one-gimbal turn indicator. Coordinates
//! are `[gimbal_angle, rotor_phase]`; all quantities use SI units. Aircraft axes
//! are x forward, y right, z down. Quaternions use `[x, y, z, w]` Hamilton order
//! and map local component coordinates to world coordinates.
//!
//! ```
//! use gyro_core::{InitialState, Simulation, SimulationConfig};
//!
//! let mut sim = Simulation::new(SimulationConfig::default(), InitialState::default())?;
//! let before = sim.snapshot();
//! let after = sim.step()?;
//! assert_eq!(after.step_index, before.step_index + 1);
//! # Ok::<(), gyro_core::SimulationError>(())
//! ```

mod dynamics;
mod housing;
mod math;
mod simulation;
mod types;

pub use dynamics::Model;
pub use housing::{HousingMotion, HousingSample, YawRateProfile};
pub use simulation::Simulation;
pub use types::{
    AdvanceResult, Diagnostics, DriveMode, DriveState, Evaluation, InitialState, JointState,
    MotorConfig, RotorInertia, SimulationConfig, SimulationError, Snapshot, Work,
};

/// Convert bearing-relative RPM to radians per second.
pub fn rpm_to_rad_s(rpm: f64) -> f64 {
    rpm * (std::f64::consts::TAU / 60.0)
}

/// Convert bearing-relative radians per second to RPM.
pub fn rad_s_to_rpm(rate: f64) -> f64 {
    rate * (60.0 / std::f64::consts::TAU)
}
