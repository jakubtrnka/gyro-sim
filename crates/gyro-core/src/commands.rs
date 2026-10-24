use crate::{DriveMode, InitialState, Maneuver, SimulationConfig, SimulationError};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// World-from-housing xyzw target. Never overwrites the physical pose.
    SetHousingTarget([f64; 4]),
    /// Target the current pose and decelerate smoothly. Ordinary mouse release
    /// should send no command: the last target remains in place.
    StopHousing,
    /// Relative to the current pose; requires housing at rest when executed.
    StartManeuver(Maneuver),
    SetTargetSpin(f64),
    SetDriveMode(DriveMode),
    SetHeadingOffset(f64),
}

impl Command {
    pub(crate) fn validated(self, dt: f64) -> Result<Self, SimulationError> {
        match self {
            Self::SetHousingTarget(q) => {
                // Preserve the input bits in the log. Normalize at execution, so
                // replay cannot re-normalize an already rounded quaternion.
                crate::math::canonical(q)?;
                Ok(self)
            }
            Self::StartManeuver(maneuver) => {
                maneuver.motion().validate_steps(dt)?;
                Ok(self)
            }
            Self::SetTargetSpin(v) | Self::SetHeadingOffset(v) if !v.is_finite() => Err(
                SimulationError::InvalidInput("command value must be finite"),
            ),
            Self::SetHeadingOffset(v) => {
                Ok(Self::SetHeadingOffset(v.rem_euclid(std::f64::consts::TAU)))
            }
            _ => Ok(self),
        }
    }
}

/// Executed just before step `step_index`, ordered by `sequence` within that step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandRecord {
    pub step_index: u64,
    pub sequence: u32,
    pub command: Command,
}

/// Owned, versioned native replay. Serialization belongs at the WASM boundary.
/// Includes future/pending commands as well as those already applied.
#[derive(Clone, Debug, PartialEq)]
pub struct Replay {
    pub version: u32,
    pub config: SimulationConfig,
    pub initial: InitialState,
    pub commands: Vec<CommandRecord>,
    pub end_step: u64,
}

impl Replay {
    pub const VERSION: u32 = 1;
}
