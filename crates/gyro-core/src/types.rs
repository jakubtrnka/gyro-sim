use crate::{HousingMotion, HousingSample, rpm_to_rad_s};
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SimulationError {
    InvalidConfiguration(&'static str),
    InvalidInput(&'static str),
    NumericalFailure {
        step: Option<u64>,
        reason: &'static str,
    },
}

impl SimulationError {
    pub(crate) fn numerical(reason: &'static str) -> Self {
        Self::NumericalFailure { step: None, reason }
    }

    pub(crate) fn at_step(self, step: u64) -> Self {
        match self {
            Self::NumericalFailure { reason, .. } => Self::NumericalFailure {
                step: Some(step),
                reason,
            },
            other => other,
        }
    }
}

impl fmt::Display for SimulationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(reason) => write!(f, "invalid configuration: {reason}"),
            Self::InvalidInput(reason) => write!(f, "invalid input: {reason}"),
            Self::NumericalFailure { step, reason } => match step {
                Some(step) => write!(f, "numerical failure at step {step}: {reason}"),
                None => write!(f, "numerical failure: {reason}"),
            },
        }
    }
}

impl std::error::Error for SimulationError {}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotorInertia {
    pub axial: f64,
    pub transverse: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorConfig {
    pub gain: f64,
    pub torque_limit: f64,
    pub spin_damping: f64,
}

/// Parameters of the one-gimbal x/y turn-indicator mechanism, in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationConfig {
    pub rotor: RotorInertia,
    pub gimbal_inertia: [f64; 3],
    pub spring_stiffness: f64,
    pub gimbal_damping: f64,
    pub motor: MotorConfig,
    pub step_seconds: f64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        let axial = 3.125e-5;
        let transverse = 1.5625e-5;
        let spring_stiffness = axial * rpm_to_rad_s(12_000.0) * 0.3;
        Self {
            rotor: RotorInertia { axial, transverse },
            gimbal_inertia: [2e-5; 3],
            spring_stiffness,
            gimbal_damping: 2.0 * 0.7 * ((2e-5 + transverse) * spring_stiffness).sqrt(),
            motor: MotorConfig {
                gain: axial / 0.5,
                torque_limit: 0.005,
                spin_damping: 0.0,
            },
            step_seconds: 1.0 / 16_384.0,
        }
    }
}

impl SimulationConfig {
    pub fn validate(&self) -> Result<(), SimulationError> {
        let inertia_valid = |i: [f64; 3]| {
            if !i.iter().all(|v| v.is_finite() && *v > 0.0) {
                return false;
            }
            let scale = i.iter().copied().fold(0.0, f64::max);
            let scaled = i.map(|v| v / scale);
            (0..3).all(|j| {
                scaled[j] <= scaled[(j + 1) % 3] + scaled[(j + 2) % 3] + 16.0 * f64::EPSILON
            })
        };
        if !inertia_valid(self.gimbal_inertia)
            || !inertia_valid([
                self.rotor.transverse,
                self.rotor.axial,
                self.rotor.transverse,
            ])
        {
            return Err(SimulationError::InvalidConfiguration(
                "inertias must be positive, finite, and satisfy triangle inequalities",
            ));
        }
        if ![
            self.spring_stiffness,
            self.gimbal_damping,
            self.motor.gain,
            self.motor.torque_limit,
            self.motor.spin_damping,
        ]
        .iter()
        .all(|v| v.is_finite() && *v >= 0.0)
        {
            return Err(SimulationError::InvalidConfiguration(
                "stiffness, damping, motor gain and torque limit must be finite and nonnegative",
            ));
        }
        if !self.step_seconds.is_finite() || self.step_seconds <= 0.0 {
            return Err(SimulationError::InvalidConfiguration(
                "step size must be finite and positive",
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveMode {
    Powered,
    Coast,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DriveState {
    pub mode: DriveMode,
    /// Bearing-relative target angular velocity in rad/s, not RPM.
    pub target_spin: f64,
}

impl Default for DriveState {
    fn default() -> Self {
        Self {
            mode: DriveMode::Powered,
            target_spin: rpm_to_rad_s(12_000.0),
        }
    }
}

impl DriveState {
    pub(crate) fn validate(self) -> Result<(), SimulationError> {
        if self.target_spin.is_finite() {
            Ok(())
        } else {
            Err(SimulationError::InvalidInput("target spin must be finite"))
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointState {
    /// `[theta, psi]`, unwrapped radians.
    pub angles: [f64; 2],
    /// `[theta_dot, psi_dot]`, radians per second relative to each joint's parent.
    pub rates: [f64; 2],
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            angles: [0.0; 2],
            rates: [0.0, rpm_to_rad_s(12_000.0)],
        }
    }
}

impl JointState {
    pub(crate) fn is_finite(self) -> bool {
        self.angles
            .iter()
            .chain(self.rates.iter())
            .all(|x| x.is_finite())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InitialState {
    pub joints: JointState,
    pub drive: DriveState,
    /// Prescribed motion starts at simulation time zero, including after reset.
    pub housing: HousingMotion,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Diagnostics {
    pub kinetic_energy: f64,
    pub spring_energy: f64,
    pub axial_momentum: f64,
    pub absolute_axial_speed: f64,
    pub spindle_direction: [f64; 3],
    pub rotor_angular_momentum: [f64; 3],
    pub assembly_angular_momentum: [f64; 3],
    pub housing_torque: [f64; 3],
    pub motor_torque: f64,
    pub spin_loss_torque: f64,
    pub housing_power: f64,
    pub motor_power: f64,
    pub dissipated_power: f64,
    pub mass_condition: f64,
}

impl Diagnostics {
    pub fn total_energy(self) -> f64 {
        self.kinetic_energy + self.spring_energy
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Evaluation {
    pub accelerations: [f64; 2],
    /// World-from-gimbal and world-from-rotor quaternions in xyzw order.
    pub component_orientations: [[f64; 4]; 2],
    pub diagnostics: Diagnostics,
}

/// Work integrals in joules, evaluated with the same RK4 stages as the state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Work {
    pub housing: f64,
    pub motor: f64,
    pub dissipated: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snapshot {
    /// Number of completed steps. Physical time is exactly this index times dt.
    pub step_index: u64,
    pub time_seconds: f64,
    pub joints: JointState,
    pub drive: DriveState,
    pub housing: HousingSample,
    pub component_orientations: [[f64; 4]; 2],
    pub diagnostics: Diagnostics,
    pub work: Work,
    /// E - E_initial - (housing_work + motor_work - dissipated_work).
    pub energy_balance_residual: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdvanceResult {
    pub previous: Snapshot,
    pub current: Snapshot,
    pub steps_completed: u32,
}
