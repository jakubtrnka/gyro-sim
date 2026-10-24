use crate::{HousingControllerConfig, HousingMotion, HousingSample, rpm_to_rad_s};
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

/// Serial joint axes are expressed in each joint's parent frame.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Preset {
    #[default]
    TurnIndicator,
    Attitude,
    Directional,
}

impl Preset {
    pub fn joint_count(self) -> usize {
        match self {
            Self::TurnIndicator => 2,
            _ => 3,
        }
    }

    pub fn spin_index(self) -> usize {
        self.joint_count() - 1
    }

    pub(crate) fn axes(self) -> [[f64; 3]; 3] {
        use crate::math::{X, Y, Z, ZERO};
        match self {
            Self::TurnIndicator => [X, Y, ZERO],
            Self::Attitude => [X, Y, Z],
            Self::Directional => [Z, X, Y],
        }
    }
}

/// Mechanism and interaction parameters in SI units; immutable during a run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationConfig {
    pub preset: Preset,
    pub rotor: RotorInertia,
    /// Sole gimbal for TurnIndicator; outer gimbal for the two-gimbal presets.
    pub gimbal_inertia: [f64; 3],
    pub inner_gimbal_inertia: [f64; 3],
    pub spring_stiffness: f64,
    pub gimbal_damping: f64,
    pub inner_spring_stiffness: f64,
    pub inner_gimbal_damping: f64,
    pub motor: MotorConfig,
    pub housing_controller: HousingControllerConfig,
    /// Positive calibration momentum for the turn-rate dial, not measured spin.
    pub nominal_axial_momentum: f64,
    pub step_seconds: f64,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        let axial = 3.125e-5;
        let transverse = 1.5625e-5;
        let spring_stiffness = axial * rpm_to_rad_s(12_000.0) * 0.3;
        Self {
            preset: Preset::TurnIndicator,
            rotor: RotorInertia { axial, transverse },
            gimbal_inertia: [2e-5; 3],
            inner_gimbal_inertia: [2e-5; 3],
            spring_stiffness,
            gimbal_damping: 2.0 * 0.7 * ((2e-5 + transverse) * spring_stiffness).sqrt(),
            inner_spring_stiffness: 0.0,
            inner_gimbal_damping: 0.0,
            motor: MotorConfig {
                gain: axial / 0.5,
                torque_limit: 0.005,
                spin_damping: 0.0,
            },
            housing_controller: HousingControllerConfig::default(),
            nominal_axial_momentum: axial * rpm_to_rad_s(12_000.0),
            step_seconds: 1.0 / 16_384.0,
        }
    }
}

impl SimulationConfig {
    pub fn for_preset(preset: Preset) -> Self {
        if preset == Preset::TurnIndicator {
            return Self::default();
        }
        Self {
            preset,
            spring_stiffness: 0.0,
            gimbal_damping: 0.0,
            step_seconds: 1.0 / 32_768.0,
            ..Self::default()
        }
    }

    pub(crate) fn stiffness(self) -> [f64; 3] {
        [
            self.spring_stiffness,
            if self.preset.joint_count() == 3 {
                self.inner_spring_stiffness
            } else {
                0.0
            },
            0.0,
        ]
    }

    pub(crate) fn damping(self) -> [f64; 3] {
        [
            self.gimbal_damping,
            if self.preset.joint_count() == 3 {
                self.inner_gimbal_damping
            } else {
                0.0
            },
            0.0,
        ]
    }

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
            || !inertia_valid(self.inner_gimbal_inertia)
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
            self.inner_spring_stiffness,
            self.inner_gimbal_damping,
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
        if !self.nominal_axial_momentum.is_finite() || self.nominal_axial_momentum <= 0.0 {
            return Err(SimulationError::InvalidConfiguration(
                "nominal momentum must be finite and positive",
            ));
        }
        self.housing_controller.validate()?;
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
    /// `[theta, psi, 0]` or `[alpha, beta, psi]`, unwrapped radians.
    pub angles: [f64; 3],
    /// Relative to each joint's parent. Unused TurnIndicator slot must be zero.
    pub rates: [f64; 3],
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            angles: [0.0; 3],
            rates: [0.0, rpm_to_rad_s(12_000.0), 0.0],
        }
    }
}

impl JointState {
    pub fn for_preset(preset: Preset) -> Self {
        let mut state = Self {
            angles: [0.0; 3],
            rates: [0.0; 3],
        };
        state.rates[preset.spin_index()] = rpm_to_rad_s(12_000.0);
        state
    }

    pub fn turn_indicator(angles: [f64; 2], rates: [f64; 2]) -> Self {
        Self {
            angles: [angles[0], angles[1], 0.0],
            rates: [rates[0], rates[1], 0.0],
        }
    }

    pub(crate) fn validate(self, preset: Preset) -> Result<(), SimulationError> {
        if !self.is_finite() {
            return Err(SimulationError::InvalidInput(
                "joint angles and rates must be finite",
            ));
        }
        if preset == Preset::TurnIndicator && (self.angles[2] != 0.0 || self.rates[2] != 0.0) {
            return Err(SimulationError::InvalidInput(
                "unused turn-indicator joint must be zero",
            ));
        }
        Ok(())
    }

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
    /// Display offset only: does not torque or reset the DG mechanism.
    pub heading_offset: f64,
}

impl InitialState {
    pub fn for_preset(preset: Preset) -> Self {
        Self {
            joints: JointState::for_preset(preset),
            ..Self::default()
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InstrumentReading {
    TurnIndicator {
        deflection: f64,
        indicated_rate: f64,
    },
    Attitude {
        pitch: f64,
        bank: Option<f64>,
    },
    Directional {
        heading: f64,
    },
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
    pub generalized_momenta: [f64; 3],
    pub near_axis_alignment: bool,
    pub poorly_conditioned: bool,
}

impl Diagnostics {
    pub fn total_energy(self) -> f64 {
        self.kinetic_energy + self.spring_energy
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Evaluation {
    pub accelerations: [f64; 3],
    /// World-from-body, in chain order; unused slot is identity. Quaternions: xyzw.
    pub component_orientations: [[f64; 4]; 3],
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
    pub preset: Preset,
    /// Number of completed steps. Physical time is exactly this index times dt.
    pub step_index: u64,
    pub time_seconds: f64,
    pub joints: JointState,
    pub drive: DriveState,
    pub housing: HousingSample,
    pub component_orientations: [[f64; 4]; 3],
    pub housing_target: Option<[f64; 4]>,
    pub housing_rate_exceeded: bool,
    pub heading_offset: f64,
    pub instrument: InstrumentReading,
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
