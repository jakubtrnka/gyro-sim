use crate::SimulationError;
use crate::math::{self, IDENTITY, axis_rotation, normalize_quaternion};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct HousingControllerConfig {
    pub position_gain: f64,
    pub rate_time_constant: f64,
    pub max_rate: f64,
    pub max_acceleration: f64,
}

impl Default for HousingControllerConfig {
    fn default() -> Self {
        Self {
            position_gain: 4.0,
            rate_time_constant: 0.2,
            max_rate: std::f64::consts::FRAC_PI_2,
            max_acceleration: std::f64::consts::TAU,
        }
    }
}

impl HousingControllerConfig {
    pub(crate) fn validate(self) -> Result<(), SimulationError> {
        if ![
            self.position_gain,
            self.rate_time_constant,
            self.max_rate,
            self.max_acceleration,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
        {
            return Err(SimulationError::InvalidConfiguration(
                "housing controller parameters must be finite and positive",
            ));
        }
        Ok(())
    }

    pub(crate) fn acceleration(
        self,
        orientation: [f64; 4],
        rate: [f64; 3],
        target: [f64; 4],
    ) -> Result<[f64; 3], SimulationError> {
        let error = math::rotation_error(target, orientation)?;
        let command = math::clip_norm(math::scale(error, self.position_gain), self.max_rate);
        let acceleration = math::clip_norm(
            math::scale(
                math::add(command, math::scale(rate, -1.0)),
                1.0 / self.rate_time_constant,
            ),
            self.max_acceleration,
        );
        if !acceleration.iter().all(|x| x.is_finite()) {
            return Err(SimulationError::numerical("housing controller overflow"));
        }
        Ok(acceleration)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub enum Axis {
    Roll,
    Pitch,
    Yaw,
}

impl Axis {
    pub(crate) fn vector(self) -> [f64; 3] {
        match self {
            Self::Roll => math::X,
            Self::Pitch => math::Y,
            Self::Yaw => math::Z,
        }
    }
}

/// Quintic excursion from zero to an angle, a hold, then a smooth return to zero.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct ExcursionProfile {
    pub axis: Axis,
    pub angle_rad: f64,
    pub move_seconds: f64,
    pub hold_seconds: f64,
}

impl Default for ExcursionProfile {
    fn default() -> Self {
        Self {
            axis: Axis::Roll,
            angle_rad: 20.0_f64.to_radians(),
            move_seconds: 2.0,
            hold_seconds: 2.0,
        }
    }
}

impl ExcursionProfile {
    pub fn duration(self) -> f64 {
        2.0 * self.move_seconds + self.hold_seconds
    }

    fn validate(self) -> Result<(), SimulationError> {
        if self.move_seconds <= 0.0
            || self.hold_seconds < 0.0
            || ![
                self.angle_rad,
                self.move_seconds,
                self.hold_seconds,
                self.duration(),
                60.0 * self.angle_rad / self.move_seconds / self.move_seconds,
            ]
            .iter()
            .all(|x| x.is_finite())
        {
            return Err(SimulationError::InvalidInput(
                "excursion needs finite values, positive move time and nonnegative hold time",
            ));
        }
        Ok(())
    }

    fn sample(self, t: f64) -> HousingSample {
        let (s, sign, offset) = if t < self.move_seconds {
            (t / self.move_seconds, 1.0, 0.0)
        } else if t < self.move_seconds + self.hold_seconds {
            (0.0, 0.0, 1.0)
        } else if t < self.duration() {
            (
                (t - self.move_seconds - self.hold_seconds) / self.move_seconds,
                -1.0,
                1.0,
            )
        } else {
            (0.0, 0.0, 0.0)
        };
        let angle = self.angle_rad
            * (offset + sign * (10.0 * s.powi(3) - 15.0 * s.powi(4) + 6.0 * s.powi(5)));
        let rate = self.angle_rad * sign / self.move_seconds
            * (30.0 * s.powi(2) - 60.0 * s.powi(3) + 30.0 * s.powi(4));
        let acceleration = self.angle_rad * sign / self.move_seconds / self.move_seconds
            * (60.0 * s - 180.0 * s.powi(2) + 120.0 * s.powi(3));
        HousingSample {
            orientation: axis_rotation(self.axis.vector(), angle),
            angular_velocity: math::scale(self.axis.vector(), rate),
            angular_acceleration: math::scale(self.axis.vector(), acceleration),
        }
    }
}

/// Consistent instantaneous prescribed housing kinematics in world axes.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct HousingSample {
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
    pub angular_acceleration: [f64; 3],
}

impl Default for HousingSample {
    fn default() -> Self {
        Self {
            orientation: IDENTITY,
            angular_velocity: [0.0; 3],
            angular_acceleration: [0.0; 3],
        }
    }
}

impl HousingSample {
    pub fn normalized(mut self) -> Result<Self, SimulationError> {
        self.orientation = normalize_quaternion(self.orientation)?;
        if !self
            .angular_velocity
            .iter()
            .chain(self.angular_acceleration.iter())
            .all(|v| v.is_finite())
        {
            return Err(SimulationError::InvalidInput(
                "housing rates and accelerations must be finite",
            ));
        }
        Ok(self)
    }
}

/// Smooth yaw-rate ramp, constant-rate hold, ramp down, then rest.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub struct YawRateProfile {
    pub rate_rad_s: f64,
    pub ramp_seconds: f64,
    pub hold_seconds: f64,
    pub initial_yaw_rad: f64,
}

impl Default for YawRateProfile {
    fn default() -> Self {
        Self {
            rate_rad_s: 3.0_f64.to_radians(),
            ramp_seconds: 1.0,
            hold_seconds: 4.0,
            initial_yaw_rad: 0.0,
        }
    }
}

impl YawRateProfile {
    pub fn duration(self) -> f64 {
        2.0 * self.ramp_seconds + self.hold_seconds
    }

    fn validate(self) -> Result<(), SimulationError> {
        if ![
            self.rate_rad_s,
            self.ramp_seconds,
            self.hold_seconds,
            self.initial_yaw_rad,
            self.duration(),
            self.rate_rad_s / self.ramp_seconds,
            self.initial_yaw_rad + self.rate_rad_s * (self.ramp_seconds + self.hold_seconds),
        ]
        .iter()
        .all(|v| v.is_finite())
            || self.ramp_seconds <= 0.0
            || self.hold_seconds < 0.0
        {
            return Err(SimulationError::InvalidInput(
                "yaw profile needs finite values, positive ramp time, and nonnegative hold time",
            ));
        }
        Ok(())
    }

    fn sample(self, t: f64) -> HousingSample {
        let r = self.ramp_seconds;
        let h = self.hold_seconds;
        let w = self.rate_rad_s;
        let (angle, rate, acceleration) = if t < r {
            let s = t / r;
            (
                w * r * (s.powi(3) - 0.5 * s.powi(4)),
                w * (3.0 * s * s - 2.0 * s.powi(3)),
                w / r * (6.0 * s - 6.0 * s * s),
            )
        } else if t < r + h {
            (w * (0.5 * r + t - r), w, 0.0)
        } else if t < self.duration() {
            let s = (t - r - h) / r;
            (
                w * (0.5 * r + h + r * (s - s.powi(3) + 0.5 * s.powi(4))),
                w * (1.0 - 3.0 * s * s + 2.0 * s.powi(3)),
                w / r * (-6.0 * s + 6.0 * s * s),
            )
        } else {
            (w * (r + h), 0.0, 0.0)
        };
        HousingSample {
            orientation: axis_rotation([0.0, 0.0, 1.0], self.initial_yaw_rad + angle),
            angular_velocity: [0.0, 0.0, rate],
            angular_acceleration: [0.0, 0.0, acceleration],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub enum HousingMotion {
    Fixed {
        orientation: [f64; 4],
    },
    YawRate(YawRateProfile),
    Excursion(ExcursionProfile),
    /// Integrated by Simulation; `sample` is only defined at time zero.
    Manual {
        orientation: [f64; 4],
        angular_velocity: [f64; 3],
        target: [f64; 4],
    },
}

impl Default for HousingMotion {
    fn default() -> Self {
        Self::Fixed {
            orientation: IDENTITY,
        }
    }
}

impl HousingMotion {
    pub fn sample(self, time_seconds: f64) -> Result<HousingSample, SimulationError> {
        if !time_seconds.is_finite() || time_seconds < 0.0 {
            return Err(SimulationError::InvalidInput(
                "sample time must be finite and nonnegative",
            ));
        }
        let sample = match self {
            Self::Fixed { orientation } => HousingSample {
                orientation,
                ..HousingSample::default()
            },
            Self::YawRate(profile) => {
                profile.validate()?;
                profile.sample(time_seconds)
            }
            Self::Excursion(profile) => {
                profile.validate()?;
                profile.sample(time_seconds)
            }
            Self::Manual {
                orientation,
                angular_velocity,
                target,
            } => {
                normalize_quaternion(target)?;
                if time_seconds != 0.0 {
                    return Err(SimulationError::InvalidInput(
                        "manual motion must be integrated by Simulation",
                    ));
                }
                HousingSample {
                    orientation,
                    angular_velocity,
                    angular_acceleration: math::ZERO,
                }
            }
        };
        sample.normalized()
    }

    pub(crate) fn validate_steps(self, dt: f64) -> Result<(), SimulationError> {
        self.sample(0.0)?;
        let durations = match self {
            Self::YawRate(profile) => [profile.ramp_seconds, profile.hold_seconds],
            Self::Excursion(profile) => [profile.move_seconds, profile.hold_seconds],
            _ => [0.0; 2],
        };
        for duration in durations {
            let steps = duration / dt;
            if !steps.is_finite()
                || (duration > 0.0 && steps < 1.0)
                || (steps - steps.round()).abs() > 1e-8
            {
                return Err(SimulationError::InvalidInput(
                    "profile durations must be whole multiples of the simulation step",
                ));
            }
        }
        Ok(())
    }
}

/// Script starts relative to the current housing orientation, at rest.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(deny_unknown_fields))]
pub enum Maneuver {
    Yaw {
        rate_rad_s: f64,
        ramp_seconds: f64,
        hold_seconds: f64,
    },
    Excursion(ExcursionProfile),
}

impl Maneuver {
    pub(crate) fn motion(self) -> HousingMotion {
        match self {
            Self::Yaw {
                rate_rad_s,
                ramp_seconds,
                hold_seconds,
            } => HousingMotion::YawRate(YawRateProfile {
                rate_rad_s,
                ramp_seconds,
                hold_seconds,
                initial_yaw_rad: 0.0,
            }),
            Self::Excursion(profile) => HousingMotion::Excursion(profile),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum Controller {
    Prescribed {
        motion: HousingMotion,
        epoch: f64,
        base: [f64; 4],
    },
    Manual {
        target: [f64; 4],
    },
}

impl Controller {
    pub(crate) fn initial(motion: HousingMotion) -> Result<Self, SimulationError> {
        Ok(match motion {
            HousingMotion::Manual { target, .. } => Self::Manual {
                target: math::canonical(target)?,
            },
            _ => Self::Prescribed {
                motion,
                epoch: 0.0,
                base: IDENTITY,
            },
        })
    }

    pub(crate) fn target(self) -> Option<[f64; 4]> {
        match self {
            Self::Manual { target } => Some(target),
            _ => None,
        }
    }

    pub(crate) fn sample(
        self,
        time: f64,
        orientation: [f64; 4],
        rate: [f64; 3],
        config: HousingControllerConfig,
    ) -> Result<HousingSample, SimulationError> {
        match self {
            Self::Manual { target } => {
                let orientation = normalize_quaternion(orientation)?;
                HousingSample {
                    orientation,
                    angular_velocity: rate,
                    angular_acceleration: config.acceleration(orientation, rate, target)?,
                }
                .normalized()
            }
            Self::Prescribed {
                motion,
                epoch,
                base,
            } => {
                let local = motion.sample(time - epoch)?;
                let rotation = math::rotation_matrix(base);
                HousingSample {
                    orientation: math::quat_mul(base, local.orientation),
                    angular_velocity: math::mat_vec(rotation, local.angular_velocity),
                    angular_acceleration: math::mat_vec(rotation, local.angular_acceleration),
                }
                .normalized()
            }
        }
    }
}
