use crate::SimulationError;
use crate::math::{IDENTITY, axis_rotation, normalize_quaternion};

/// Consistent instantaneous prescribed housing kinematics in world axes.
#[derive(Clone, Copy, Debug, PartialEq)]
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
pub enum HousingMotion {
    Fixed { orientation: [f64; 4] },
    YawRate(YawRateProfile),
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
        };
        sample.normalized()
    }

    pub(crate) fn validate_steps(self, dt: f64) -> Result<(), SimulationError> {
        self.sample(0.0)?;
        if let Self::YawRate(profile) = self {
            for duration in [profile.ramp_seconds, profile.hold_seconds] {
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
        }
        Ok(())
    }
}
