use crate::math::{self, Mat3, Vec3, ZERO, add, cross, dot, mat_vec, scale};
use crate::{
    Diagnostics, DriveMode, DriveState, Evaluation, HousingSample, InstrumentReading, JointState,
    Preset, SimulationConfig, SimulationError,
};

/// Validated, immutable parameters and instantaneous equations of motion.
#[derive(Clone, Debug)]
pub struct Model {
    config: SimulationConfig,
}

#[derive(Clone, Copy)]
struct Body {
    inertia: Mat3,
    omega: Vec3,
    /// Angular acceleration with joint accelerations set to zero.
    bias_acceleration: Vec3,
    /// Columns of the world angular Jacobian; unused columns are zero.
    axes: [Vec3; 3],
}

impl Model {
    pub fn new(config: SimulationConfig) -> Result<Self, SimulationError> {
        config.validate()?;
        Ok(Self { config })
    }

    pub fn config(&self) -> SimulationConfig {
        self.config
    }

    /// General angular-Jacobian equations (mechanics equation 27), shared by all
    /// presets. Direct callers must supply mutually consistent housing kinematics.
    pub fn evaluate(
        &self,
        state: JointState,
        housing: HousingSample,
        drive: DriveState,
    ) -> Result<Evaluation, SimulationError> {
        let config = self.config;
        state.validate(config.preset)?;
        drive.validate()?;
        let housing = housing.normalized()?;
        let n = config.preset.joint_count();
        let spin = config.preset.spin_index();
        let mut orientations = [math::IDENTITY; 3];
        let mut axes = [ZERO; 3];
        let mut orientation = housing.orientation;
        let mut omega = housing.angular_velocity;
        let mut bias = housing.angular_acceleration;
        let empty = Body {
            inertia: [[0.0; 3]; 3],
            omega: ZERO,
            bias_acceleration: ZERO,
            axes: [ZERO; 3],
        };
        let mut bodies = [empty; 3];
        for (i, local_axis) in config.preset.axes().iter().enumerate().take(n) {
            let axis = mat_vec(math::rotation_matrix(orientation), *local_axis);
            axes[i] = axis;
            // Each axis rotates with its parent, before adding its own joint rate.
            bias = add(bias, scale(cross(omega, axis), state.rates[i]));
            omega = add(omega, scale(axis, state.rates[i]));
            orientation = math::quat_mul(
                orientation,
                math::axis_rotation(*local_axis, state.angles[i]),
            );
            orientations[i] = orientation;
            let inertia = if i == spin {
                math::axial_inertia(axis, config.rotor.axial, config.rotor.transverse)
            } else {
                math::rotate_diagonal(
                    math::rotation_matrix(orientation),
                    if i == 0 {
                        config.gimbal_inertia
                    } else {
                        config.inner_gimbal_inertia
                    },
                )
            };
            bodies[i] = Body {
                inertia,
                omega,
                bias_acceleration: bias,
                axes,
            };
        }
        let spindle = axes[spin];
        let spin_rate = state.rates[spin];
        let motor_torque = match drive.mode {
            DriveMode::Coast => 0.0,
            DriveMode::Powered => (config.motor.gain * (drive.target_spin - spin_rate))
                .clamp(-config.motor.torque_limit, config.motor.torque_limit),
        };
        let spin_loss_torque = -config.motor.spin_damping * spin_rate;
        let stiffness = config.stiffness();
        let damping = config.damping();
        let mut rhs =
            std::array::from_fn(|i| -stiffness[i] * state.angles[i] - damping[i] * state.rates[i]);
        rhs[spin] = motor_torque + spin_loss_torque;
        let mut mass = [[0.0; 3]; 3];
        for body in bodies.iter().take(n) {
            let momentum = mat_vec(body.inertia, body.omega);
            let bias_torque = add(
                mat_vec(body.inertia, body.bias_acceleration),
                cross(body.omega, momentum),
            );
            for (j, axis) in body.axes.iter().enumerate().take(n) {
                rhs[j] -= dot(*axis, bias_torque);
                for (k, other_axis) in body.axes.iter().enumerate().take(n).skip(j) {
                    let contribution = dot(*axis, mat_vec(body.inertia, *other_axis));
                    mass[j][k] += contribution;
                    if j != k {
                        mass[k][j] += contribution;
                    }
                }
            }
        }
        let (accelerations, mass_condition) = math::solve_spd(mass, rhs, n)?;
        let mut kinetic_energy = 0.0;
        let mut assembly_momentum = ZERO;
        let mut housing_torque = ZERO;
        let mut generalized_momenta = ZERO;
        for body in bodies.iter().take(n) {
            let momentum = mat_vec(body.inertia, body.omega);
            kinetic_energy += 0.5 * dot(body.omega, momentum);
            assembly_momentum = add(assembly_momentum, momentum);
            let mut acceleration = body.bias_acceleration;
            for (i, axis) in body.axes.iter().enumerate().take(n) {
                acceleration = add(acceleration, scale(*axis, accelerations[i]));
                generalized_momenta[i] += dot(*axis, momentum);
            }
            // Internal torques cancel in the sum dH/dt: the remainder is housing torque.
            housing_torque = add(
                housing_torque,
                add(
                    mat_vec(body.inertia, acceleration),
                    cross(body.omega, momentum),
                ),
            );
        }
        let absolute_axial_speed = dot(bodies[spin].omega, spindle);
        let diagnostics = Diagnostics {
            kinetic_energy,
            spring_energy: (0..spin)
                .map(|i| 0.5 * stiffness[i] * state.angles[i].powi(2))
                .sum(),
            axial_momentum: config.rotor.axial * absolute_axial_speed,
            absolute_axial_speed,
            spindle_direction: spindle,
            rotor_angular_momentum: mat_vec(bodies[spin].inertia, bodies[spin].omega),
            assembly_angular_momentum: assembly_momentum,
            housing_torque,
            motor_torque,
            spin_loss_torque,
            housing_power: dot(housing_torque, housing.angular_velocity),
            motor_power: motor_torque * spin_rate,
            dissipated_power: (0..spin)
                .map(|i| damping[i] * state.rates[i].powi(2))
                .sum::<f64>()
                + config.motor.spin_damping * spin_rate.powi(2),
            mass_condition,
            generalized_momenta,
            near_axis_alignment: n == 3 && state.angles[1].cos().abs() < 0.05,
            poorly_conditioned: mass_condition > 1e10,
        };
        let scalars = [
            diagnostics.kinetic_energy,
            diagnostics.spring_energy,
            diagnostics.total_energy(),
            diagnostics.axial_momentum,
            diagnostics.absolute_axial_speed,
            motor_torque,
            spin_loss_torque,
            diagnostics.housing_power,
            diagnostics.motor_power,
            diagnostics.dissipated_power,
        ];
        let vectors = [
            spindle,
            diagnostics.rotor_angular_momentum,
            assembly_momentum,
            housing_torque,
            generalized_momenta,
        ];
        if !scalars
            .iter()
            .chain(vectors.iter().flatten())
            .chain(orientations.iter().flatten())
            .all(|x| x.is_finite())
        {
            return Err(SimulationError::numerical("non-finite dynamics or energy"));
        }
        Ok(Evaluation {
            accelerations,
            component_orientations: orientations,
            diagnostics,
        })
    }

    pub(crate) fn instrument(
        &self,
        joints: JointState,
        housing: HousingSample,
        value: Evaluation,
        heading_offset: f64,
    ) -> Result<InstrumentReading, SimulationError> {
        let reading = match self.config.preset {
            Preset::TurnIndicator => InstrumentReading::TurnIndicator {
                deflection: joints.angles[0],
                indicated_rate: self.config.spring_stiffness * joints.angles[0]
                    / self.config.nominal_axial_momentum,
            },
            Preset::Directional => InstrumentReading::Directional {
                heading: (heading_offset - joints.angles[0]).rem_euclid(std::f64::consts::TAU),
            },
            Preset::Attitude => {
                // Geometric spindle, not momentum direction (which reverses with spin).
                let down = mat_vec(
                    math::rotation_matrix(math::conjugate(housing.orientation)),
                    value.diagnostics.spindle_direction,
                );
                InstrumentReading::Attitude {
                    pitch: (-down[0]).clamp(-1.0, 1.0).asin(),
                    bank: (down[1].hypot(down[2]) >= 1e-6).then(|| down[1].atan2(down[2])),
                }
            }
        };
        let finite = match reading {
            InstrumentReading::TurnIndicator {
                deflection,
                indicated_rate,
            } => deflection.is_finite() && indicated_rate.is_finite(),
            InstrumentReading::Directional { heading } => heading.is_finite(),
            InstrumentReading::Attitude { pitch, bank } => {
                pitch.is_finite() && bank.is_none_or(f64::is_finite)
            }
        };
        if !finite {
            return Err(SimulationError::numerical("instrument reading overflow"));
        }
        Ok(reading)
    }
}
