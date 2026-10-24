use crate::math::{self, Mat3, Vec3, X, Y, ZERO, add, cross, dot, mat_vec, scale};
use crate::{
    Diagnostics, DriveMode, DriveState, Evaluation, HousingSample, JointState, SimulationConfig,
    SimulationError,
};

/// Validated, immutable parameters and instantaneous equations of motion.
#[derive(Clone, Debug)]
pub struct Model {
    config: SimulationConfig,
}

struct Body {
    inertia: Mat3,
    omega: Vec3,
    /// Angular acceleration with joint accelerations set to zero.
    bias_acceleration: Vec3,
    /// Columns of the world angular Jacobian for [theta, psi].
    axes: [Vec3; 2],
}

impl Model {
    pub fn new(config: SimulationConfig) -> Result<Self, SimulationError> {
        config.validate()?;
        Ok(Self { config })
    }

    pub fn config(&self) -> SimulationConfig {
        self.config
    }

    /// Evaluate the general angular-Jacobian equations (mechanics equation 27).
    ///
    /// The housing sample may have arbitrary orientation, rate and acceleration.
    /// Callers supplying samples directly are responsible for trajectory consistency.
    pub fn evaluate(
        &self,
        state: JointState,
        housing: HousingSample,
        drive: DriveState,
    ) -> Result<Evaluation, SimulationError> {
        if !state.is_finite() {
            return Err(SimulationError::InvalidInput(
                "joint angles and rates must be finite",
            ));
        }
        drive.validate()?;
        let housing = housing.normalized()?;
        let config = self.config;
        let [theta, psi] = state.angles;
        let [theta_rate, spin_rate] = state.rates;
        let housing_rotation = math::rotation_matrix(housing.orientation);
        let gimbal_orientation = math::quat_mul(housing.orientation, math::axis_rotation(X, theta));
        let gimbal_rotation = math::rotation_matrix(gimbal_orientation);
        let rotor_orientation = math::quat_mul(gimbal_orientation, math::axis_rotation(Y, psi));

        let gimbal_axis = mat_vec(housing_rotation, X);
        let spindle = mat_vec(gimbal_rotation, Y);
        let gimbal_omega = add(housing.angular_velocity, scale(gimbal_axis, theta_rate));
        let rotor_omega = add(gimbal_omega, scale(spindle, spin_rate));
        // Each axis moves with its parent, not with an arbitrary fixed world axis.
        let gimbal_axis_rate = cross(housing.angular_velocity, gimbal_axis);
        let spindle_rate = cross(gimbal_omega, spindle);
        let gimbal_bias = add(
            housing.angular_acceleration,
            scale(gimbal_axis_rate, theta_rate),
        );
        let rotor_bias = add(gimbal_bias, scale(spindle_rate, spin_rate));

        let bodies = [
            Body {
                inertia: math::rotate_diagonal(gimbal_rotation, config.gimbal_inertia),
                omega: gimbal_omega,
                bias_acceleration: gimbal_bias,
                axes: [gimbal_axis, ZERO],
            },
            Body {
                inertia: math::axial_inertia(spindle, config.rotor.axial, config.rotor.transverse),
                omega: rotor_omega,
                bias_acceleration: rotor_bias,
                axes: [gimbal_axis, spindle],
            },
        ];

        let motor_torque = match drive.mode {
            DriveMode::Coast => 0.0,
            DriveMode::Powered => (config.motor.gain * (drive.target_spin - spin_rate))
                .clamp(-config.motor.torque_limit, config.motor.torque_limit),
        };
        let spin_loss_torque = -config.motor.spin_damping * spin_rate;
        let mut rhs = [
            -config.spring_stiffness * theta - config.gimbal_damping * theta_rate,
            motor_torque + spin_loss_torque,
        ];
        let mut mass = [[0.0; 2]; 2];

        for body in &bodies {
            let momentum = mat_vec(body.inertia, body.omega);
            let bias_torque = add(
                mat_vec(body.inertia, body.bias_acceleration),
                cross(body.omega, momentum),
            );
            for (j, axis) in body.axes.iter().enumerate() {
                rhs[j] -= dot(*axis, bias_torque);
                for (k, other_axis) in body.axes.iter().enumerate().skip(j) {
                    let contribution = dot(*axis, mat_vec(body.inertia, *other_axis));
                    mass[j][k] += contribution;
                    if j != k {
                        mass[k][j] += contribution;
                    }
                }
            }
        }
        let (accelerations, mass_condition) = math::solve_spd(mass, rhs)?;

        let mut kinetic_energy = 0.0;
        let mut assembly_momentum = ZERO;
        let mut housing_torque = ZERO;
        for body in &bodies {
            let momentum = mat_vec(body.inertia, body.omega);
            kinetic_energy += 0.5 * dot(body.omega, momentum);
            assembly_momentum = add(assembly_momentum, momentum);
            let mut acceleration = body.bias_acceleration;
            for (axis, rate) in body.axes.iter().zip(accelerations) {
                acceleration = add(acceleration, scale(*axis, rate));
            }
            // Sum dH_i/dt: internal joint torques cancel, leaving net housing torque.
            housing_torque = add(
                housing_torque,
                add(
                    mat_vec(body.inertia, acceleration),
                    cross(body.omega, momentum),
                ),
            );
        }
        let absolute_axial_speed = dot(rotor_omega, spindle);
        let diagnostics = Diagnostics {
            kinetic_energy,
            spring_energy: 0.5 * config.spring_stiffness * theta * theta,
            axial_momentum: config.rotor.axial * absolute_axial_speed,
            absolute_axial_speed,
            spindle_direction: spindle,
            rotor_angular_momentum: mat_vec(bodies[1].inertia, rotor_omega),
            assembly_angular_momentum: assembly_momentum,
            housing_torque,
            motor_torque,
            spin_loss_torque,
            housing_power: dot(housing_torque, housing.angular_velocity),
            motor_power: motor_torque * spin_rate,
            dissipated_power: config.gimbal_damping * theta_rate * theta_rate
                + config.motor.spin_damping * spin_rate * spin_rate,
            mass_condition,
        };
        let scalars = [
            diagnostics.kinetic_energy,
            diagnostics.spring_energy,
            diagnostics.total_energy(),
            diagnostics.axial_momentum,
            diagnostics.absolute_axial_speed,
            diagnostics.motor_torque,
            diagnostics.spin_loss_torque,
            diagnostics.housing_power,
            diagnostics.motor_power,
            diagnostics.dissipated_power,
        ];
        let vectors = [
            diagnostics.spindle_direction,
            diagnostics.rotor_angular_momentum,
            diagnostics.assembly_angular_momentum,
            diagnostics.housing_torque,
        ];
        if !scalars
            .iter()
            .chain(vectors.iter().flatten())
            .chain(gimbal_orientation.iter())
            .chain(rotor_orientation.iter())
            .all(|x| x.is_finite())
        {
            return Err(SimulationError::numerical("non-finite dynamics or energy"));
        }
        Ok(Evaluation {
            accelerations,
            component_orientations: [gimbal_orientation, rotor_orientation],
            diagnostics,
        })
    }
}
