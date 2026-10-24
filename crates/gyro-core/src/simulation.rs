use crate::{
    AdvanceResult, DriveMode, DriveState, Evaluation, HousingMotion, InitialState, JointState,
    Model, SimulationConfig, SimulationError, Snapshot, Work,
};

#[derive(Clone, Copy)]
struct Derivative {
    angles: [f64; 2],
    rates: [f64; 2],
    work: Work,
}

impl Derivative {
    fn from_evaluation(state: JointState, value: Evaluation) -> Self {
        Self {
            angles: state.rates,
            rates: value.accelerations,
            work: Work {
                housing: value.diagnostics.housing_power,
                motor: value.diagnostics.motor_power,
                dissipated: value.diagnostics.dissipated_power,
            },
        }
    }
}

fn shifted(state: JointState, derivative: Derivative, dt: f64) -> JointState {
    JointState {
        angles: std::array::from_fn(|i| state.angles[i] + dt * derivative.angles[i]),
        rates: std::array::from_fn(|i| state.rates[i] + dt * derivative.rates[i]),
    }
}

/// A fixed-step simulation. Calling no methods is equivalent to pausing.
///
/// Drive changes occur between steps. Timestamped command queues, manual housing
/// following, and the two-gimbal presets belong to the next implementation stage.
#[derive(Clone, Debug)]
pub struct Simulation {
    model: Model,
    housing: HousingMotion,
    current: Snapshot,
    initial_energy: f64,
}

impl Simulation {
    pub fn new(config: SimulationConfig, initial: InitialState) -> Result<Self, SimulationError> {
        let model = Model::new(config)?;
        initial.housing.validate_steps(config.step_seconds)?;
        let housing = initial.housing.sample(0.0)?;
        let value = model.evaluate(initial.joints, housing, initial.drive)?;
        let initial_energy = value.diagnostics.total_energy();
        Ok(Self {
            model,
            housing: initial.housing,
            current: Snapshot {
                step_index: 0,
                time_seconds: 0.0,
                joints: initial.joints,
                drive: initial.drive,
                housing,
                component_orientations: value.component_orientations,
                diagnostics: value.diagnostics,
                work: Work::default(),
                energy_balance_residual: 0.0,
            },
            initial_energy,
        })
    }

    pub fn config(&self) -> SimulationConfig {
        self.model.config()
    }

    pub fn snapshot(&self) -> Snapshot {
        self.current
    }

    /// Reset time, state and work integrals. Failure leaves the old run untouched.
    pub fn reset(&mut self, initial: InitialState) -> Result<Snapshot, SimulationError> {
        let replacement = Self::new(self.config(), initial)?;
        *self = replacement;
        Ok(self.current)
    }

    pub fn set_drive_mode(&mut self, mode: DriveMode) -> Result<Snapshot, SimulationError> {
        self.set_drive(DriveState {
            mode,
            ..self.current.drive
        })
    }

    /// Set a bearing-relative target in rad/s, without overwriting the rotor rate.
    pub fn set_target_spin(&mut self, target_spin: f64) -> Result<Snapshot, SimulationError> {
        self.set_drive(DriveState {
            target_spin,
            ..self.current.drive
        })
    }

    fn set_drive(&mut self, drive: DriveState) -> Result<Snapshot, SimulationError> {
        let value = self
            .model
            .evaluate(self.current.joints, self.current.housing, drive)?;
        self.current.drive = drive;
        self.current.diagnostics = value.diagnostics;
        Ok(self.current)
    }

    fn derivative(&self, time: f64, state: JointState) -> Result<Derivative, SimulationError> {
        if !state.is_finite() {
            return Err(SimulationError::numerical("non-finite integration stage"));
        }
        let housing = self.housing.sample(time)?;
        let evaluation = self.model.evaluate(state, housing, self.current.drive)?;
        Ok(Derivative::from_evaluation(state, evaluation))
    }

    /// Advance one RK4 step. No state, time, or diagnostics commit on failure.
    pub fn step(&mut self) -> Result<Snapshot, SimulationError> {
        let next = self
            .next_snapshot()
            .map_err(|error| error.at_step(self.current.step_index))?;
        self.current = next;
        Ok(next)
    }

    fn next_snapshot(&self) -> Result<Snapshot, SimulationError> {
        let dt = self.config().step_seconds;
        let t = self.current.time_seconds;
        let step_index = self
            .current
            .step_index
            .checked_add(1)
            .ok_or_else(|| SimulationError::numerical("step counter overflow"))?;
        let next_time = step_index as f64 * dt;
        if !next_time.is_finite() || next_time <= t {
            return Err(SimulationError::numerical(
                "simulation time overflow or lost precision",
            ));
        }
        let state = self.current.joints;
        let a = self.derivative(t, state)?;
        let b = self.derivative(t + 0.5 * dt, shifted(state, a, 0.5 * dt))?;
        let c = self.derivative(t + 0.5 * dt, shifted(state, b, 0.5 * dt))?;
        let d = self.derivative(next_time, shifted(state, c, dt))?;
        let integrate = |start: f64, a: f64, b: f64, c: f64, d: f64| {
            start + (dt / 6.0) * (a + 2.0 * b + 2.0 * c + d)
        };
        let joints = JointState {
            angles: std::array::from_fn(|i| {
                integrate(
                    state.angles[i],
                    a.angles[i],
                    b.angles[i],
                    c.angles[i],
                    d.angles[i],
                )
            }),
            rates: std::array::from_fn(|i| {
                integrate(
                    state.rates[i],
                    a.rates[i],
                    b.rates[i],
                    c.rates[i],
                    d.rates[i],
                )
            }),
        };
        if !joints.is_finite() {
            return Err(SimulationError::numerical("non-finite integrated state"));
        }
        let old_work = self.current.work;
        let work = Work {
            housing: integrate(
                old_work.housing,
                a.work.housing,
                b.work.housing,
                c.work.housing,
                d.work.housing,
            ),
            motor: integrate(
                old_work.motor,
                a.work.motor,
                b.work.motor,
                c.work.motor,
                d.work.motor,
            ),
            dissipated: integrate(
                old_work.dissipated,
                a.work.dissipated,
                b.work.dissipated,
                c.work.dissipated,
                d.work.dissipated,
            ),
        };
        let housing = self.housing.sample(next_time)?;
        let value = self.model.evaluate(joints, housing, self.current.drive)?;
        let residual = value.diagnostics.total_energy()
            - self.initial_energy
            - (work.housing + work.motor - work.dissipated);
        if ![work.housing, work.motor, work.dissipated, residual]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(SimulationError::numerical("work integral overflow"));
        }
        Ok(Snapshot {
            step_index,
            time_seconds: next_time,
            joints,
            drive: self.current.drive,
            housing,
            component_orientations: value.component_orientations,
            diagnostics: value.diagnostics,
            work,
            energy_balance_residual: residual,
        })
    }

    /// Advance a batch. Earlier successful steps remain committed if a later step
    /// fails; inspect `snapshot()` for the last accepted state. A zero-size batch
    /// returns the current snapshot in both slots.
    pub fn advance_steps(&mut self, count: u32) -> Result<AdvanceResult, SimulationError> {
        let mut previous = self.current;
        for _ in 0..count {
            previous = self.current;
            self.step()?;
        }
        Ok(AdvanceResult {
            previous,
            current: self.current,
            steps_completed: count,
        })
    }
}
