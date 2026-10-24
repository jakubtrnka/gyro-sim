use crate::housing::Controller;
use crate::math;
use crate::{
    AdvanceResult, Command, CommandRecord, DriveMode, DriveState, InitialState, JointState, Model,
    Replay, SimulationConfig, SimulationError, Snapshot, Work,
};
use std::collections::BTreeMap;

#[derive(Clone, Copy)]
struct State {
    joints: JointState,
    orientation: [f64; 4],
    omega: [f64; 3],
}

impl State {
    fn shifted(self, d: Self, dt: f64) -> Self {
        Self {
            joints: JointState {
                angles: std::array::from_fn(|i| self.joints.angles[i] + dt * d.joints.angles[i]),
                rates: std::array::from_fn(|i| self.joints.rates[i] + dt * d.joints.rates[i]),
            },
            orientation: std::array::from_fn(|i| self.orientation[i] + dt * d.orientation[i]),
            omega: std::array::from_fn(|i| self.omega[i] + dt * d.omega[i]),
        }
    }
}

#[derive(Clone, Copy)]
struct Derivative {
    state: State,
    work: Work,
}

/// Fixed-step RK4 simulation; no clock, browser, or ambient random input.
/// Controls queue at step boundaries. Not stepping is equivalent to pausing.
#[derive(Clone, Debug)]
pub struct Simulation {
    model: Model,
    controller: Controller,
    initial: InitialState,
    current: Snapshot,
    initial_energy: f64,
    commands: BTreeMap<(u64, u32), Command>,
}

impl Simulation {
    pub fn new(config: SimulationConfig, initial: InitialState) -> Result<Self, SimulationError> {
        let model = Model::new(config)?;
        initial.housing.validate_steps(config.step_seconds)?;
        if !initial.heading_offset.is_finite() {
            return Err(SimulationError::InvalidInput(
                "heading offset must be finite",
            ));
        }
        let controller = Controller::initial(initial.housing)?;
        let start = initial.housing.sample(0.0)?;
        let housing = controller.sample(
            0.0,
            start.orientation,
            start.angular_velocity,
            config.housing_controller,
        )?;
        let value = model.evaluate(initial.joints, housing, initial.drive)?;
        let initial_energy = value.diagnostics.total_energy();
        let heading_offset = initial.heading_offset.rem_euclid(std::f64::consts::TAU);
        let current = Snapshot {
            preset: config.preset,
            step_index: 0,
            time_seconds: 0.0,
            joints: initial.joints,
            drive: initial.drive,
            housing,
            component_orientations: value.component_orientations,
            housing_target: controller.target(),
            housing_rate_exceeded: math::norm(housing.angular_velocity)
                > config.housing_controller.max_rate * (1.0 + 1e-10),
            heading_offset,
            instrument: model.instrument(initial.joints, housing, value, heading_offset)?,
            diagnostics: value.diagnostics,
            work: Work::default(),
            energy_balance_residual: 0.0,
        };
        Ok(Self {
            model,
            controller,
            initial,
            current,
            initial_energy,
            commands: BTreeMap::new(),
        })
    }

    pub fn config(&self) -> SimulationConfig {
        self.model.config()
    }
    pub fn snapshot(&self) -> Snapshot {
        self.current
    }

    /// Clears commands, clock and work. Failure leaves the entire old run intact.
    pub fn reset(&mut self, initial: InitialState) -> Result<Snapshot, SimulationError> {
        *self = Self::new(self.config(), initial)?;
        Ok(self.current)
    }

    /// Queue a live event at the next unstarted step, after existing events there.
    pub fn queue(&mut self, command: Command) -> Result<CommandRecord, SimulationError> {
        let step_index = self.current.step_index;
        let sequence = match self
            .commands
            .range((step_index, 0)..=(step_index, u32::MAX))
            .next_back()
        {
            Some(((.., sequence), _)) => sequence
                .checked_add(1)
                .ok_or(SimulationError::InvalidInput("command sequence overflow"))?,
            None => 0,
        };
        self.enqueue(CommandRecord {
            step_index,
            sequence,
            command,
        })
    }

    /// Validate and retain a timestamped event; accepts out-of-order future input.
    /// Rejection changes neither the physical snapshot nor the replay log.
    pub fn enqueue(&mut self, mut record: CommandRecord) -> Result<CommandRecord, SimulationError> {
        let key = (record.step_index, record.sequence);
        if record.step_index < self.current.step_index {
            return Err(SimulationError::InvalidInput(
                "command targets a completed step",
            ));
        }
        if self.commands.contains_key(&key) {
            return Err(SimulationError::InvalidInput(
                "duplicate command sequence at this step",
            ));
        }
        record.command = record.command.validated(self.config().step_seconds)?;
        self.commands.insert(key, record.command);
        Ok(record)
    }

    pub fn set_drive_mode(&mut self, mode: DriveMode) -> Result<CommandRecord, SimulationError> {
        self.queue(Command::SetDriveMode(mode))
    }

    /// Bearing-relative rad/s; does not overwrite the measured rotor rate.
    pub fn set_target_spin(&mut self, rate: f64) -> Result<CommandRecord, SimulationError> {
        self.queue(Command::SetTargetSpin(rate))
    }

    pub fn replay(&self) -> Replay {
        Replay {
            version: Replay::VERSION,
            config: self.config(),
            initial: self.initial,
            commands: self
                .commands
                .iter()
                .map(|(&(step_index, sequence), &command)| CommandRecord {
                    step_index,
                    sequence,
                    command,
                })
                .collect(),
            end_step: self.current.step_index,
        }
    }

    /// Reconstructs through `end_step`; pending/future commands stay queued.
    pub fn from_replay(replay: &Replay) -> Result<Self, SimulationError> {
        if replay.version != Replay::VERSION {
            return Err(SimulationError::InvalidInput("unsupported replay version"));
        }
        let mut sim = Self::new(replay.config, replay.initial)?;
        for record in &replay.commands {
            sim.enqueue(*record)?;
        }
        while sim.current.step_index < replay.end_step {
            sim.advance_steps(
                (replay.end_step - sim.current.step_index).min(u64::from(u32::MAX)) as u32,
            )?;
        }
        Ok(sim)
    }

    fn derivative(
        &self,
        controller: Controller,
        drive: DriveState,
        time: f64,
        state: State,
    ) -> Result<Derivative, SimulationError> {
        if !state.joints.is_finite() {
            return Err(SimulationError::numerical("non-finite integration stage"));
        }
        let housing = controller.sample(
            time,
            state.orientation,
            state.omega,
            self.config().housing_controller,
        )?;
        let value = self.model.evaluate(state.joints, housing, drive)?;
        let (orientation, omega) = if matches!(controller, Controller::Manual { .. }) {
            let [x, y, z] = housing.angular_velocity;
            (
                math::quat_mul([x, y, z, 0.0], housing.orientation).map(|v| 0.5 * v),
                housing.angular_acceleration,
            )
        } else {
            ([0.0; 4], [0.0; 3])
        };
        Ok(Derivative {
            state: State {
                joints: JointState {
                    angles: state.joints.rates,
                    rates: value.accelerations,
                },
                orientation,
                omega,
            },
            work: Work {
                housing: value.diagnostics.housing_power,
                motor: value.diagnostics.motor_power,
                dissipated: value.diagnostics.dissipated_power,
            },
        })
    }

    /// All queued controls and the step commit together, only on success.
    pub fn step(&mut self) -> Result<Snapshot, SimulationError> {
        let (next, controller) = self
            .next_snapshot()
            .map_err(|error| error.at_step(self.current.step_index))?;
        self.current = next;
        self.controller = controller;
        Ok(next)
    }

    fn next_snapshot(&self) -> Result<(Snapshot, Controller), SimulationError> {
        let config = self.config();
        let dt = config.step_seconds;
        let current = self.current;
        let t = current.time_seconds;
        let step_index = current
            .step_index
            .checked_add(1)
            .ok_or_else(|| SimulationError::numerical("step counter overflow"))?;
        let next_time = step_index as f64 * dt;
        if !next_time.is_finite() || next_time <= t {
            return Err(SimulationError::numerical(
                "simulation time overflow or lost precision",
            ));
        }
        let mut controller = self.controller;
        let mut drive = current.drive;
        let mut heading_offset = current.heading_offset;
        for command in self
            .commands
            .range((current.step_index, 0)..=(current.step_index, u32::MAX))
            .map(|(_, v)| v)
        {
            match *command {
                Command::SetDriveMode(mode) => drive.mode = mode,
                Command::SetTargetSpin(rate) => drive.target_spin = rate,
                Command::SetHeadingOffset(offset) => heading_offset = offset,
                Command::SetHousingTarget(target) => {
                    controller = Controller::Manual {
                        target: math::canonical(target)?,
                    }
                }
                Command::StopHousing => {
                    controller = Controller::Manual {
                        target: math::canonical(current.housing.orientation)?,
                    }
                }
                Command::StartManeuver(maneuver) => {
                    if math::norm(current.housing.angular_velocity) > 1e-10 {
                        return Err(SimulationError::InvalidInput(
                            "scripted maneuver requires housing at rest; reset or let manual motion settle first",
                        ));
                    }
                    controller = Controller::Prescribed {
                        motion: maneuver.motion(),
                        epoch: t,
                        base: current.housing.orientation,
                    };
                }
            }
        }
        let state = State {
            joints: current.joints,
            orientation: current.housing.orientation,
            omega: current.housing.angular_velocity,
        };
        let a = self.derivative(controller, drive, t, state)?;
        let b = self.derivative(
            controller,
            drive,
            t + 0.5 * dt,
            state.shifted(a.state, 0.5 * dt),
        )?;
        let c = self.derivative(
            controller,
            drive,
            t + 0.5 * dt,
            state.shifted(b.state, 0.5 * dt),
        )?;
        let d = self.derivative(controller, drive, next_time, state.shifted(c.state, dt))?;
        let integrate = |start: f64, a: f64, b: f64, c: f64, d: f64| {
            start + (dt / 6.0) * (a + 2.0 * b + 2.0 * c + d)
        };
        let joints = JointState {
            angles: std::array::from_fn(|i| {
                integrate(
                    state.joints.angles[i],
                    a.state.joints.angles[i],
                    b.state.joints.angles[i],
                    c.state.joints.angles[i],
                    d.state.joints.angles[i],
                )
            }),
            rates: std::array::from_fn(|i| {
                integrate(
                    state.joints.rates[i],
                    a.state.joints.rates[i],
                    b.state.joints.rates[i],
                    c.state.joints.rates[i],
                    d.state.joints.rates[i],
                )
            }),
        };
        if !joints.is_finite() {
            return Err(SimulationError::numerical("non-finite integrated state"));
        }
        let orientation = std::array::from_fn(|i| {
            integrate(
                state.orientation[i],
                a.state.orientation[i],
                b.state.orientation[i],
                c.state.orientation[i],
                d.state.orientation[i],
            )
        });
        let omega = std::array::from_fn(|i| {
            integrate(
                state.omega[i],
                a.state.omega[i],
                b.state.omega[i],
                c.state.omega[i],
                d.state.omega[i],
            )
        });
        let work = Work {
            housing: integrate(
                current.work.housing,
                a.work.housing,
                b.work.housing,
                c.work.housing,
                d.work.housing,
            ),
            motor: integrate(
                current.work.motor,
                a.work.motor,
                b.work.motor,
                c.work.motor,
                d.work.motor,
            ),
            dissipated: integrate(
                current.work.dissipated,
                a.work.dissipated,
                b.work.dissipated,
                c.work.dissipated,
                d.work.dissipated,
            ),
        };
        let housing =
            controller.sample(next_time, orientation, omega, config.housing_controller)?;
        let value = self.model.evaluate(joints, housing, drive)?;
        let residual = value.diagnostics.total_energy()
            - self.initial_energy
            - (work.housing + work.motor - work.dissipated);
        if ![work.housing, work.motor, work.dissipated, residual]
            .iter()
            .all(|v| v.is_finite())
        {
            return Err(SimulationError::numerical("work integral overflow"));
        }
        Ok((
            Snapshot {
                preset: config.preset,
                step_index,
                time_seconds: next_time,
                joints,
                drive,
                housing,
                component_orientations: value.component_orientations,
                housing_target: controller.target(),
                housing_rate_exceeded: math::norm(housing.angular_velocity)
                    > config.housing_controller.max_rate * (1.0 + 1e-10),
                heading_offset,
                instrument: self
                    .model
                    .instrument(joints, housing, value, heading_offset)?,
                diagnostics: value.diagnostics,
                work,
                energy_balance_residual: residual,
            },
            controller,
        ))
    }

    /// Earlier successful steps remain committed if a later step fails. A zero
    /// batch neither applies controls nor advances time and returns current twice.
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
