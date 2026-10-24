# Aircraft gyroscope simulator

An educational simulation for understanding mechanical aircraft gyroscopes. The mathematical documents and both native Rust stages are implemented: the one-gimbal turn indicator, two-gimbal attitude and directional mechanisms, smooth housing control, instrument readings, and deterministic command replay. All presets use the same finite-inertia dynamics solver and torque-limited spin motor.

This is currently a native core and command-line experiment runner, not yet a browser application. The next stage is WASM/browser presentation; the Rust web server and Nix package follow it in the [implementation sequence](docs/02-simulation-design.md#10-implementation-sequence-and-exit-criteria).

## Try the attitude and directional gyros

```sh
cargo run --release --locked --offline -p gyro-core --example gyro_experiment -- --preset attitude --motion roll
cargo run --release --locked --offline -p gyro-core --example gyro_experiment -- --preset directional --motion yaw
cargo run --release --locked --offline -p gyro-core --example gyro_experiment -- --preset attitude --motion manual
```

The runner prints eight simulated seconds as CSV, then reconstructs the run from its command log and checks the complete final snapshot. `roll` and `pitch` are smooth 20° excursions with a hold and return; `yaw` ramps to 3°/s, holds, then stops turning. `manual` requests two mixed-axis housing targets at 0 and 4 s and exercises the same Rust follower intended for browser dragging. `--preset turn`, `--rpm -12000`, and `--coast` are also supported. Blank CSV fields mean an instrument reading is inapplicable or its bank decomposition is undefined.

The mechanisms are idealized: no erection system, gimbal stops, Earth/transport rate, or acceleration-sensitive pendulous mechanism is implemented. They illustrate gyro mechanics, not the complete behavior of a particular aircraft instrument.

## Run the native experiment

The workspace pins Rust 1.99.1 in `rust-toolchain.toml`. With Rust/rustup installed, run:

```sh
cargo run --release --locked --offline -p gyro-core --example turn_indicator
```

The example prints CSV for eight seconds of simulated motion: a one-second ramp to 3°/s housing yaw, four seconds at that rate, a one-second ramp down, and two seconds of settling. Columns show simulation time, housing yaw rate, gimbal angle, measured relative RPM, axial momentum, and energy-balance residual. No display or network connection is required by the executable.

Try a small-rate comparison, reversed spin, or coasting with a 30-second viscous-loss time constant:

```sh
cargo run --release --locked --offline -p gyro-core --example turn_indicator -- --rate-deg-s 0.3
cargo run --release --locked --offline -p gyro-core --example turn_indicator -- --rpm -12000
cargo run --release --locked --offline -p gyro-core --example turn_indicator -- --coast --loss-seconds 30
```

The positive-spin small-rate case settles close to 1° gimbal deflection. At 3°/s the finite-angle model settles below the nominal 10° small-signal calibration; that difference is physical, not a numerical correction. `--rpm` sets both initial and target relative speed. `--coast` disables the motor; without `--loss-seconds`, axial momentum is conserved and coasting alone does not cause spin-down.

The crate has no external dependencies. Cargo's offline commands work once the pinned compiler and its components are installed; rustup may need a network connection for that one-time toolchain installation. An already-installed `stable` toolchain of exactly version 1.99.1 can also run these commands as `cargo +stable ...`.

## Verify the implementation

```sh
cargo test --workspace --release --locked --offline -- --nocapture
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked --offline -- -D warnings
```

The tests retain the one-gimbal checks and add 2,000 two-gimbal fixed-housing references plus 1,500 moving-housing comparisons for all three presets against an independent energy formulation. That oracle differentiates body-frame energies using test-only second-order automatic differentiation; it does not reuse the production Jacobian or linear solver. Further tests cover nutation, 60 s conservation, 10 s manoeuvre convergence, manual motion/work balance, quaternion shortest paths, instrument signs, alignment/conditioning flags, command ordering, replay, reset, and atomic failure recovery.

Reference results from Rust 1.99.1 on x86_64 Linux:

| Check | Result |
|---|---|
| One-gimbal acceleration reference | Maximum scaled discrepancy $4.75\times10^{-11}$ |
| Two-gimbal fixed-housing references | Maximum scaled discrepancy $1.44\times10^{-11}$ |
| Independent moving-housing energy references, all presets | Maximum scaled discrepancy $1.91\times10^{-11}$ |
| Two-gimbal RK4 step-halving | Difference ratio approximately 16.019 |
| Small-motion nutation | 882.160152 rad/s measured; 882.160154 predicted |
| 60 s conservative two-gimbal runs | Maximum sampled energy drift below $5.7\times10^{-12}$ J; initial dynamic energy $7.59\times10^{-6}$ J |
| Nine 10 s yaw/pitch/roll comparisons at `dt` and `dt/2` | Maximum sampled pose difference below $6.7\times10^{-9}$ rad, gimbal-rate difference below $5.8\times10^{-6}$ rad/s |

Those results apply only to the retained fixtures, not a general parameter envelope, browser performance, or WASM parity. Conservation is sampled every 0.5 s, manoeuvre differences at 256 Hz; convergence fixtures start with nonzero gimbal displacement/rates. See [one-gimbal tests](crates/gyro-core/tests/physics.rs), [two-gimbal tests](crates/gyro-core/tests/two_gimbal.rs), [energy references](crates/gyro-core/tests/energy_reference.rs), and [control/replay tests](crates/gyro-core/tests/controls.rs). Native defaults are 1/16,384 s for the turn indicator and a conservative 1/32,768 s for attitude/DG. These are verified fixture resolutions, not proven minimum step requirements. Custom high-spin/low-inertia settings need their own convergence study.

## Use the core

Use matching `SimulationConfig::for_preset(preset)` and `InitialState::for_preset(preset)` values. Both `Default` implementations select the turn indicator. `Simulation::new(config, initial)` creates a validated run. `step()` advances exactly one coupled RK4 step; `advance_steps(n)` returns the last two accepted snapshots. A run advances only when asked, so pausing means not stepping. `reset(initial)` clears commands, time, and work integrals.

`queue(Command::...)` assigns a live event to the next unstarted step. `enqueue(CommandRecord { step_index, sequence, command })` accepts explicitly scheduled events; sequence numbers order events at the same step. Rejecting a past/duplicate/invalid command leaves the run unchanged. Commands and the physics step commit together on success; earlier successful steps of a failed batch remain committed. A zero-size batch applies nothing.

API change from stage 1: `set_target_spin(rad_per_second)` and `set_drive_mode(...)` now **queue** commands and return their `CommandRecord`; they no longer immediately update diagnostics. Targets, drive modes, and physical state change on the next step. `SetHeadingOffset` changes only the DG display, never its mechanism.

`SetHousingTarget([x,y,z,w])` changes the follower target without teleporting the housing or altering its rate. Releasing a drag should send no command: the last target remains. `StopHousing` targets the current pose and smoothly decelerates. `StartManeuver` starts a script relative to the current pose and requires housing at rest when the command executes; otherwise the whole step fails. Reset to a new experiment or let manual motion settle before scheduling a script. To recover from an already queued, rejected script, reset the run. Configuration is immutable during a run.

`replay()` returns a versioned, owned Rust `Replay` containing configuration, initial state, all accepted commands (including pending/future commands), and the completed-step count. `Simulation::from_replay(&replay)` reconstructs that state and retains pending commands. Replay is currently an in-memory native API: JSON/file import/export and cross-platform parity belong to the WASM stage.

`Model::evaluate(...)` exposes instantaneous dynamics for arbitrary housing orientation, angular velocity, and angular acceleration. Direct callers must supply mutually consistent trajectories. `Simulation` supplies fixed housing, analytic yaw/pitch/roll scripts, and the integrated manual follower. Profile segment durations must align with the physics step; manual housing motion cannot be sampled analytically through `HousingMotion::sample(t)` for `t > 0`.

The API uses SI units. Joint arrays now have three slots: `[theta, psi, 0]` for turn indicators and `[alpha, beta, psi]` for attitude/DG. Unused turn-indicator slots must be zero; `preset.spin_index()` locates bearing-relative spin. `JointState::turn_indicator([theta, psi], [theta_rate, spin_rate])` is a convenience constructor. Component poses follow chain order, with identity in the unused slot. Quaternions are `[x, y, z, w]`, use Hamilton multiplication, and map local coordinates to world coordinates.

Snapshots include physical and target housing poses separately, instrument readings, generalized momenta, work/energy balance, actual-rate overshoot, and distinct axis-alignment/poor-conditioning flags. Alignment does not automatically imply a singular mass matrix. Attitude uses the geometric spindle rather than the sign of angular momentum, so reversing spin does not invert the instrument by definition; bank is `None` where its decomposition is undefined.

## Read the design

- [Gyroscope mechanics](docs/01-gyroscope-mechanics.md): learner-oriented explanation and full equations, from one gimbal to two.
- [Simulation design](docs/02-simulation-design.md): architecture, interfaces, browser interaction, build strategy, and acceptance criteria.
- [Original design paper](gyroscope-simulation-design.md): preserved background proposal.

