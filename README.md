# Aircraft gyroscope simulator

An educational simulation for understanding mechanical aircraft gyroscopes. The mathematical and implementation documents are written, and the first native Rust stage is implemented: a one-gimbal turn indicator with finite gimbal inertia, a spinning rotor, a spring/damper, and a torque-limited motor.

The native core is ready to run and test. Two-gimbal presets, interactive housing control, browser/WASM presentation, the Rust web server, and Nix packaging are subsequent stages described in the [implementation sequence](docs/02-simulation-design.md#10-implementation-sequence-and-exit-criteria).

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

The physics tests check 1,000 seeded acceleration cases against independently derived body-frame Lagrangian equations, small-rate sensitivity, reversed and zero spin, motor saturation, continuous reversal, motor/coast time constants, housing work, fourth-order convergence, and long-run conservation. Lifecycle tests cover trajectory consistency, input validation, quaternion conventions, deterministic batching, reset, and atomic failure recovery.

Reference results from Rust 1.99.1 on x86_64 Linux:

| Check | Result |
|---|---|
| 1,000 acceleration comparisons | Maximum scaled discrepancy approximately $4.75\times10^{-11}$ |
| RK4 step-halving comparison | Difference ratio approximately 15.999 |
| 60 s fixed-housing conservative oscillator | Energy drift approximately $6.30\times10^{-12}$ J |

Those results apply to the checked fixtures. They do not establish a general operating envelope or validate future two-gimbal/WASM implementations. Fixture parameters and tolerances are retained in [the physics tests](crates/gyro-core/tests/physics.rs). The trial default step remains 1/16,384 s; individual tests use explicitly specified steps to check convergence and analytical limits.

## Use the core

`Simulation::new(config, initial)` creates a validated run. `step()` advances exactly one RK4 step; `advance_steps(n)` advances a batch and returns the last two accepted snapshots. A run advances only when asked, so a native caller pauses by not stepping. `reset(initial)` restores initial conditions and clears time/work integrals.

`set_target_spin(rad_per_second)` and `set_drive_mode(...)` take effect between steps and preserve angles and rates. They are immediate native controls; timestamped command queues and replay are scheduled for the next stage. Configuration is immutable during a run. A failed step leaves its state and diagnostics untouched; earlier successful steps of a failed batch remain committed.

`Model::evaluate(...)` exposes instantaneous dynamics for a supplied housing orientation, angular velocity, and angular acceleration. It supports arbitrary instantaneous housing kinematics; callers must supply mutually consistent trajectories. The built-in `Simulation` drivers currently provide fixed housing and a smooth yaw-rate ramp/hold/ramp-down. Profile durations must align with the chosen physics step.

The API uses SI units and arrays in documented order. Joint coordinates are `[theta, psi]`, with spin relative to the supporting gimbal. Quaternions are `[x, y, z, w]`, use Hamilton multiplication, and map local coordinates to world coordinates. Snapshots contain owned values, component poses, angular momentum, torques, power, and integrated work.

## Read the design

- [Gyroscope mechanics](docs/01-gyroscope-mechanics.md): learner-oriented explanation and full equations, from one gimbal to two.
- [Simulation design](docs/02-simulation-design.md): architecture, interfaces, browser interaction, build strategy, and acceptance criteria.
- [Original design paper](gyroscope-simulation-design.md): preserved background proposal.

