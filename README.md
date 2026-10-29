# Aircraft gyroscope simulator

An interactive 3D teaching lab for mechanical aircraft gyroscopes: one-gimbal turn indicator, two-gimbal attitude reference and directional gyro. Rust computes finite-inertia dynamics in WebAssembly; Three.js displays the exposed mechanism alongside simplified instrument faces. Dragging requests a housing orientation that a smooth Rust controller follows—it never teleports the gyro.

The native solver, browser lab, local Axum asset server, and Nix packaging are implemented. See the [implementation sequence](docs/02-simulation-design.md#10-implementation-sequence-and-exit-criteria).

## Build and run with Nix

From the repository root, with Nix's `nix-command` and `flakes` features enabled:

```sh
nix run
```

Open **http://127.0.0.1:3000** and stop with Ctrl+C. Nix builds the Rust server, WASM bindings, and production web bundle using the committed locks. The launcher supplies an immutable asset path, so it works from any directory and needs no runtime dependency downloads. Initial builds need network access to fetch pinned inputs and dependencies.

```sh
nix build                         # result/bin/gyro-sim and result/share/gyro-sim/web
./result/bin/gyro-sim --port 3107
nix run . -- --bind 127.0.0.1 --port 3107
nix flake check                   # native, parity, frontend, browser, packaging, formatting
nix develop                       # pinned Rust/WASM, wasm-bindgen, Node/npm, Chromium
```

Individual build outputs are `nix build .#server`, `nix build .#wasm`, and `nix build .#web`. The WASM output contains `web/` and `nodejs/` bindings; the web output is ready to serve. The default package's launcher also accepts `--assets` to serve an explicitly chosen directory. Use an absolute flake path to launch from elsewhere, e.g. `nix run /path/to/gyro-sim`.

The flake defines `x86_64-linux` and `aarch64-linux` outputs; validation is performed on x86_64 Linux. ARM64 requires validation on a matching builder. `flake.lock` pins Nixpkgs and rust-overlay; `rust-toolchain.toml` selects Rust 1.99.1 including the WASM target. The locked Nixpkgs provides wasm-bindgen 0.2.129 and Node 26.12.1. Builds fetch Cargo dependencies by their lockfile checksums and npm dependencies by a fixed hash, then compile and test in the Nix sandbox.

For development, entering the shell does not install project dependencies or start services:

```sh
nix develop
npm --prefix web ci
npm --prefix web run wasm
npm --prefix web run dev
```

The shell provides Chromium through `CHROMIUM_PATH` and the matching CLI through `WASM_BINDGEN`; rustup is not required. Rebuild WASM after Rust changes. To run production browser tests in this shell, run `npm --prefix web run build`, `cargo build --release --locked -p gyro-server`, then `npm --prefix web run test:browser`.

Use `nix fmt flake.nix nix/default.nix` to format the Nix definitions. To update build tools deliberately, run `nix flake update` and rerun `nix flake check`; the wasm-bindgen CLI must still match `Cargo.lock`. After changing `web/package-lock.json`, replace the `npmDeps` hash in `nix/default.nix` with `lib.fakeHash`, run `nix build .#web`, then set the reported `got: sha256-…` hash and rebuild. Add new source files to Git so Git-backed flakes include them. See the [Nix flake documentation](https://nix.dev/concepts/flakes) for feature setup and lockfile behavior.

## GitHub CI and Pages

`.github/workflows/ci.yml` runs `nix flake check` and builds `.#web` on branch pushes, pull requests targeting `main`, and manual runs. The checks include the full Chromium suite both at `/` and at `/gyro-sim/`, plus workflow validation with actionlint. The workflow uses pinned Action revisions and the committed Nix lockfile on an x86_64 Ubuntu runner.

After checks pass on `main`, the same workflow uploads the tested static bundle and deploys it through the `github-pages` environment. Pull requests and other branches only run CI. Only the deployment job has Pages write permission; no personal access token or external cache account is required.

To enable publishing:

1. In the repository's **Settings → Pages → Build and deployment**, set **Source** to **GitHub Actions**.
2. Commit and push the Nix and workflow files to `main` (or merge them through a pull request).
3. Watch **Actions → CI and Pages**. After the first successful deployment, the site is available at **https://jakubtrnka.github.io/gyro-sim/** unless a custom domain is configured. Later pushes to `main` update it automatically; **Run workflow** on `main` can redeploy it.

GitHub Pages serves HTML, JavaScript, CSS, WASM, and the bundled documentation. The simulation runs entirely in the browser, so Pages does not need the local Axum server. Vite uses relative asset URLs, and navigation/documentation links are relative too; the same `.#web` artifact works locally, under a repository path, or on a custom domain. See [GitHub's custom Pages workflow documentation](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages) for repository settings and deployment permissions.

The web build renders the mechanics, simulation design, project guide, and original proposal from Markdown into styled HTML. KaTeX renders equations with bundled fonts; Mermaid diagrams become inline SVG during the build using Chromium. The documentation includes navigation and a table of contents, works without JavaScript or external downloads, and preserves links between documents and heading anchors. Original Markdown and linked source examples remain available alongside the HTML.

## Run the browser lab without Nix

From the repository root, with rustup and Node/npm installed:

```sh
rustup toolchain install 1.99.1 --profile minimal --component rustfmt --component clippy --target wasm32-unknown-unknown
cargo +1.99.1 install wasm-bindgen-cli --version 0.2.129 --locked --root .tools/wasm-bindgen
npm --prefix web ci
npm --prefix web run wasm
npm --prefix web run build
cargo run --release --locked -p gyro-server -- --assets web/dist
```

Open **http://127.0.0.1:3000**. Tested with Rust 1.99.1, wasm-bindgen 0.2.129, Node 26.12.0, npm 12.2.1 and Chromium 156 on x86_64 Linux. Initial dependency/toolchain installation needs network access; the built application uses only local assets, with no CDN or server-side simulation. WebGL 2 is required. No hosted deployment is created.

Building documentation diagrams also requires Chromium. The Nix shell supplies it automatically. Outside Nix, use `/usr/bin/chromium`, set `CHROMIUM_PATH` to your Chromium executable, or run `npx playwright install chromium` from `web/` before building. Chromium is only a build dependency; the resulting documentation is static HTML, CSS, and SVG.

The server accepts `--bind`, `--port`, and `--assets`; the default bind is loopback. An absolute `--assets` path allows launching from another directory. After building, `./target/release/gyro-server --assets web/dist` starts it without Cargo. Stop with Ctrl+C.

The local server disables browser caching so restarting with a new build always serves the current HTML and styles. Nix normalizes file timestamps across builds, so filesystem modification dates cannot safely validate cached pages. GitHub Pages manages its own caching independently.

The build script checks the exact Rust version and matching wasm-bindgen CLI. It accepts a matching compiler directly on `PATH`, otherwise selects the pinned rustup toolchain or an installed `stable` of exactly the same version. `WASM_BINDGEN` may select an existing CLI and `GYRO_RUST_TOOLCHAIN` explicitly selects an equivalent named rustup toolchain. `CARGO_TARGET_DIR` is respected when locating the WASM artifact. If a host compiler-cache wrapper fails, prefix Cargo/WASM/parity commands with `RUSTC_WRAPPER=` without changing the global Cargo configuration. After dependencies are cached, `CARGO_NET_OFFLINE=true npm --prefix web run wasm` rebuilds offline.

For frontend iteration, build WASM first and run `npm --prefix web run dev`. Vite serves the development UI; rendered documentation and acceptance tests use the Axum-served production bundle. Rebuild WASM after Rust changes, and rebuild the web bundle after frontend or documentation changes.

## Explore an experiment

- Choose a preset, press Start, and drag the housing or its rotation handles. While paused, only the target ghost moves; Step advances exactly one physics step. Orbit camera and wheel zoom never affect physics.
- The enlarged rotation rings have matching larger mouse targets; hovering identifies Roll, Pitch or Yaw. The FWD arrow and teal TOP edge belong to the actual housing. A small aircraft inset shares the main camera's viewing direction, showing the physical housing rather than the ghost or gyro. Its light top, ochre underside and separate world-up marker help distinguish inversion.
- “Level housing” smoothly returns to this run's **starting housing orientation** without resetting time, spin, gimbal motion or command history. While paused it queues the return for Start/Step. For an imported run that starts tilted, it returns to that tilted pose—not necessarily world level. “↺ View” only restores the camera.
- Use the keyboard roll/pitch/yaw fields as an alternative to dragging. World-fixed and housing-fixed cameras, transparent frames, momentum/torque arrows and bounded traces help explain the mechanism.
- Roll/pitch/yaw experiment buttons explicitly reset and run a repeatable input. Reset returns to that saved experiment's initial conditions and pauses.
- Live target RPM and powered/coast controls are queued for the next step. They do not change measured spin instantaneously. Playback speed changes simulated time per wall time, not rotor RPM or numerical resolution.
- Advanced settings change initial RPM, physical parameters and resolution through “Apply and reset”. The simple gimbal-inertia control applies the same value to each principal axis of both rings. These editable settings are exploratory, not a validated operating envelope.
- Export/import a versioned JSON replay, including commands queued while paused. Reconstruction is cancellable and leaves the old run intact on failure. Imports are limited to 2 MB and 50,000 commands; all step/sequence indices must be JavaScript-safe integers. JSON canonically represents negative zero as zero.

The rotor marker is explicitly slowed to **1/200** for visibility; it does not change spindle direction, instruments or physics. Near-alignment and poor numerical conditioning are separate warnings. An undefined attitude bank is labelled instead of drawing a false horizon. Hidden tabs pause; overload pauses with an explanation without skipping physical steps or silently increasing `dt`.

The mechanisms are idealized: no erection system, gimbal stops, Earth/transport rate, acceleration-sensitive pendulous mechanism, or slip/skid ball. They illustrate gyro mechanics, not the complete behavior of a particular aircraft instrument or a certified training device.

## Try the attitude and directional gyros

```sh
cargo run --release --locked --offline -p gyro-core --example gyro_experiment -- --preset attitude --motion roll
cargo run --release --locked --offline -p gyro-core --example gyro_experiment -- --preset directional --motion yaw
cargo run --release --locked --offline -p gyro-core --example gyro_experiment -- --preset attitude --motion manual
```

The runner prints eight simulated seconds as CSV, then reconstructs the run from its command log and checks the complete final snapshot. `roll` and `pitch` are smooth 20° excursions with a hold and return; `yaw` ramps to 3°/s, holds, then stops turning. `manual` requests two mixed-axis housing targets at 0 and 4 s and exercises the same Rust follower intended for browser dragging. `--preset turn`, `--rpm -12000`, and `--coast` are also supported. Blank CSV fields mean an instrument reading is inapplicable or its bank decomposition is undefined.

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

`gyro-core` has no external dependencies with default features; its optional `serde` feature supplies serialization for the wrapper. Workspace dependency resolution requires an initial `cargo fetch --locked` before offline commands on a clean machine. An already-installed `stable` toolchain of exactly version 1.99.1 can also run these commands as `cargo +stable ...`.

## Verify the implementation

```sh
cargo test --workspace --release --locked --offline -- --nocapture
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked --offline -- -D warnings
npm --prefix web run check
npm --prefix web run check:format
npm --prefix web test
npm --prefix web run test:parity
cargo build --release --locked -p gyro-server
npm --prefix web run test:browser
```

Frontend checks require the generated WASM bindings and installed npm dependencies; browser tests additionally require the production web bundle and release server from the build instructions above. Playwright uses `/usr/bin/chromium` when present, otherwise its installed browser; override with `CHROMIUM_PATH`. To install Playwright's browser when needed, run `npm exec --prefix web -- playwright install chromium`. The suite starts/stops its own server on port 3000. If that port is already in use, leave the existing server alone and select another, e.g. `GYRO_TEST_PORT=3107 npm --prefix web run test:browser`.

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

Browser-stage evidence is separate: [native/WASM parity](web/tests/parity.mjs) compares nine 10-second trajectories (all presets at +12,000, −12,000 and zero initial RPM) at 0.5-second intervals, including mixed-axis housing targets, coast/drive switching and pending commands. Angle tolerance is `1e-8` rad; rate tolerance is `1e-7` rad/s plus `1e-8` relative; energy/momentum/work use `1e-10` absolute plus `1e-8` relative. Quaternion comparison is sign-independent chord distance below `1e-8`. It also checks exact same-WASM serialized replay, owned snapshots, invalid inputs and finite aligned-gimbal states. The [native fixture generator](crates/gyro-wasm/examples/parity_reference.rs) runs with the same pinned compiler as WASM.

Four [scheduler tests](web/tests/scheduler.test.ts), four [aircraft-projection tests](web/tests/orientation.test.ts), and sixteen [Chromium acceptance tests](web/tests/browser/lab.spec.ts) exercise lifecycle, larger rotation handles, dragging/camera isolation, aircraft-reference alignment and inversion, smooth leveling (including imported initial poses), keyboard controls, 30/60/120 Hz schedule equality, failed initialization, local-only asset loading, replay reconstruction/cancellation, undefined bank, responsive layout and performance. Desktop/mobile screenshots were inspected. Interactive hardware-accelerated Chromium, Firefox and Safari have not been manually validated; automated frame-schedule equality is not a claim that every display achieves 120 Hz.

Initial browser-stage reference measurement (before the aircraft-reference UI update), on an AMD Ryzen 9 3950X, x86_64 Linux, headless Chromium 156 with **SwiftShader software rendering**, default resolution, 1440×1100 viewport:

| Preset | Physics-only throughput | Achieved playback including rendering | Display FPS |
|---|---|---|---|
| Attitude | 5.08× real time | 0.94× | 19.9 |
| Directional | 4.89× real time | 0.98× | 23.3 |
| Turn indicator | 12.10× real time | 0.97× | 13.9 |

These are short local observations, not performance guarantees. Physics-only batches cover 1 simulated second for two-gimbal presets and 2 seconds for the turn indicator; playback includes roughly 2.4 seconds of wall time and browser automation overhead. No overload pause occurred. The **60 FPS display target is not met in this software-rendered environment** despite approximately real-time simulation. Hardware-accelerated testing and render/main-thread profiling remain necessary before a 60 FPS claim; physics resolution was unchanged.

## Use the core

Use matching `SimulationConfig::for_preset(preset)` and `InitialState::for_preset(preset)` values. Both `Default` implementations select the turn indicator. `Simulation::new(config, initial)` creates a validated run. `step()` advances exactly one coupled RK4 step; `advance_steps(n)` returns the last two accepted snapshots. A run advances only when asked, so pausing means not stepping. `reset(initial)` clears commands, time, and work integrals.

`queue(Command::...)` assigns a live event to the next unstarted step. `enqueue(CommandRecord { step_index, sequence, command })` accepts explicitly scheduled events; sequence numbers order events at the same step. Rejecting a past/duplicate/invalid command leaves the run unchanged. Commands and the physics step commit together on success; earlier successful steps of a failed batch remain committed. A zero-size batch applies nothing.

API change from stage 1: `set_target_spin(rad_per_second)` and `set_drive_mode(...)` now **queue** commands and return their `CommandRecord`; they no longer immediately update diagnostics. Targets, drive modes, and physical state change on the next step. `SetHeadingOffset` changes only the DG display, never its mechanism.

`SetHousingTarget([x,y,z,w])` changes the follower target without teleporting the housing or altering its rate. Releasing a drag should send no command: the last target remains. `StopHousing` targets the current pose and smoothly decelerates. `StartManeuver` starts a script relative to the current pose and requires housing at rest when the command executes; otherwise the whole step fails. Reset to a new experiment or let manual motion settle before scheduling a script. To recover from an already queued, rejected script, reset the run. Configuration is immutable during a run.

`replay()` returns a versioned, owned Rust `Replay` containing configuration, initial state, all accepted commands (including pending/future commands), and the completed-step count. `Simulation::from_replay(&replay)` reconstructs that state and retains pending commands. The browser exports/imports this schema as JSON. WASM `GyroSimulation.prepare_replay(replay)` validates and queues the log at step zero; the browser advances to `end_step` in cancellable batches rather than blocking on a synchronous reconstruction.

`Model::evaluate(...)` exposes instantaneous dynamics for arbitrary housing orientation, angular velocity, and angular acceleration. Direct callers must supply mutually consistent trajectories. `Simulation` supplies fixed housing, analytic yaw/pitch/roll scripts, and the integrated manual follower. Profile segment durations must align with the physics step; manual housing motion cannot be sampled analytically through `HousingMotion::sample(t)` for `t > 0`.

The API uses SI units. Joint arrays now have three slots: `[theta, psi, 0]` for turn indicators and `[alpha, beta, psi]` for attitude/DG. Unused turn-indicator slots must be zero; `preset.spin_index()` locates bearing-relative spin. `JointState::turn_indicator([theta, psi], [theta_rate, spin_rate])` is a convenience constructor. Component poses follow chain order, with identity in the unused slot. Quaternions are `[x, y, z, w]`, use Hamilton multiplication, and map local coordinates to world coordinates.

Snapshots include physical and target housing poses separately, instrument readings, generalized momenta, work/energy balance, actual-rate overshoot, and distinct axis-alignment/poor-conditioning flags. Alignment does not automatically imply a singular mass matrix. Attitude uses the geometric spindle rather than the sign of angular momentum, so reversing spin does not invert the instrument by definition; bank is `None` where its decomposition is undefined.

## Read the design

- [Gyroscope mechanics](docs/01-gyroscope-mechanics.md): learner-oriented explanation and full equations, from one gimbal to two.
- [Simulation design](docs/02-simulation-design.md): architecture, interfaces, browser interaction, build strategy, and acceptance criteria.
- [Original design paper](gyroscope-simulation-design.md): preserved background proposal.

