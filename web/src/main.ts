import "./style.css";
import init, {
  GyroSimulation,
  preset_definition,
} from "../generated/web/gyro_wasm";
import type {
  Command,
  Definition,
  DriveMode,
  Preset,
  Quat,
  Replay,
  Snapshot,
} from "../generated/web/gyro_wasm";
import wasmUrl from "../generated/web/gyro_wasm_bg.wasm?url";
import { Euler, Quaternion } from "three";
import { MechanismView } from "./scene";
import { FixedClock } from "./scheduler";
import {
  degrees,
  radians,
  rpm,
  spinIndex,
  interpolate,
  instrumentSvg,
} from "./display";

const element = <T extends HTMLElement = HTMLElement>(id: string) =>
  document.getElementById(id) as T;
const input = (id: string) => element<HTMLInputElement>(id);
const select = (id: string) => element<HTMLSelectElement>(id);
const button = (id: string) => element<HTMLButtonElement>(id);
const controls = element<HTMLFieldSetElement>("lab-controls");
function message(text: string, error = false) {
  element("status").textContent = text;
  element("status").classList.toggle("error", error);
}
function describe(error: unknown) {
  if (error && typeof error === "object" && "message" in error)
    return String(error.message);
  return String(error);
}
function number(id: string) {
  const field = input(id);
  if (
    !field.checkValidity() ||
    field.value.trim() === "" ||
    !Number.isFinite(field.valueAsNumber)
  )
    throw new Error(
      `Check ${field.labels?.[0]?.textContent?.trim() ?? id}: enter a finite number in the offered range.`,
    );
  return field.valueAsNumber;
}
const names = {
  Attitude: {
    title: "Attitude gyroscope",
    instrument: "Artificial horizon",
    code: "01 / VERTICAL REFERENCE",
    note: "A geometric vertical reference. No erection or drift correction; not flight-director guidance.",
  },
  Directional: {
    title: "Directional gyroscope",
    instrument: "Heading card",
    code: "02 / DIRECTIONAL REFERENCE",
    note: "Card geared to the outer gimbal. No Earth-rate or drift correction; not a magnetic compass.",
  },
  TurnIndicator: {
    title: "Turn indicator",
    instrument: "Rate of turn",
    code: "03 / RATE RESPONSE",
    note: "A spring-restrained rate gyro. Fixed positive-spin dial calibration; no slip/skid ball.",
  },
};

class Lab {
  definition = preset_definition("Attitude");
  sim = new GyroSimulation(this.definition.config, this.definition.initial);
  current = this.sim.snapshot();
  previous = this.current;
  clock = new FixedClock(this.definition.config.step_seconds);
  view: MechanismView;
  pendingTarget: Quat | null = null;
  trace: Array<{ t: number; outer: number; inner: number }> = [];
  importToken = 0;
  busy = false;
  graphicsFailed = false;
  physicsMs = 0;
  constructor() {
    this.view = new MechanismView(
      element("viewport"),
      (q) => this.safe(() => this.queue({ SetHousingTarget: q })),
      (text) => {
        this.graphicsFailed = true;
        this.pause();
        controls.disabled = true;
        message(text, true);
      },
    );
    this.adopt(this.definition, this.sim);
    this.bind();
    controls.disabled = false;
    message(
      "Ready. Start the rotor experiment, then drag the housing. Camera orbit never changes the physics.",
    );
    requestAnimationFrame(this.frame);
  }
  safe(action: () => void) {
    if (this.busy) return;
    try {
      action();
    } catch (error) {
      this.pause();
      message(describe(error), true);
    }
  }
  pause() {
    this.clock.pause();
    this.previous = this.current;
    this.updateTransport();
  }
  start() {
    if (this.graphicsFailed) return;
    this.clock.start();
    this.updateTransport();
  }
  updateTransport() {
    button("play").textContent = this.clock.running ? "Ⅱ Pause" : "▶ Start";
    button("step").disabled = this.clock.running || this.busy;
    element("motion-badge").textContent = this.clock.running
      ? `RUNNING · PLAYBACK ${this.clock.speed}×`
      : "PAUSED · INSPECT THE MECHANISM";
  }
  adopt(definition: Definition, sim: GyroSimulation) {
    if (this.sim !== sim) this.sim.free();
    this.definition = structuredClone(definition);
    this.sim = sim;
    this.current = sim.snapshot();
    this.previous = this.current;
    this.clock = new FixedClock(definition.config.step_seconds);
    this.clock.speed = Number(select("speed").value);
    this.pendingTarget = null;
    this.trace = [];
    this.view.configure(definition.config.preset);
    this.view.setTarget(
      this.current.housing_target ?? this.current.housing.orientation,
    );
    this.syncControls();
    this.updateTransport();
    this.draw(this.current);
  }
  syncControls() {
    const config = this.definition.config;
    const initial = this.definition.initial;
    const text = names[config.preset];
    element("mechanism-title").textContent = text.title;
    element("instrument-title").textContent = text.instrument;
    element("preset-code").textContent = text.code;
    element("instrument-note").textContent = text.note;
    element("inner-legend").hidden = config.preset === "TurnIndicator";
    element("heading-control").hidden = config.preset !== "Directional";
    document
      .querySelectorAll<HTMLButtonElement>("[data-preset]")
      .forEach((b) => {
        const active = b.dataset.preset === config.preset;
        b.classList.toggle("selected", active);
        b.setAttribute("aria-pressed", String(active));
      });
    input("initial-rpm").value = String(
      rpm(initial.joints.rates[spinIndex(config)]),
    );
    input("target-rpm").value = String(rpm(this.current.drive.target_spin));
    select("drive-mode").value = this.current.drive.mode;
    input("axial").value = String(config.rotor.axial);
    input("transverse").value = String(config.rotor.transverse);
    input("gimbal-inertia").value = String(config.gimbal_inertia[0]);
    input("spring").value = String(config.spring_stiffness);
    input("damping").value = String(config.gimbal_damping);
    const loss =
      config.motor.spin_damping === 0
        ? 0
        : config.rotor.axial / config.motor.spin_damping;
    this.selectSetting(
      "spin-loss",
      loss,
      `${loss.toPrecision(6)} s (imported)`,
    );
    const defaultStep = preset_definition(config.preset).config.step_seconds;
    this.selectSetting(
      "resolution",
      defaultStep / config.step_seconds,
      `${config.step_seconds.toPrecision(6)} s (imported)`,
    );
    for (const id of ["roll", "pitch", "yaw"]) input(id).value = "0";
    element("pending-target").textContent = "No pending target.";
  }
  selectSetting(id: string, value: number, label: string) {
    const control = select(id);
    control
      .querySelectorAll("[data-imported]")
      .forEach((option) => option.remove());
    if (
      ![...control.options].some((option) => option.value === String(value))
    ) {
      const option = new Option(label, String(value));
      option.dataset.imported = "true";
      control.add(option);
    }
    control.value = String(value);
  }
  queue(command: Command) {
    if (this.busy) return;
    this.sim.queue(command);
    if (typeof command === "object" && "SetHousingTarget" in command) {
      this.pendingTarget = command.SetHousingTarget;
      this.view.setTarget(this.pendingTarget);
      element("pending-target").textContent = this.clock.running
        ? "Target updated. The housing follows with bounded acceleration."
        : "Target queued. Ghost moved; physical housing stays still until Start or Step.";
    } else if (command === "StopHousing") {
      this.pendingTarget = this.current.housing.orientation;
      this.view.setTarget(this.pendingTarget);
      element("pending-target").textContent =
        "Current pose queued as target. Existing motion will decelerate smoothly.";
    }
  }
  advance(count: number) {
    const start = performance.now();
    try {
      const result = this.sim.advance_steps(count);
      this.previous = result.previous;
      this.current = result.current;
    } catch (error) {
      // A failed batch may contain earlier accepted steps: always recover its last snapshot.
      this.current = this.sim.snapshot();
      this.previous = this.current;
      throw error;
    } finally {
      this.physicsMs = performance.now() - start;
    }
    if (count > 0) {
      this.pendingTarget = null;
      element("pending-target").textContent = this.current.housing_target
        ? "Physical housing is following the requested target."
        : "Housing is fixed or follows an analytic experiment.";
    }
  }
  frame = (now: number) => {
    try {
      const result = this.clock.tick(now, (n) => this.advance(n));
      if (result.overloaded) {
        this.updateTransport();
        message(
          "Playback paused: the browser fell behind. No physics steps were skipped. Choose a slower playback speed and resume.",
          true,
        );
      }
      const shown = this.clock.running
        ? interpolate(
            this.previous,
            this.current,
            this.clock.fraction,
            this.definition.config,
          )
        : this.current;
      this.view.setTarget(
        this.pendingTarget ??
          this.current.housing_target ??
          this.current.housing.orientation,
      );
      this.view.render(shown);
      this.draw(shown);
    } catch (error) {
      this.pause();
      message(describe(error), true);
    }
    if (!this.graphicsFailed) requestAnimationFrame(this.frame);
  };
  draw(s: Snapshot) {
    element("time").textContent = s.time_seconds.toFixed(3);
    element("measured-rpm").textContent = rpm(
      s.joints.rates[spinIndex(s)],
    ).toLocaleString("en-US", { maximumFractionDigits: 0 });
    element("instrument-face").innerHTML = instrumentSvg(s);
    let readings: Array<[string, string]>;
    const warnings: string[] = [];
    if ("Attitude" in s.instrument) {
      const { pitch, bank } = s.instrument.Attitude;
      readings = [
        [`${degrees(pitch).toFixed(1)}°`, "PITCH"],
        [bank === null ? "—" : `${degrees(bank).toFixed(1)}°`, "BANK"],
      ];
      if (bank === null)
        warnings.push(
          "Bank is undefined near vertical pitch; horizon bank is not a valid attitude decomposition.",
        );
    } else if ("Directional" in s.instrument)
      readings = [
        [
          `${degrees(s.instrument.Directional.heading).toFixed(1).padStart(5, "0")}°`,
          "INDICATED HEADING",
        ],
      ];
    else {
      const rate = degrees(s.instrument.TurnIndicator.indicated_rate);
      readings = [
        [`${rate.toFixed(2)}°/s`, "INDICATED TURN"],
        [
          `${degrees(s.housing.angular_velocity[2]).toFixed(2)}°/s`,
          "WORLD YAW INPUT",
        ],
      ];
      if (Math.abs(rate) > 4.7)
        warnings.push(
          "Needle off scale. Numeric rate and physical gimbal state remain unclipped.",
        );
    }
    element("instrument-values").innerHTML = readings
      .map(
        ([value, label]) =>
          `<div><strong>${value}</strong><span>${label}</span></div>`,
      )
      .join("");
    if (s.diagnostics.near_axis_alignment)
      warnings.push(
        "Gimbal axes nearly aligned: one stabilizing direction is lost.",
      );
    if (s.diagnostics.poorly_conditioned)
      warnings.push(
        "Mass matrix is poorly conditioned; results require a numerical check.",
      );
    if (s.housing_rate_exceeded)
      warnings.push(
        "Housing rate exceeds the follower command limit; actual motion is not clamped.",
      );
    if (Math.abs(rpm(s.joints.rates[spinIndex(s)])) < 100)
      warnings.push(
        "Low spin: little gyroscopic rigidity. The mechanism still has finite inertia.",
      );
    element("warnings").textContent = warnings.join(" ");
    const d = s.diagnostics;
    const fields: Array<[string, string]> = [
      ["Completed step", String(this.current.step_index)],
      [
        "Physics dt",
        `${this.definition.config.step_seconds.toExponential(5)} s`,
      ],
      ["Kinetic energy", `${d.kinetic_energy.toExponential(5)} J`],
      ["Spring energy", `${d.spring_energy.toExponential(5)} J`],
      ["Axial momentum h", `${d.axial_momentum.toExponential(5)} N m s`],
      ["Absolute axial rate", `${d.absolute_axial_speed.toFixed(4)} rad/s`],
      [
        "Motor / housing power",
        `${d.motor_power.toExponential(2)} / ${d.housing_power.toExponential(2)} W`,
      ],
      ["Dissipated power", `${d.dissipated_power.toExponential(3)} W`],
      [
        "Work (motor / housing)",
        `${s.work.motor.toExponential(2)} / ${s.work.housing.toExponential(2)} J`,
      ],
      [
        "Energy balance residual",
        `${s.energy_balance_residual.toExponential(3)} J`,
      ],
      ["Matrix condition", d.mass_condition.toExponential(3)],
      ["Last WASM batch", `${this.physicsMs.toFixed(2)} ms`],
    ];
    element("diagnostics").innerHTML = fields
      .map(([a, b]) => `<dt>${a}</dt><dd>${b}</dd>`)
      .join("");
    const last = this.trace.at(-1);
    if (!last || s.time_seconds - last.t >= 1 / 30)
      this.trace.push({
        t: s.time_seconds,
        outer: degrees(s.joints.angles[0]),
        inner: s.preset === "TurnIndicator" ? 0 : degrees(s.joints.angles[1]),
      });
    while (
      this.trace.length > 400 ||
      (this.trace.length > 1 && this.trace[0].t < s.time_seconds - 12)
    )
      this.trace.shift();
    const range = Math.max(
      1,
      ...this.trace.flatMap((p) => [Math.abs(p.outer), Math.abs(p.inner)]),
    );
    const path = (field: "outer" | "inner") =>
      this.trace
        .map(
          (p, i) =>
            `${i ? "L" : "M"}${((600 * (p.t - Math.max(0, s.time_seconds - 12))) / 12).toFixed(2)},${(65 - (p[field] / range) * 52).toFixed(2)}`,
        )
        .join(" ");
    element("trace").innerHTML =
      `<path d="M0 13H600M0 65H600M0 117H600" stroke="#e2e8e1" stroke-dasharray="4 5"/><path d="${path("outer")}" fill="none" stroke="#b8784f" stroke-width="2"/>${s.preset === "TurnIndicator" ? "" : `<path d="${path("inner")}" fill="none" stroke="#278e8a" stroke-width="1.5"/>`}`;
    element("trace-range").textContent = `±${range.toFixed(1)}°`;
  }
  bind() {
    const click = (id: string, fn: () => void) =>
      button(id).addEventListener("click", () => this.safe(fn));
    click("play", () => {
      if (this.clock.running) this.pause();
      else this.start();
    });
    click("step", () => {
      this.advance(1);
      this.previous = this.current;
      this.draw(this.current);
    });
    click("reset", () => {
      this.adopt(
        this.definition,
        new GyroSimulation(this.definition.config, this.definition.initial),
      );
      message("Reset to saved initial conditions. Paused.");
    });
    document.querySelectorAll<HTMLButtonElement>("[data-preset]").forEach((b) =>
      b.addEventListener("click", () =>
        this.safe(() => {
          const definition = preset_definition(b.dataset.preset!);
          this.adopt(
            definition,
            new GyroSimulation(definition.config, definition.initial),
          );
          message(
            `${names[definition.config.preset].title} selected. Paused at initial conditions.`,
          );
        }),
      ),
    );
    select("speed").addEventListener("change", () => {
      this.clock.speed = Number(select("speed").value);
      this.updateTransport();
    });
    select("drive-mode").addEventListener("change", () =>
      this.safe(() => {
        this.queue({ SetDriveMode: select("drive-mode").value as DriveMode });
        message(
          "Drive mode queued for the next step; measured RPM is unchanged until then.",
        );
      }),
    );
    click("apply-rpm", () => {
      this.queue({ SetTargetSpin: (number("target-rpm") * 2 * Math.PI) / 60 });
      message("RPM target queued. Motor torque changes speed continuously.");
    });
    click("set-heading", () =>
      this.queue({
        SetHeadingOffset:
          radians(number("heading")) + this.current.joints.angles[0],
      }),
    );
    click("apply-pose", () => {
      // Aircraft yaw-pitch-roll = Rz(yaw) Ry(pitch) Rx(roll), not scene Euler angles.
      const q = new Quaternion().setFromEuler(
        new Euler(
          radians(number("roll")),
          radians(number("pitch")),
          radians(number("yaw")),
          "ZYX",
        ),
      );
      this.queue({ SetHousingTarget: q.toArray() as Quat });
      message(
        "Housing target queued. Start or Step to move the physical housing.",
      );
    });
    click("stop-housing", () => this.queue("StopHousing"));
    const mode = (move: boolean) => {
      this.view.setMode(move);
      button("move-mode").classList.toggle("selected", move);
      button("orbit-mode").classList.toggle("selected", !move);
      button("move-mode").setAttribute("aria-pressed", String(move));
      button("orbit-mode").setAttribute("aria-pressed", String(!move));
      element("drag-hint").textContent = move
        ? "Drag to rotate the housing target · scroll to zoom"
        : "Drag to orbit the camera · physics unchanged";
    };
    click("move-mode", () => mode(true));
    click("orbit-mode", () => mode(false));
    click("home-camera", () => this.view.home());
    select("camera-view").addEventListener("change", () =>
      this.view.setCameraView(select("camera-view").value === "housing"),
    );
    for (const id of ["transparent", "vectors", "marker"])
      input(id).addEventListener("change", () => {
        this.view.setOptions(
          input("transparent").checked,
          input("vectors").checked,
          input("marker").checked,
        );
        element("vector-scale").hidden = !input("vectors").checked;
        element("marker-label").textContent = input("marker").checked
          ? "Rotor marker ×1/200"
          : "Symmetric disc · no spin marker";
      });
    document.querySelectorAll<HTMLButtonElement>("[data-motion]").forEach((b) =>
      b.addEventListener("click", () =>
        this.safe(() => {
          const definition = structuredClone(this.definition);
          const initial = preset_definition(definition.config.preset).initial;
          initial.joints.rates[spinIndex(definition.config)] =
            this.definition.initial.joints.rates[spinIndex(definition.config)];
          initial.drive = { ...this.definition.initial.drive };
          const motion = b.dataset.motion!;
          initial.housing =
            motion === "Yaw" || motion === "SlowYaw"
              ? {
                  YawRate: {
                    rate_rad_s: radians(motion === "Yaw" ? 3 : 0.3),
                    ramp_seconds: 1,
                    hold_seconds: 4,
                    initial_yaw_rad: 0,
                  },
                }
              : {
                  Excursion: {
                    axis: motion as "Roll" | "Pitch",
                    angle_rad: radians(20),
                    move_seconds: 2,
                    hold_seconds: 2,
                  },
                };
          definition.initial = initial;
          this.adopt(
            definition,
            new GyroSimulation(definition.config, initial),
          );
          this.start();
          message(
            `${motion} experiment reset and running. This analytic input is independent of display frame rate.`,
          );
        }),
      ),
    );
    click("apply-settings", () => {
      const definition = structuredClone(this.definition);
      const c = definition.config;
      c.rotor = { axial: number("axial"), transverse: number("transverse") };
      const inertia = number("gimbal-inertia");
      c.gimbal_inertia = [inertia, inertia, inertia];
      c.inner_gimbal_inertia = [inertia, inertia, inertia];
      c.spring_stiffness = number("spring");
      c.gimbal_damping = number("damping");
      const loss = Number(select("spin-loss").value);
      c.motor.spin_damping = loss ? c.rotor.axial / loss : 0;
      c.step_seconds =
        preset_definition(c.preset).config.step_seconds /
        Number(select("resolution").value);
      const initial = preset_definition(c.preset).initial;
      const rate = (number("initial-rpm") * 2 * Math.PI) / 60;
      initial.joints.rates[spinIndex(c)] = rate;
      initial.drive = {
        mode: select("drive-mode").value as DriveMode,
        target_spin: rate,
      };
      definition.initial = initial;
      this.adopt(definition, new GyroSimulation(c, initial));
      message(
        "Settings applied and reset. Custom parameters require a convergence check; halve the physics step to compare.",
      );
    });
    click("export-replay", () => {
      const blob = new Blob([JSON.stringify(this.sim.replay(), null, 2)], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `gyro-${this.current.preset.toLowerCase()}-${this.current.step_index}.json`;
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
    input("import-replay").addEventListener("change", () => {
      const file = input("import-replay").files?.[0];
      if (file) void this.importReplay(file);
      input("import-replay").value = "";
    });
    button("cancel-import").addEventListener("click", () => {
      this.importToken++;
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden && this.clock.running) {
        this.pause();
        message(
          "Paused while the tab was hidden. Resume explicitly; no catch-up steps will be added.",
        );
      }
    });
  }
  async importReplay(file: File) {
    this.pause();
    if (file.size > 2_000_000) {
      message("Replay is larger than the 2 MB import limit.", true);
      return;
    }
    const token = ++this.importToken;
    this.busy = true;
    controls.disabled = true;
    button("cancel-import").hidden = false;
    let candidate: GyroSimulation | undefined;
    try {
      const replay = JSON.parse(await file.text()) as Replay;
      candidate = GyroSimulation.prepare_replay(replay);
      while (candidate.snapshot().step_index < replay.end_step) {
        if (token !== this.importToken)
          throw new Error(
            "Replay import cancelled. The previous run is retained, paused.",
          );
        const completed = candidate.snapshot().step_index;
        candidate.advance_steps(Math.min(2048, replay.end_step - completed));
        message(
          `Reconstructing replay… ${Math.floor((100 * candidate.snapshot().step_index) / replay.end_step)}%. The previous run is retained until this succeeds.`,
        );
        await new Promise((resolve) => setTimeout(resolve, 0));
      }
      if (token !== this.importToken)
        throw new Error("Replay import cancelled.");
      this.adopt({ config: replay.config, initial: replay.initial }, candidate);
      candidate = undefined;
      // A target queued while paused is part of the saved replay, but not the physical snapshot.
      for (const record of replay.commands
        .filter((r) => r.step_index === replay.end_step)
        .sort((a, b) => a.sequence - b.sequence)) {
        if (
          typeof record.command === "object" &&
          "SetHousingTarget" in record.command
        )
          this.pendingTarget = record.command.SetHousingTarget;
        if (record.command === "StopHousing")
          this.pendingTarget = this.current.housing.orientation;
      }
      if (this.pendingTarget)
        element("pending-target").textContent =
          "Target restored from replay. Ghost moved; physical housing stays still until Start or Step.";
      message(
        "Replay reconstructed. Paused at its saved step; pending commands remain queued.",
      );
    } catch (error) {
      message(`Import failed: ${describe(error)}`, true);
    } finally {
      candidate?.free();
      this.busy = false;
      controls.disabled = this.graphicsFailed;
      button("cancel-import").hidden = true;
      this.updateTransport();
    }
  }
}

try {
  await init({ module_or_path: wasmUrl });
  const lab = new Lab();
  // Read-only observation plus explicit test stepping, enabled only by a test URL.
  if (new URLSearchParams(location.search).has("test"))
    Object.assign(window, {
      __gyroTest: {
        snapshot: () => structuredClone(lab.current),
        replay: () => lab.sim.replay(),
        definition: () => structuredClone(lab.definition),
        queue: (command: Command) => lab.queue(command),
        advance: (n: number) => {
          lab.pause();
          lab.advance(n);
          return lab.current;
        },
        schedule: (fps: number, seconds: number) => {
          lab.pause();
          const replay = lab.sim.replay();
          const sim = GyroSimulation.prepare_replay(replay);
          const clock = new FixedClock(replay.config.step_seconds);
          clock.start();
          for (let i = 0; i <= fps * seconds; i++)
            clock.tick((i * 1000) / fps, (n) => sim.advance_steps(n));
          const snapshot = sim.snapshot();
          sim.free();
          return snapshot;
        },
        overload: () => {
          lab.start();
          lab.clock.tick(0, () => {});
          const result = lab.clock.tick(1000, () => {
            throw new Error("overload must not step");
          });
          lab.updateTransport();
          return result;
        },
        benchmark: () => {
          lab.pause();
          const before = lab.current.time_seconds;
          const start = performance.now();
          for (let i = 0; i < 16; i++) lab.advance(2048);
          return {
            wall_ms: performance.now() - start,
            simulated_seconds: lab.current.time_seconds - before,
          };
        },
      },
    });
} catch (error) {
  controls.disabled = true;
  message(
    `Could not start the lab: ${describe(error)}. This application needs WebAssembly and WebGL 2. Check the browser console or reload after rebuilding the assets.`,
    true,
  );
}
