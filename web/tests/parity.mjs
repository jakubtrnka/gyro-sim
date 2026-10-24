import assert from "node:assert/strict";
import { createRequire } from "node:module";
import { spawnSync } from "node:child_process";
import { rustToolchain } from "../scripts/toolchain.mjs";
const require = createRequire(import.meta.url);
const {
  GyroSimulation,
  preset_definition,
} = require("../generated/nodejs/gyro_wasm.js");

const native = spawnSync(
  "cargo",
  [
    `+${rustToolchain()}`,
    "run",
    "--release",
    "--locked",
    "--offline",
    "-p",
    "gyro-wasm",
    "--example",
    "parity_reference",
  ],
  {
    cwd: new URL("../../", import.meta.url),
    encoding: "utf8",
    maxBuffer: 8_000_000,
  },
);
assert.equal(native.status, 0, native.stderr);
const fixtures = JSON.parse(native.stdout);
function close(a, b, abs, name) {
  assert.ok(
    Math.abs(a - b) <= abs + 1e-8 * Math.max(Math.abs(a), Math.abs(b)),
    `${name}: ${a} vs ${b}`,
  );
}
function quat(a, b, name) {
  // Chord distance avoids acos amplification near identical unit quaternions.
  const dot = a.reduce((v, x, i) => v + x * b[i], 0);
  const sign = dot < 0 ? -1 : 1;
  const chord = Math.hypot(...a.map((x, i) => x - sign * b[i]));
  assert.ok(chord < 1e-8, `${name}: quaternion chord ${chord}`);
}
function compare(a, b, name) {
  assert.equal(a.step_index, b.step_index);
  assert.equal(a.preset, b.preset);
  assert.equal(a.drive.mode, b.drive.mode);
  close(a.drive.target_spin, b.drive.target_spin, 0, `${name}/target spin`);
  for (let i = 0; i < 3; i++) {
    const spin = a.preset === "TurnIndicator" ? 1 : 2;
    const difference =
      i === spin
        ? Math.atan2(
            Math.sin(a.joints.angles[i] - b.joints.angles[i]),
            Math.cos(a.joints.angles[i] - b.joints.angles[i]),
          )
        : a.joints.angles[i] - b.joints.angles[i];
    close(difference, 0, 1e-8, `${name}/angle${i}`);
    close(a.joints.rates[i], b.joints.rates[i], 1e-7, `${name}/rate${i}`);
    close(
      a.housing.angular_velocity[i],
      b.housing.angular_velocity[i],
      1e-7,
      `${name}/housing rate`,
    );
    quat(a.component_orientations[i], b.component_orientations[i], name);
  }
  quat(a.housing.orientation, b.housing.orientation, name);
  for (const key of ["kinetic_energy", "spring_energy", "axial_momentum"])
    close(a.diagnostics[key], b.diagnostics[key], 1e-10, `${name}/${key}`);
  for (const key of ["housing", "motor", "dissipated"])
    close(a.work[key], b.work[key], 1e-10, `${name}/work/${key}`);
  close(
    a.energy_balance_residual,
    b.energy_balance_residual,
    1e-10,
    `${name}/balance`,
  );
}
for (const fixture of fixtures) {
  const sim = GyroSimulation.prepare_replay(fixture.replay);
  for (const sample of fixture.samples) {
    while (sim.snapshot().step_index < sample.step_index)
      sim.advance_steps(
        Math.min(2048, sample.step_index - sim.snapshot().step_index),
      );
    compare(sim.snapshot(), sample, fixture.name);
  }
  // Returned snapshots are deep owned values, never live WASM memory views.
  const retained = sim.snapshot();
  const before = structuredClone(retained);
  sim.advance_steps(100);
  assert.deepEqual(retained, before);
  const replay = sim.replay();
  const rebuilt = GyroSimulation.prepare_replay(
    JSON.parse(JSON.stringify(replay)),
  );
  while (rebuilt.snapshot().step_index < replay.end_step)
    rebuilt.advance_steps(
      Math.min(32768, replay.end_step - rebuilt.snapshot().step_index),
    );
  // JSON canonically maps -0 to +0; require exact equality of every serialized
  // field without mistaking that representation change for a physics mismatch.
  assert.equal(
    JSON.stringify(rebuilt.snapshot()),
    JSON.stringify(sim.snapshot()),
  );
  rebuilt.free();
  sim.free();
  console.log(`PASS native/WASM 10 s: ${fixture.name}`);
}
const d = preset_definition("Attitude");
const sim = new GyroSimulation(d.config, d.initial);
const original = sim.snapshot();
for (const count of [-1, NaN, Infinity, 1.5, 32769])
  assert.throws(
    () => sim.advance_steps(count),
    (e) => e.kind === "invalid-input",
  );
for (const command of [
  { SetHousingTarget: [0, 0, 0, 0] },
  { SetTargetSpin: NaN },
  { SetTargetSpin: Infinity },
  { unknown: true },
])
  assert.throws(
    () => sim.queue(command),
    (e) => e.kind === "invalid-input",
  );
assert.throws(() =>
  sim.enqueue({
    step_index: 0,
    sequence: 0,
    command: { SetHeadingOffset: NaN },
  }),
);
assert.deepEqual(sim.snapshot(), original);
assert.throws(() =>
  GyroSimulation.prepare_replay({ ...sim.replay(), version: 0 }),
);
assert.throws(() =>
  GyroSimulation.prepare_replay({
    ...sim.replay(),
    end_step: Number.MAX_SAFE_INTEGER + 1,
  }),
);
assert.throws(
  () =>
    new GyroSimulation(
      { ...d.config, rotor: { axial: 0, transverse: 1 } },
      d.initial,
    ),
  (e) => e.kind === "invalid-configuration",
);
assert.throws(
  () => new GyroSimulation({ ...d.config, unexpected: 1 }, d.initial),
);
for (const preset of ["Attitude", "Directional"]) {
  const def = preset_definition(preset);
  def.initial.joints.angles[1] = Math.PI / 2;
  const aligned = new GyroSimulation(def.config, def.initial);
  aligned.advance_steps(100);
  assert.equal(aligned.snapshot().diagnostics.near_axis_alignment, true);
  assert.equal(aligned.snapshot().diagnostics.poorly_conditioned, false);
  aligned.free();
}
sim.free();
console.log(
  "PASS boundary validation, exact WASM replay, owned snapshots, and alignment",
);
