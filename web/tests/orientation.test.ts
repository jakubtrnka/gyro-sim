import { test } from "node:test";
import assert from "node:assert/strict";
import { Euler, Quaternion, Vector3 } from "three";
import { aircraftProjection, aircraftSvg } from "../src/orientation.ts";
import type { Quat } from "../generated/web/gyro_wasm";

const identity: Quat = [0, 0, 0, 1];
const near = (a: Vector3, b: Vector3) => assert.ok(a.distanceTo(b) < 1e-12);

test("aircraft nose follows housing yaw and pitch; inversion exposes its underside", () => {
  const camera = new Quaternion();
  const rotation = (axis: Vector3, angle: number) =>
    new Quaternion().setFromAxisAngle(axis, angle).toArray() as Quat;
  near(aircraftProjection(identity, camera).nose, new Vector3(1.55, 0, 0));
  near(
    aircraftProjection(rotation(new Vector3(0, 0, 1), Math.PI / 2), camera)
      .nose,
    new Vector3(0, 1.55, 0),
  );
  near(
    aircraftProjection(rotation(new Vector3(0, 1, 0), Math.PI / 2), camera)
      .nose,
    new Vector3(0, 0, -1.55),
  );
  const inverted = rotation(new Vector3(1, 0, 0), Math.PI);
  assert.equal(aircraftProjection(inverted, camera).topAlignment, -1);
  assert.match(aircraftSvg(inverted, camera), /Top faces down/);
  assert.match(aircraftSvg(identity, camera), /Top faces up/);
});

test("inset uses the main camera basis once and keeps world up independent of housing", () => {
  const housing = new Quaternion().setFromEuler(
    new Euler(0.7, -1.2, 2.1, "ZYX"),
  );
  const camera = new Quaternion().setFromEuler(new Euler(-0.2, 0.6, 0.9));
  const model = aircraftProjection(housing.toArray() as Quat, camera);
  near(
    model.nose,
    new Vector3(1.55, 0, 0).applyQuaternion(housing).applyQuaternion(camera),
  );
  near(model.worldUp, new Vector3(0, 0, -1).applyQuaternion(camera));
  near(model.worldUp, aircraftProjection(identity, camera).worldUp);
  // A housing-fixed camera cancels aircraft motion, while world up still moves.
  const fixed = camera.clone().multiply(housing.clone().invert());
  near(
    aircraftProjection(housing.toArray() as Quat, fixed).nose,
    aircraftProjection(identity, camera).nose,
  );
});

test("all projected faces stay finite and depth ordered through arbitrary rotations", () => {
  for (let i = 0; i < 100; i++) {
    const housing = new Quaternion()
      .setFromEuler(new Euler(i * 0.17, i * -0.31, i * 0.57))
      .toArray() as Quat;
    const camera = new Quaternion().setFromEuler(
      new Euler(i * -0.23, i * 0.11, 0.4),
    );
    const model = aircraftProjection(housing, camera);
    const equivalent = aircraftProjection(
      housing.map((x) => -x) as Quat,
      camera,
    );
    near(model.nose, equivalent.nose);
    for (let f = 0; f < model.faces.length; f++) {
      for (const vertex of model.faces[f].vertices)
        assert.ok(vertex.toArray().every(Number.isFinite));
      if (f > 0) assert.ok(model.faces[f].depth >= model.faces[f - 1].depth);
    }
    assert.doesNotMatch(aircraftSvg(housing, camera), /NaN|Infinity/);
  }
});

test("world-up marker remains meaningful when pointing towards or away from camera", () => {
  assert.match(aircraftSvg(identity, new Quaternion()), /data-world-up="away"/);
  assert.match(
    aircraftSvg(identity, new Quaternion(1, 0, 0, 0)),
    /data-world-up="towards"/,
  );
  assert.doesNotMatch(
    aircraftSvg(identity, new Quaternion(1, 0, 0, 0)),
    /NaN|Infinity/,
  );
});
