import { test } from "node:test";
import assert from "node:assert/strict";
import { FixedClock } from "../src/scheduler.ts";

test("30/60/120 Hz produce the same physical step count at the same time", () => {
  for (const dt of [1 / 16384, 1 / 32768])
    for (const fps of [30, 60, 120]) {
      const clock = new FixedClock(dt);
      let count = 0;
      clock.start();
      for (let i = 0; i <= fps * 10; i++)
        clock.tick((i * 1000) / fps, (n) => {
          assert.ok(n <= 2048);
          count += n;
        });
      assert.equal(count, 10 / dt);
      assert.ok(clock.accumulator < 1e-10);
    }
});
test("pause/resume discards wall time and interpolation debt, not simulation state", () => {
  const c = new FixedClock(1 / 32768);
  let count = 0;
  c.start();
  c.tick(0, (n) => (count += n));
  c.tick(10, (n) => (count += n));
  c.pause();
  const stopped = count;
  c.tick(1e8, (n) => (count += n));
  assert.equal(count, stopped);
  assert.equal(c.fraction, 0);
  c.start();
  c.tick(1e8 + 1000, (n) => (count += n));
  assert.equal(count, stopped);
  c.tick(1e8 + 1010, (n) => (count += n));
  assert.equal(count, stopped + 327);
});
test("overload pauses before skipping or adding any physical steps", () => {
  const c = new FixedClock(1 / 32768);
  c.start();
  c.tick(0, () => {});
  const result = c.tick(1000, () => assert.fail("no advance on overload"));
  assert.equal(result.overloaded, true);
  assert.equal(c.running, false);
  assert.equal(c.accumulator, 0);
});
test("playback speed scales elapsed simulation time, never dt", () => {
  for (const speed of [0.1, 0.25, 0.5, 1, 2]) {
    const c = new FixedClock(1 / 32768);
    c.speed = speed;
    c.start();
    let count = 0;
    for (let i = 0; i <= 600; i++) c.tick((i * 1000) / 60, (n) => (count += n));
    assert.equal(count, Math.floor((10 * speed) / c.dt));
    assert.equal(c.dt, 1 / 32768);
  }
});
