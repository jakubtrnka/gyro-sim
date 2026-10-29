import { test, expect } from "@playwright/test";
import { readFile } from "node:fs/promises";

const snapshot = (page: any) =>
  page.evaluate(() => (window as any).__gyroTest.snapshot());
async function ready(page: any) {
  await page.goto("./?test");
  await expect(page.locator("#play")).toBeEnabled();
  await expect(page.locator("#status")).toContainText("Ready.");
}

test("production assets, WASM MIME, local-only requests, docs and responsive view", async ({
  page,
  request,
  baseURL,
}) => {
  const errors: string[] = [];
  const outsideSite: string[] = [];
  page.on("pageerror", (e) => errors.push(e.message));
  await page.route("**/*", (route) => {
    const url = route.request().url();
    if (
      url.startsWith("http") &&
      (new URL(url).origin !== new URL(baseURL!).origin ||
        !new URL(url).pathname.startsWith(new URL(baseURL!).pathname))
    ) {
      outsideSite.push(url);
      return route.abort();
    }
    return route.continue();
  });
  const wasm = page.waitForResponse((response) =>
    response.url().endsWith(".wasm"),
  );
  await ready(page);
  expect((await wasm).headers()["content-type"]).toContain("application/wasm");
  expect((await request.get("./missing-asset.js")).status()).toBe(404);
  for (const path of [
    "./docs/01-gyroscope-mechanics.md",
    "./docs/02-simulation-design.md",
    "./gyroscope-simulation-design.md",
    "./README.md",
    "./crates/gyro-core/tests/physics.rs",
    "./crates/gyro-wasm/src/bindings.d.ts",
    "./crates/gyro-wasm/examples/parity_reference.rs",
    "./web/tests/browser/lab.spec.ts",
  ])
    expect((await request.get(path)).status()).toBe(200);
  expect(
    await page.locator("a.brand").evaluate((a: HTMLAnchorElement) => a.href),
  ).toBe(baseURL);
  for (const href of await page
    .locator('a[href$=".md"]')
    .evaluateAll((links) =>
      links.map((link) => (link as HTMLAnchorElement).href),
    )) {
    expect(href.startsWith(baseURL!)).toBe(true);
    expect((await request.get(href)).status()).toBe(200);
  }
  expect((await snapshot(page)).step_index).toBe(0);
  await page.screenshot({ path: "/tmp/gyro-lab-desktop.png", fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  ).toBe(true);
  await page.screenshot({ path: "/tmp/gyro-lab-mobile.png", fullPage: true });
  expect(outsideSite).toEqual([]);
  expect(errors).toEqual([]);
});

test("dragging moves only the queued ghost until stepping; orbit does not change physics", async ({
  page,
}) => {
  await ready(page);
  const initial = await snapshot(page);
  const canvas = page.locator("#viewport canvas");
  const box = (await canvas.boundingBox())!;
  await page.mouse.move(box.x + box.width * 0.3, box.y + box.height * 0.65);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.58, box.y + box.height * 0.42, {
    steps: 12,
  });
  await page.mouse.up();
  expect(await snapshot(page)).toEqual(initial);
  const records = await page.evaluate(
    () => (window as any).__gyroTest.replay().commands,
  );
  expect(records.length).toBeGreaterThan(0);
  const moved = await page.evaluate(() =>
    (window as any).__gyroTest.advance(2048),
  );
  expect(moved.housing.orientation).not.toEqual(initial.housing.orientation);
  await page.locator("#orbit-mode").click();
  const before = await snapshot(page);
  const replayBefore = await page.evaluate(() =>
    (window as any).__gyroTest.replay(),
  );
  await page.mouse.move(box.x + box.width * 0.5, box.y + box.height * 0.5);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.65, {
    steps: 10,
  });
  await page.mouse.up();
  expect(await snapshot(page)).toEqual(before);
  expect(
    await page.evaluate(() => (window as any).__gyroTest.replay()),
  ).toEqual(replayBefore);
  await page.selectOption("#camera-view", "housing");
  await page.mouse.wheel(0, 100);
  expect(await snapshot(page)).toEqual(before);
});

test("start, pause, single step, hidden-tab pause, overload and reset", async ({
  page,
}) => {
  await ready(page);
  await page.locator("#step").click();
  expect((await snapshot(page)).step_index).toBe(1);
  await page.locator("#play").click();
  await expect
    .poll(async () => (await snapshot(page)).step_index)
    .toBeGreaterThan(1);
  await page.locator("#play").click();
  const paused = await snapshot(page);
  await page.waitForTimeout(100);
  expect(await snapshot(page)).toEqual(paused);
  await page.locator("#play").click();
  await page.evaluate(() => {
    Object.defineProperty(document, "hidden", {
      configurable: true,
      get: () => true,
    });
    document.dispatchEvent(new Event("visibilitychange"));
  });
  await expect(page.locator("#play")).toHaveText("▶ Start");
  const hidden = await snapshot(page);
  await page.waitForTimeout(100);
  expect(await snapshot(page)).toEqual(hidden);
  await page.evaluate(() => {
    Object.defineProperty(document, "hidden", {
      configurable: true,
      get: () => false,
    });
    document.dispatchEvent(new Event("visibilitychange"));
  });
  expect(await snapshot(page)).toEqual(hidden);
  expect(
    await page.evaluate(() => (window as any).__gyroTest.overload()),
  ).toEqual({ steps: 0, overloaded: true });
  expect(await snapshot(page)).toEqual(hidden);
  await page.locator("#reset").click();
  expect((await snapshot(page)).step_index).toBe(0);
});

test("larger rotation rings expose named handles and rotate only the housing target", async ({
  page,
}) => {
  await ready(page);
  const initial = await snapshot(page);
  const box = (await page.locator("#viewport canvas").boundingBox())!;
  let handle: { x: number; y: number } | undefined;
  // Pick from the expanded outer control region, outside the old ~40 px radius.
  for (const radius of [0.18, 0.22, 0.26]) {
    for (let angle = 0; angle < 2 * Math.PI; angle += Math.PI / 8) {
      const point = {
        x: box.x + box.width / 2 + box.height * radius * Math.cos(angle),
        y: box.y + box.height / 2 + box.height * radius * Math.sin(angle),
      };
      await page.mouse.move(point.x, point.y);
      if (
        /^(Roll|Pitch|Yaw)/.test(
          (await page.locator("#rotation-feedback").textContent())!,
        )
      ) {
        handle = point;
        break;
      }
    }
    if (handle) break;
  }
  expect(handle).toBeDefined();
  await expect(page.locator("#rotation-feedback")).toContainText(
    "housing target",
  );
  await page.mouse.down();
  await page.mouse.move(handle!.x + 24, handle!.y + 18, { steps: 8 });
  await page.mouse.up();
  expect(await snapshot(page)).toEqual(initial);
  const commands = await page.evaluate(
    () => (window as any).__gyroTest.replay().commands,
  );
  expect(commands.length).toBeGreaterThan(0);
  expect(commands.at(-1).command.SetHousingTarget).not.toEqual([0, 0, 0, 1]);
});

test("aircraft reference follows actual housing and camera, not a paused ghost target", async ({
  page,
}) => {
  await ready(page);
  const art = page.locator("#aircraft-view [data-aircraft]");
  const initialArt = await art.innerHTML();
  await page.locator("#roll").fill("180");
  await page.locator("#apply-pose").click();
  expect(await art.innerHTML()).toBe(initialArt);
  for (let i = 0; i < 4; i++)
    await page.evaluate(() => (window as any).__gyroTest.advance(32768));
  await expect(page.locator("#aircraft-view")).toContainText("Top faces down");
  const invertedArt = await art.innerHTML();
  expect(invertedArt).not.toBe(initialArt);
  await page
    .locator("#viewport")
    .screenshot({ path: "/tmp/gyro-orientation-inverted.png" });
  const before = await snapshot(page);
  await page.locator("#orbit-mode").click();
  const box = (await page.locator("#viewport canvas").boundingBox())!;
  await page.mouse.move(box.x + box.width * 0.45, box.y + box.height * 0.45);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.6, box.y + box.height * 0.55, {
    steps: 8,
  });
  await page.mouse.up();
  await expect.poll(() => art.innerHTML()).not.toBe(invertedArt);
  expect(await snapshot(page)).toEqual(before);
  await page.selectOption("#camera-view", "housing");
  await expect(page.locator("#aircraft-view")).toContainText("Top faces down");
  expect(await snapshot(page)).toEqual(before);
  await page
    .locator("#viewport")
    .screenshot({ path: "/tmp/gyro-orientation-housing-view.png" });
  await page.setViewportSize({ width: 390, height: 844 });
  await expect(page.locator("#level-housing")).toBeInViewport();
  const inset = (await page.locator("#aircraft-inset").boundingBox())!;
  const mobile = (await page.locator("#viewport").boundingBox())!;
  const stage = (await page.locator("#viewport canvas").boundingBox())!;
  expect(inset.x).toBeGreaterThan(mobile.x + mobile.width / 2);
  expect(inset.x + inset.width).toBeLessThanOrEqual(mobile.x + mobile.width);
  expect(inset.y).toBeGreaterThanOrEqual(stage.y + stage.height);
  await page
    .locator("#viewport")
    .screenshot({ path: "/tmp/gyro-orientation-mobile.png" });
});

test("level housing preserves the running experiment and smoothly returns all presets", async ({
  page,
}) => {
  await ready(page);
  for (const preset of ["Attitude", "Directional", "TurnIndicator"]) {
    await page.locator(`[data-preset=${preset}]`).click();
    await page.evaluate(() =>
      (window as any).__gyroTest.queue({
        SetHousingTarget: [Math.sin(0.45), 0, 0, Math.cos(0.45)],
      }),
    );
    await page.evaluate(() => (window as any).__gyroTest.advance(32768));
    const before = await snapshot(page);
    const log = await page.evaluate(() => (window as any).__gyroTest.replay());
    await page.locator("#level-housing").click();
    expect(await snapshot(page)).toEqual(before);
    const queued = await page.evaluate(() =>
      (window as any).__gyroTest.replay(),
    );
    expect(queued.end_step).toBe(log.end_step);
    expect(queued.commands.slice(0, -1)).toEqual(log.commands);
    expect(queued.commands.at(-1).command).toEqual({
      SetHousingTarget: [0, 0, 0, 1],
    });
    await page.locator("#step").click();
    const next = await snapshot(page);
    expect(next.step_index).toBe(before.step_index + 1);
    expect(next.drive).toEqual(before.drive);
    const change = Math.hypot(
      ...next.housing.orientation.map(
        (x: number, i: number) => x - before.housing.orientation[i],
      ),
    );
    expect(change).toBeLessThan(0.001);
    expect(
      Math.abs(next.joints.rates[preset === "TurnIndicator" ? 1 : 2]),
    ).toBeGreaterThan(1000);
    for (let i = 0; i < 3; i++)
      await page.evaluate(() => (window as any).__gyroTest.advance(32768));
    const after = await snapshot(page);
    expect(Math.hypot(...after.housing.orientation.slice(0, 3))).toBeLessThan(
      0.01,
    );
    expect(after.step_index).toBe(before.step_index + 1 + 3 * 32768);
  }
  const savedStep = (await snapshot(page)).step_index;
  await page.locator("#play").click();
  await page.locator("#level-housing").click();
  await expect(page.locator("#play")).toHaveText("Ⅱ Pause");
  expect((await snapshot(page)).step_index).toBeGreaterThanOrEqual(savedStep);
  await page.locator("#play").click();
});

test("level housing returns to an imported run's original pose, not its replay end pose", async ({
  page,
}) => {
  await ready(page);
  const definition = await page.evaluate(() =>
    (window as any).__gyroTest.definition(),
  );
  const home = [Math.sin(0.3), 0, 0, Math.cos(0.3)];
  definition.initial.housing = { Fixed: { orientation: home } };
  const replay = {
    version: 1,
    ...definition,
    commands: [
      {
        step_index: 0,
        sequence: 0,
        command: { SetHousingTarget: [0, 0, 0, 1] },
      },
    ],
    end_step: 32768,
  };
  await page.locator("#import-replay").setInputFiles({
    name: "tilted-start.json",
    mimeType: "application/json",
    buffer: Buffer.from(JSON.stringify(replay)),
  });
  await expect(page.locator("#status")).toContainText("Replay reconstructed");
  const before = await snapshot(page);
  expect(before.housing.orientation).not.toEqual(home);
  await page.locator("#level-housing").click();
  expect(await snapshot(page)).toEqual(before);
  const log = await page.evaluate(() => (window as any).__gyroTest.replay());
  const target = log.commands.at(-1).command.SetHousingTarget;
  home.forEach((x, i) => expect(target[i]).toBeCloseTo(x, 12));
});

test("same commands at 30, 60 and 120 display Hz give identical WASM snapshots", async ({
  page,
}) => {
  await ready(page);
  for (const preset of ["Attitude", "Directional", "TurnIndicator"]) {
    await page.locator(`[data-preset=${preset}]`).click();
    await page.evaluate(() =>
      (window as any).__gyroTest.queue({
        SetHousingTarget: [0.17, -0.29, 0.31, 0.89],
      }),
    );
    const states = [];
    for (const fps of [30, 60, 120])
      states.push(
        await page.evaluate(
          (f) => (window as any).__gyroTest.schedule(f, 1),
          fps,
        ),
      );
    expect(states[0]).toEqual(states[1]);
    expect(states[1]).toEqual(states[2]);
  }
});

test("keyboard targets, instrument signs, queued spin controls and heading setting", async ({
  page,
}) => {
  await ready(page);
  await page.locator("#roll").fill("20");
  await page.locator("#apply-pose").click();
  expect((await snapshot(page)).step_index).toBe(0);
  await expect(page.locator("#pending-target")).toContainText("Ghost moved");
  for (let i = 0; i < 3; i++)
    await page.evaluate(() => (window as any).__gyroTest.advance(32768));
  expect((await snapshot(page)).instrument.Attitude.bank).toBeGreaterThan(0.3);
  await page.locator("#target-rpm").fill("-12000");
  await page.locator("#apply-rpm").click();
  const before = await snapshot(page);
  await page.locator("#step").click();
  const after = await snapshot(page);
  expect(after.drive.target_spin).toBeLessThan(0);
  expect(after.joints.rates[2]).toBeGreaterThan(1000);
  expect(Math.abs(after.joints.rates[2] - before.joints.rates[2])).toBeLessThan(
    1,
  );
  await page.locator("[data-preset=Directional]").click();
  await page.locator("#heading").fill("90");
  await page.locator("#set-heading").click();
  await page.locator("#step").click();
  expect((await snapshot(page)).instrument.Directional.heading).toBeCloseTo(
    Math.PI / 2,
    8,
  );
  await page.locator("[data-motion=Yaw]").click();
  await page.locator("#play").click();
  for (let i = 0; i < 2; i++)
    await page.evaluate(() => (window as any).__gyroTest.advance(32768));
  expect((await snapshot(page)).instrument.Directional.heading).toBeGreaterThan(
    0,
  );
  await page.locator("[data-preset=TurnIndicator]").click();
  await page.locator("[data-motion=SlowYaw]").click();
  await page.locator("#play").click();
  // Sample during the 1–5 s constant-rate hold, not after the 6 s ramp-down.
  // Clicking Pause may allow a few real animation frames before it takes effect.
  for (let i = 0; i < 2; i++)
    await page.evaluate(() => (window as any).__gyroTest.advance(32768));
  expect(
    (await snapshot(page)).instrument.TurnIndicator.indicated_rate,
  ).toBeGreaterThan(0);
});

test("settings reset to zero spin, invalid inertia keeps state, replay import is atomic", async ({
  page,
}) => {
  await ready(page);
  await page.locator("summary").click();
  await page.locator("#initial-rpm").fill("0");
  await page.locator("#apply-settings").click();
  await expect(page.locator("#status")).toContainText("Settings applied");
  expect((await snapshot(page)).joints.rates[2]).toBe(0);
  const before = await snapshot(page);
  await page.locator("#axial").fill("0.001");
  await page.locator("#transverse").fill("0.000001");
  await page.locator("#apply-settings").click();
  await expect(page.locator("#status")).toContainText("triangle inequalities");
  expect(await snapshot(page)).toEqual(before);
  await page.locator("#import-replay").setInputFiles({
    name: "bad.json",
    mimeType: "application/json",
    buffer: Buffer.from('{"version":999}'),
  });
  await expect(page.locator("#status")).toContainText("Import failed");
  expect(await snapshot(page)).toEqual(before);
});

test("export/import restores complete state and retains paused commands", async ({
  page,
}) => {
  await ready(page);
  await page.evaluate(() =>
    (window as any).__gyroTest.queue({
      SetHousingTarget: [0.17, -0.29, 0.31, 0.89],
    }),
  );
  await page.evaluate(() => (window as any).__gyroTest.advance(4096));
  await page.evaluate(() =>
    (window as any).__gyroTest.queue({ SetTargetSpin: 50 }),
  );
  await page.evaluate(() =>
    (window as any).__gyroTest.queue({ SetHousingTarget: [0, 0, 0, 1] }),
  );
  const expected = await snapshot(page);
  const log = await page.evaluate(() => (window as any).__gyroTest.replay());
  const downloadPromise = page.waitForEvent("download");
  await page.locator("#export-replay").click();
  const download = await downloadPromise;
  const bytes = await readFile((await download.path())!);
  expect(JSON.parse(bytes.toString())).toEqual(log);
  await page.locator("#reset").click();
  await page.locator("#import-replay").setInputFiles({
    name: "replay.json",
    mimeType: "application/json",
    buffer: bytes,
  });
  await expect(page.locator("#status")).toContainText("Replay reconstructed");
  await expect(page.locator("#pending-target")).toContainText(
    "Target restored from replay",
  );
  expect(await snapshot(page)).toEqual(expected);
  expect(
    await page.evaluate(() => (window as any).__gyroTest.replay()),
  ).toEqual(log);
  await page.locator("#step").click();
  expect((await snapshot(page)).drive.target_spin).toBe(50);
});

test("initialization failure leaves controls disabled with an explanation", async ({
  page,
}) => {
  await page.route("**/*.wasm", (route) => route.abort());
  await page.goto("./");
  await expect(page.locator("#status")).toContainText("Could not start");
  await expect(page.locator("#play")).toBeDisabled();
});

test("axis alignment reports undefined bank instead of inventing a horizon", async ({
  page,
}) => {
  await ready(page);
  const definition = await page.evaluate(() =>
    (window as any).__gyroTest.definition(),
  );
  definition.initial.joints.angles[1] = Math.PI / 2;
  const replay = { version: 1, ...definition, commands: [], end_step: 0 };
  await page.locator("#import-replay").setInputFiles({
    name: "alignment.json",
    mimeType: "application/json",
    buffer: Buffer.from(JSON.stringify(replay)),
  });
  await expect(page.locator("#status")).toContainText("Replay reconstructed");
  await expect(page.locator("#instrument-face")).toContainText(
    "BANK UNDEFINED",
  );
  await expect(page.locator("#warnings")).toContainText(
    "Gimbal axes nearly aligned",
  );
  expect((await snapshot(page)).diagnostics.poorly_conditioned).toBe(false);
});

test("replay reconstruction can be cancelled without replacing the old run", async ({
  page,
}) => {
  await ready(page);
  const before = await snapshot(page);
  const definition = await page.evaluate(() =>
    (window as any).__gyroTest.definition(),
  );
  const replay = {
    version: 1,
    ...definition,
    commands: [],
    end_step: 32768 * 300,
  };
  await page.locator("#import-replay").setInputFiles({
    name: "long.json",
    mimeType: "application/json",
    buffer: Buffer.from(JSON.stringify(replay)),
  });
  await expect(page.locator("#cancel-import")).toBeVisible();
  await page.locator("#cancel-import").click();
  await expect(page.locator("#status")).toContainText("cancelled");
  expect(await snapshot(page)).toEqual(before);
  await expect(page.locator("#play")).toBeEnabled();
});

test("record achieved playback rate including rendering", async ({ page }) => {
  await ready(page);
  const results = [];
  for (const preset of ["Attitude", "Directional", "TurnIndicator"]) {
    await page.locator(`[data-preset=${preset}]`).click();
    await page.locator("#roll").fill("20");
    await page.locator("#apply-pose").click();
    await page.evaluate(() => {
      const state = window as any;
      state.__frameTimes = [];
      const token = (state.__frameToken = (state.__frameToken ?? 0) + 1);
      const sample = (now: number) => {
        if (state.__frameToken !== token) return;
        state.__frameTimes.push(now);
        requestAnimationFrame(sample);
      };
      requestAnimationFrame(sample);
    });
    await page.locator("#play").click();
    const start = Date.now();
    await page.waitForTimeout(2200);
    const overloaded = !(await page.locator("#play").textContent())?.includes(
      "Pause",
    );
    if (!overloaded) await page.locator("#play").click();
    const s = await snapshot(page);
    const times = await page.evaluate(() => {
      const state = window as any;
      state.__frameToken++;
      return state.__frameTimes as number[];
    });
    const fps = ((times.length - 1) * 1000) / (times.at(-1)! - times[0]);
    results.push({
      preset,
      wall_seconds: (Date.now() - start) / 1000,
      sim_seconds: s.time_seconds,
      fps,
      overloaded,
    });
    expect(s.step_index).toBeGreaterThan(0);
  }
  console.log(
    "Achieved playback including software-rendered scene:",
    JSON.stringify(results),
  );
});

test("record browser compute throughput at accepted physics resolution", async ({
  page,
}) => {
  await ready(page);
  const results = [];
  for (const preset of ["Attitude", "Directional", "TurnIndicator"]) {
    await page.locator(`[data-preset=${preset}]`).click();
    await page.evaluate(() =>
      (window as any).__gyroTest.queue({
        SetHousingTarget: [0.17, -0.29, 0.31, 0.89],
      }),
    );
    const result = await page.evaluate(() =>
      (window as any).__gyroTest.benchmark(),
    );
    results.push({
      preset,
      ...result,
      time_scale: result.simulated_seconds / (result.wall_ms / 1000),
    });
    expect(
      Number.isFinite((await snapshot(page)).energy_balance_residual),
    ).toBe(true);
  }
  console.log(
    "Browser physics throughput (headless Chromium, software GL):",
    JSON.stringify(results),
  );
});
