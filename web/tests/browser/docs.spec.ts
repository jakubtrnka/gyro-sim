import { test, expect } from "@playwright/test";

// The published pages, including equations and diagrams, are entirely static.
test.use({ javaScriptEnabled: false });

test("documentation renders without JavaScript or external assets", async ({
  page,
  context,
  baseURL,
}) => {
  const outsideSite: string[] = [];
  const failures: string[] = [];
  const requests: string[] = [];
  context.on("request", (request) => requests.push(request.url()));
  context.on("requestfailed", (request) => failures.push(request.url()));
  context.on("response", (response) => {
    if (response.status() >= 400) failures.push(response.url());
  });
  await context.route("**/*", (route) => {
    const url = route.request().url();
    if (url.startsWith("http") && !url.startsWith(baseURL!)) {
      outsideSite.push(url);
      return route.abort();
    }
    return route.continue();
  });
  await page.goto("./");
  const opened = context.waitForEvent("page");
  await page.getByRole("link", { name: "Read the mechanics" }).click();
  const document = await opened;
  await expect(document).toHaveURL(
    new URL("docs/01-gyroscope-mechanics.html", baseURL).href,
  );
  await expect(document.locator("h1")).toHaveText(
    "Gyroscope mechanics: from a turn indicator to two gimbals",
  );
  await expect(document.locator(".katex-display").first()).toBeVisible();
  await expect(document.locator("math").first()).toBeAttached();
  await expect(document.locator(".diagram svg")).toHaveCount(2);
  await expect(document.locator(".diagram svg").first()).toBeVisible();
  await expect(document.locator("script, .katex-error")).toHaveCount(0);
  expect(requests.some((url) => url.endsWith(".woff2"))).toBe(true);
  await document.screenshot({ path: "/tmp/gyro-docs-top.png" });

  await document
    .getByRole("navigation", { name: "On this page" })
    .getByRole("link", { name: "2. The mechanism and its coordinates" })
    .click();
  await expect(document).toHaveURL(/#2-the-mechanism-and-its-coordinates$/);
  await expect(
    document.locator('h2[id="2-the-mechanism-and-its-coordinates"]'),
  ).toBeInViewport();
  await document.screenshot({ path: "/tmp/gyro-docs-desktop.png" });

  const nav = document.getByRole("navigation", { name: "Documentation" });
  await nav
    .getByRole("link", { name: "Simulation design", exact: true })
    .click();
  await expect(document).toHaveURL(
    new URL("docs/02-simulation-design.html", baseURL).href,
  );
  await expect(document.locator(".diagram svg")).toHaveCount(2);
  await nav.getByRole("link", { name: "Project guide" }).click();
  await expect(document).toHaveURL(new URL("README.html", baseURL).href);
  // Follow a rewritten Markdown link and its GitHub-compatible heading anchor.
  await document
    .getByRole("link", { name: "implementation sequence", exact: true })
    .click();
  await expect(
    document.locator('h2[id="10-implementation-sequence-and-exit-criteria"]'),
  ).toBeInViewport();
  await nav.getByRole("link", { name: "Original proposal" }).click();
  await expect(document).toHaveURL(
    new URL("gyroscope-simulation-design.html", baseURL).href,
  );
  await expect(document.locator(".katex-display").first()).toBeVisible();

  await nav.getByRole("link", { name: "Gyroscope mechanics" }).click();
  await document.setViewportSize({ width: 390, height: 844 });
  await document.locator(".mobile-contents summary").click();
  await document
    .getByRole("navigation", { name: "On this page" })
    .getByRole("link", { name: "2. The mechanism and its coordinates" })
    .click();
  await expect(
    document.locator('h2[id="2-the-mechanism-and-its-coordinates"]'),
  ).toBeInViewport();
  await document.locator(".diagram").first().scrollIntoViewIfNeeded();
  const diagram = document.locator(".diagram").first();
  expect(
    await diagram.evaluate(
      (element) => element.scrollWidth > element.clientWidth,
    ),
  ).toBe(true);
  await diagram.focus();
  await document.keyboard.press("ArrowRight");
  await expect
    .poll(() => diagram.evaluate((element) => element.scrollLeft))
    .toBeGreaterThan(0);
  await document.screenshot({ path: "/tmp/gyro-docs-mobile.png" });
  expect(
    await document.evaluate(
      () => document.documentElement.scrollWidth <= window.innerWidth,
    ),
  ).toBe(true);
  await document.getByRole("link", { name: "Back to simulation" }).click();
  await expect(document).toHaveURL(new URL("index.html", baseURL).href);
  await expect(
    document.getByRole("link", { name: "Read the mechanics" }),
  ).toBeVisible();
  expect(outsideSite).toEqual([]);
  expect(failures).toEqual([]);
});
