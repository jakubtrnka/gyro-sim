import { defineConfig } from "@playwright/test";
import { existsSync } from "node:fs";
const port = Number(process.env.GYRO_TEST_PORT ?? 3000);
if (!Number.isInteger(port) || port < 1 || port > 65535)
  throw new Error("GYRO_TEST_PORT must be an integer from 1 to 65535");
const baseURL = `http://127.0.0.1:${port}`;
export default defineConfig({
  testDir: "./tests/browser",
  fullyParallel: false,
  workers: 1,
  timeout: 60_000,
  use: {
    baseURL,
    viewport: { width: 1440, height: 1100 },
    launchOptions: {
      executablePath:
        process.env.CHROMIUM_PATH ??
        (existsSync("/usr/bin/chromium") ? "/usr/bin/chromium" : undefined),
      args: [
        "--enable-unsafe-swiftshader",
        "--use-gl=angle",
        "--use-angle=swiftshader",
      ],
    },
    screenshot: "only-on-failure",
    trace: "retain-on-failure",
  },
  webServer: {
    command: `../target/release/gyro-server --assets dist --port ${port}`,
    url: baseURL,
    reuseExistingServer: false,
    timeout: 20_000,
  },
});
