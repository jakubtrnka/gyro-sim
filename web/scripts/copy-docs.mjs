import { cpSync, mkdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";
const root = fileURLToPath(new URL("../../", import.meta.url));
const out = path.join(root, "web/dist");
for (const relative of [
  "README.md",
  "gyroscope-simulation-design.md",
  "docs",
  "crates/gyro-core/tests",
  "crates/gyro-core/examples",
  "crates/gyro-wasm/src",
  "crates/gyro-wasm/examples",
  "web/tests",
]) {
  const dest = path.join(out, relative);
  mkdirSync(path.dirname(dest), { recursive: true });
  cpSync(path.join(root, relative), dest, { recursive: true });
}
