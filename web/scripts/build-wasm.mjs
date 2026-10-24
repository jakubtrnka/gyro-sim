import { spawnSync } from "node:child_process";
import { mkdirSync, writeFileSync, existsSync } from "node:fs";
import path from "node:path";
import { rustToolchain, workspaceRoot as root } from "./toolchain.mjs";

const toolchain = rustToolchain();
const cli =
  process.env.WASM_BINDGEN ??
  (existsSync(path.join(root, ".tools/wasm-bindgen/bin/wasm-bindgen"))
    ? path.join(root, ".tools/wasm-bindgen/bin/wasm-bindgen")
    : "wasm-bindgen");
function run(command, args) {
  const result = spawnSync(command, args, { cwd: root, stdio: "inherit" });
  if (result.error || result.status !== 0)
    throw result.error ?? new Error(`${command} exited ${result.status}`);
}
const version = spawnSync(cli, ["--version"], { encoding: "utf8" });
if (version.stdout?.trim() !== "wasm-bindgen 0.2.129")
  throw new Error(
    "Install wasm-bindgen-cli 0.2.129; see README. Crate and CLI versions must match.",
  );
run("cargo", [
  `+${toolchain}`,
  "build",
  "--release",
  "--locked",
  "-p",
  "gyro-wasm",
  "--target",
  "wasm32-unknown-unknown",
]);
const wasm = path.join(
  root,
  "target/wasm32-unknown-unknown/release/gyro_wasm.wasm",
);
for (const target of ["web", "nodejs"]) {
  const directory = path.join(root, "web/generated", target);
  mkdirSync(directory, { recursive: true });
  run(cli, ["--target", target, "--out-dir", directory, wasm]);
  if (target === "nodejs")
    writeFileSync(
      path.join(directory, "package.json"),
      '{"type":"commonjs"}\n',
    );
}
