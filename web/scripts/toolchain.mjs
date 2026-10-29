import { spawnSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

export const workspaceRoot = fileURLToPath(new URL("../../", import.meta.url));
export function rustToolchainArgs() {
  const pin = readFileSync(
    path.join(workspaceRoot, "rust-toolchain.toml"),
    "utf8",
  ).match(/channel\s*=\s*"([^"]+)"/)[1];
  let selected = process.env.GYRO_RUST_TOOLCHAIN;
  // Nix provides the exact compiler directly, without rustup's +toolchain syntax.
  if (!selected) {
    const direct = spawnSync("rustc", ["--version"], { encoding: "utf8" });
    if (direct.status === 0 && direct.stdout?.startsWith(`rustc ${pin} `))
      return [];
  }
  if (
    !selected &&
    spawnSync("rustup", ["which", "--toolchain", pin, "rustc"], {
      encoding: "utf8",
    }).status === 0
  )
    selected = pin;
  if (!selected) {
    const stable = spawnSync("rustc", ["+stable", "--version"], {
      encoding: "utf8",
    });
    if (stable.stdout?.startsWith(`rustc ${pin} `)) selected = "stable";
  }
  selected ??= pin;
  const result = spawnSync("rustc", [`+${selected}`, "--version"], {
    encoding: "utf8",
  });
  if (result.status !== 0 || !result.stdout.startsWith(`rustc ${pin} `))
    throw new Error(
      `Install Rust ${pin}; native and WASM builds must use this exact version. ${result.stderr}`,
    );
  return [`+${selected}`];
}
