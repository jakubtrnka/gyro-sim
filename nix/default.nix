{ pkgs }:
let
  inherit (pkgs) lib;
  root = ../.;
  version = (builtins.fromTOML (builtins.readFile ../Cargo.toml)).workspace.package.version;
  rust = pkgs.rust-bin.fromRustupToolchainFile ../rust-toolchain.toml;
  rustPlatform = pkgs.makeRustPlatform {
    cargo = rust;
    rustc = rust;
  };
  nodejs = pkgs.nodejs_26;
  fontsConf = pkgs.makeFontsConf {
    fontDirectories = [
      pkgs.dejavu_fonts
      pkgs.liberation_ttf
    ];
  };
  wasmBindgenVersion =
    (lib.findFirst (p: p.name == "wasm-bindgen") (throw "wasm-bindgen is missing from Cargo.lock")
      (builtins.fromTOML (builtins.readFile ../Cargo.lock)).package
    ).version;
  wasmBindgen =
    assert lib.assertMsg (
      pkgs.wasm-bindgen-cli.version == wasmBindgenVersion
    ) "Update the pinned nixpkgs input to provide wasm-bindgen-cli ${wasmBindgenVersion}";
    pkgs.wasm-bindgen-cli;

  # Explicit inputs keep target/, node_modules/, generated bindings, and local tools
  # out of the build, including when evaluating a path: flake during development.
  rustFiles = lib.fileset.unions [
    ../Cargo.toml
    ../Cargo.lock
    ../rust-toolchain.toml
    ../crates
  ];
  rustSrc = lib.fileset.toSource {
    inherit root;
    fileset = rustFiles;
  };
  src = lib.fileset.toSource {
    inherit root;
    fileset = lib.fileset.unions [
      rustFiles
      ../README.md
      ../LICENSE
      ../docs
      ../gyroscope-simulation-design.md
      ../web/package.json
      ../web/package-lock.json
      ../web/index.html
      ../web/tsconfig.json
      ../web/playwright.config.ts
      ../web/vite.config.ts
      ../web/src
      ../web/scripts
      ../web/tests
    ];
  };
  cargoDeps = rustPlatform.importCargoLock { lockFile = ../Cargo.lock; };
  rustArgs = {
    inherit version cargoDeps;
    src = rustSrc;
    strictDeps = true;
    meta.license = lib.licenses.mit;
  };

  server = rustPlatform.buildRustPackage (
    rustArgs
    // {
      pname = "gyro-server";
      cargoBuildFlags = [
        "-p"
        "gyro-server"
      ];
      cargoTestFlags = [
        "-p"
        "gyro-server"
      ];
      meta.mainProgram = "gyro-server";
    }
  );

  wasm = rustPlatform.buildRustPackage (
    rustArgs
    // {
      pname = "gyro-wasm";
      nativeBuildInputs = [ wasmBindgen ];
      # buildRustPackage's default build hook selects the native target.
      buildPhase = ''
        runHook preBuild
        cargo build --release --frozen -j "$NIX_BUILD_CORES" -p gyro-wasm --target wasm32-unknown-unknown
        runHook postBuild
      '';
      doCheck = false; # Executed through Node in checks.native below.
      dontStrip = true;
      installPhase = ''
        runHook preInstall
        for target in web nodejs; do
          wasm-bindgen --target "$target" --out-dir "$out/$target" \
            target/wasm32-unknown-unknown/release/gyro_wasm.wasm
        done
        printf '{"type":"commonjs"}\n' > "$out/nodejs/package.json"
        runHook postInstall
      '';
    }
  );

  npmDeps = pkgs.fetchNpmDeps {
    src = lib.fileset.toSource {
      root = ../web;
      fileset = lib.fileset.unions [
        ../web/package.json
        ../web/package-lock.json
      ];
    };
    hash = "sha256-IB5L8p4DcGdsA6PJCflF41EPf10vH7ILwrf7VZXoiKc=";
  };
  web = pkgs.buildNpmPackage {
    pname = "gyro-web";
    inherit
      version
      src
      nodejs
      npmDeps
      ;
    npmRoot = "web";
    env.PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD = "1";
    postPatch = ''
      cp -r ${wasm} web/generated
    '';
    buildPhase = ''
      runHook preBuild
      npm --prefix web run build
      runHook postBuild
    '';
    doCheck = true;
    checkPhase = ''
      runHook preCheck
      npm --prefix web test
      npm --prefix web run check:format
      runHook postCheck
    '';
    installPhase = ''
      runHook preInstall
      cp -r web/dist "$out"
      runHook postInstall
    '';
  };

  package = pkgs.symlinkJoin {
    name = "gyro-sim-${version}";
    paths = [ server ];
    nativeBuildInputs = [ pkgs.makeWrapper ];
    postBuild = ''
      mkdir -p "$out/share/gyro-sim"
      ln -s ${web} "$out/share/gyro-sim/web"
      makeWrapper ${lib.getExe server} "$out/bin/gyro-sim" \
        --add-flags "--assets $out/share/gyro-sim/web"
    '';
    meta = {
      description = "Aircraft gyroscope teaching lab with bundled browser assets";
      license = lib.licenses.mit;
      mainProgram = "gyro-sim";
    };
  };

  native = rustPlatform.buildRustPackage (
    rustArgs
    // {
      pname = "gyro-native-checks";
      inherit src;
      nativeBuildInputs = [ nodejs ];
      cargoBuildFlags = [
        "--workspace"
        "--all-targets"
      ];
      cargoTestFlags = [ "--workspace" ];
      preCheck = ''
        cargo fmt --all -- --check
        cargo clippy --workspace --all-targets --frozen -- -D warnings
      '';
      postCheck = ''
        cp -r ${wasm} web/generated
        npm --prefix web run test:parity
      '';
      installPhase = "touch $out";
    }
  );

  browser = web.overrideAttrs {
    pname = "gyro-browser-checks";
    nativeBuildInputs = web.nativeBuildInputs ++ [ pkgs.chromium ];
    CHROMIUM_PATH = lib.getExe pkgs.chromium;
    # The build sandbox has no host fonts or /etc/fonts configuration.
    FONTCONFIG_FILE = fontsConf;
    buildPhase = ''
      runHook preBuild
      cp -r ${web} web/dist
      mkdir -p target/release
      ln -s ${lib.getExe server} target/release/gyro-server
      runHook postBuild
    '';
    checkPhase = ''
      runHook preCheck
      export XDG_CACHE_HOME="$TMPDIR/chromium-cache"
      npm --prefix web run test:browser
      runHook postCheck
    '';
    installPhase = "touch $out";
  };

  browser-pages = browser.overrideAttrs {
    pname = "gyro-pages-browser-checks";
    GYRO_TEST_BASE_PATH = "/gyro-sim/";
    buildPhase = ''
      runHook preBuild
      mkdir -p web/dist target/release
      # The server requires a root index; only the repository subpath has assets.
      touch web/dist/index.html
      cp -r ${web} web/dist/gyro-sim
      ln -s ${lib.getExe server} target/release/gyro-server
      runHook postBuild
    '';
  };
in
{
  packages = {
    inherit server wasm web;
    default = package;
  };
  apps.default = {
    type = "app";
    program = lib.getExe package;
    meta.description = package.meta.description;
  };
  devShells.default = pkgs.mkShell {
    packages = [
      rust
      wasmBindgen
      nodejs
      pkgs.chromium
      pkgs.nixfmt
    ];
    WASM_BINDGEN = lib.getExe wasmBindgen;
    CHROMIUM_PATH = lib.getExe pkgs.chromium;
    FONTCONFIG_FILE = fontsConf;
    PLAYWRIGHT_SKIP_BROWSER_DOWNLOAD = "1";
  };
  checks = {
    inherit
      native
      web
      browser
      browser-pages
      ;
    workflows =
      pkgs.runCommand "gyro-workflow-check"
        {
          nativeBuildInputs = [
            pkgs.actionlint
            pkgs.shellcheck
          ];
        }
        ''
          actionlint ${../.github/workflows/ci.yml}
          touch "$out"
        '';
    package = pkgs.runCommand "gyro-package-check" { nativeBuildInputs = [ pkgs.python3 ]; } ''
      python ${./check-package.py} ${lib.getExe package} ${web} ${src}
      touch "$out"
    '';
    nix-format = pkgs.runCommand "gyro-nix-format-check" { nativeBuildInputs = [ pkgs.nixfmt ]; } ''
      nixfmt --check ${../flake.nix} ${./default.nix}
      touch "$out"
    '';
  };
  formatter = pkgs.nixfmt;
}
