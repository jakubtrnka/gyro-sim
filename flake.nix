{
  description = "Aircraft gyroscope simulator: Rust, WebAssembly, and a local browser lab";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    { nixpkgs, rust-overlay, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
      project = forAllSystems (
        system:
        import ./nix {
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ rust-overlay.overlays.default ];
          };
        }
      );
    in
    {
      packages = forAllSystems (system: project.${system}.packages);
      apps = forAllSystems (system: project.${system}.apps);
      devShells = forAllSystems (system: project.${system}.devShells);
      checks = forAllSystems (system: project.${system}.checks);
      formatter = forAllSystems (system: project.${system}.formatter);
    };
}
