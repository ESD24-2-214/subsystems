{
  description = "A basic flake with a shell";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-24.05";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let pkgs = nixpkgs.legacyPackages.${system};
      in {
        devShells.default = pkgs.mkShell {

          packages = with pkgs; [ platformio nix-ld python3 clang-tools ];

          NIX_LD_LIBRARY_PATH = nixpkgs.lib.makeLibraryPath [
            pkgs.stdenv.cc.cc
            pkgs.openssl
            # ...
          ];

          shellHook = ''
            export LD_LIBRARY_PATH=$NIX_LD_LIBRARY_PATH
            pio run -t compiledb
          '';
        };
      });
}

# https://docs.platformio.org/en/latest/integration/ide/emacs.html
