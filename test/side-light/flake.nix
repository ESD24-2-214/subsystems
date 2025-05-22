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

          packages = with pkgs;
            [
              # texlive.combined.scheme-medium

              (rWrapper.override {
                packages = with rPackages; [ jpeg mosaic tidyverse ];
              })

              # (rstudioWrapper.override {
              #   packages = with rPackages; [
              #     rmarkdown
              #     jpeg
              #     mosaic

              #   ];
              # })
            ];

        };
      });
}

# https://docs.platformio.org/en/latest/integration/ide/emacs.html
