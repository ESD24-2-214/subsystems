{
  description = "A Nix-flake-based C/C++ development environment";

  inputs.nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0.1.*.tar.gz";

  outputs = { self, nixpkgs }:
    let
      supportedSystems =
        [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forEachSupportedSystem = f:
        nixpkgs.lib.genAttrs supportedSystems
        (system: f { pkgs = import nixpkgs { inherit system; }; });
    in {
      devShells = forEachSupportedSystem ({ pkgs }: {
        default = pkgs.mkShell.override {
          # Override stdenv in order to change compiler:
          # stdenv = pkgs.clangStdenv;
        } {
          CPLUS_INCLUDE_PATH = "${pkgs.glibc.dev}/include";
          CPATH = "${pkgs.stdenv.cc.cc}/include:${pkgs.stdenv.cc.libc}/include";
          packages = with pkgs;
            [
              # Dev tings
              cmake
              gnumake
              bear

              # C++ stuff
              clang-tools
              codespell
              conan
              cppcheck
              doxygen
              gtest
              lcov
              vcpkg
              vcpkg-tool

            ] ++ (if system == "aarch64-darwin" then [ ] else [ gdb ]);
          buildInputs = with pkgs; [
            clang
            clang-tools # For clangd
            stdenv.cc.cc.lib
          ];
        };
      });
    };
}

