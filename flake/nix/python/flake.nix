{
  description = "Python devshell: PyTorch with Intel iGPU (XPU) support, managed via uv";

  inputs = {
    nixpkgs.url = "nixpkgs/nixos-26.05";
  };

  outputs =
    { self, nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };

      # Runtime libraries the prebuilt PyTorch XPU wheels (and the Level Zero
      # loader they use) need to find at dynamic-link time.
      libPath = pkgs.lib.makeLibraryPath [
        pkgs.stdenv.cc.cc.lib
        pkgs.zlib
        pkgs.glib
        pkgs.level-zero
        pkgs.intel-compute-runtime
        pkgs.intel-graphics-compiler
        pkgs.intel-media-driver
        pkgs.vpl-gpu-rt
        pkgs.libGL
        pkgs.oneDNN
        pkgs.llvmPackages.libllvm
      ];
    in
    {

      devShells.${system}.default = pkgs.mkShell {

        packages = with pkgs; [
          uv
          python3
          intel-gpu-tools # intel_gpu_top, for confirming iGPU load
          clinfo # OpenCL device listing, useful for sanity checks
        ];

        env = {
          LD_LIBRARY_PATH = libPath;

          # Same knobs used by the system-wide intel-gpu-xpu NixOS module,
          # kept here so this devshell also works standalone (e.g. via
          # `nix develop github:...` on another machine).
          NEOReadDebugKeys = "1";
          OverrideGpuAddressSpace = "48";

          UV_PYTHON_PREFERENCE = "only-system";
        };

        shellHook = ''
          nix registry add python "path:/home/warleon/.dotfiles/dev/python" >/dev/null 2>&1 || true

          # Creates .venv via uv if it doesn't exist yet, then activates it.
          venv() {
            [ -d .venv ] || uv venv
            source .venv/bin/activate
          }

          echo "python devshell ready."
          echo "  venv  # create (if needed) and activate .venv"
        '';
      };
    };
}
