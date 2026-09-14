{
  description = "ROS 1 Noetic Development Environment";

  inputs = {
    #nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/ros1-25.05";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nixgl.url = "github:nix-community/nixGL";
    pythonDev.url = "path:/home/warleon/.dotfiles/dev/python";
    pythonDev.inputs.nixpkgs.follows = "nixpkgs";
  };
  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      nixgl,
      pythonDev,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            nixgl.overlays.default
            (final: prev: {
              rosPackages = prev.rosPackages // {
                noetic = prev.rosPackages.noetic.overrideScope (
                  rosFinal: rosPrev: {
                    ouster-ros = rosFinal.callPackage ./nix/ouster-ros.nix { };
                    faster-lio = rosFinal.callPackage ./nix/faster-lio.nix { };
                  }
                );
              };
            })
          ];
          config = {
            permittedInsecurePackages = [
              "freeimage-3.18.0-unstable-2024-04-18"
            ];
          };
        };
      in
      {
        devShells.default = pkgs.mkShell {
          name = "ROS";
          # Pulls in pythonDev's uv/python3/Intel-XPU packages, env (LD_LIBRARY_PATH,
          # UV_PYTHON_PREFERENCE, ...) and its `venv()` shellHook helper.
          inputsFrom = pkgs.lib.optionals (pythonDev.devShells ? ${system}) [
            pythonDev.devShells.${system}.default
          ];
          # mkShell's inputsFrom merges buildInputs/shellHook but not `env`, so
          # forward pythonDev's env vars (Intel XPU libs, uv config) explicitly.
          env = pkgs.lib.optionalAttrs (pythonDev.devShells ? ${system}) (
            with pythonDev.devShells.${system}.default;
            {
              inherit
                LD_LIBRARY_PATH
                NEOReadDebugKeys
                OverrideGpuAddressSpace
                UV_PYTHON_PREFERENCE
                ;
            }
          );
          packages = [
            pkgs.colcon
            pkgs.git
            pkgs.nixgl.nixGLIntel
            # ... other non-ROS packages
            (
              with pkgs.rosPackages.noetic;
              buildEnv {
                #underlay = true;
                paths = [
                  ros-core
                  rosbag
                  rostopic
                  rviz
                  sensor-msgs
                  geometry-msgs
                  image-view
                  ouster-ros
                  rospy
                  faster-lio
                  robot-localization
                  imu-filter-madgwick
                  # ... other ROS packages
                ];
              }
            )
          ];
          shellHook = ''
                # Creates .venv via uv if it doesn't exist yet, then activates it.
                venv() {
                  [ -d .venv ] || uv venv
                  source .venv/bin/activate
                }
            		export QT_QPA_PLATFORM=xcb
            		export DISABLE_ROS1_EOL_WARNINGS=1
            		echo "========================================="
            		echo "🤖 ROS  Development Shell Loaded!"
            		echo "🚀 To run GUI tools, prefix them with: nixGLIntel"
            		echo "🖥️  QT_QPA_PLATFORM=xcb set (avoids rviz/OGRE crash on Wayland)"
            		echo "🐍 venv  # create/activate .venv via uv (then: uv pip install -r requirements.txt)"
            		echo "========================================="
            		'';
        };
      }
    );
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
