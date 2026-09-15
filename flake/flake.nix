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
              # faster-lio's CHECK_EQ(int, int) usage trips a header-ordering bug in
              # glog >=0.6 under GCC's stricter two-phase lookup (see nix/faster-lio.nix).
              # Exposed here too so `ws/src/faster-lio` can be built from source in the
              # devShell against the same known-good glog version.
              glog_0_5 = prev.glog.overrideAttrs (old: rec {
                version = "0.5.0";
                src = prev.fetchFromGitHub {
                  owner = "google";
                  repo = "glog";
                  rev = "v${version}";
                  hash = "sha256-421K3q//HvvIybS5ZMRaQgjDW+zcDWp09DglVgQmAZw=";
                };
              });
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
            # catkin build tool for ws/ (per README) -- provides `catkin build`,
            # `catkin config`, etc. Distinct from the `catkin` ROS package below,
            # which supplies the CMake macros consumed by each package's CMakeLists.txt.
            pkgs.python3Packages.catkin-tools
            pkgs.cmake
            pkgs.pkg-config
            # Non-ROS build/runtime deps of ws/src/faster-lio and ws/src/ouster-ros
            # (see flake/nix/faster-lio.nix and flake/nix/ouster-ros.nix), needed here
            # because those packages are built from source via `catkin build` in ws/
            # instead of as prebuilt Nix derivations.
            pkgs.eigen
            pkgs.pcl
            pkgs.glog_0_5
            pkgs.yaml-cpp
            pkgs.tbb
            pkgs.opencv
            pkgs.curl
            pkgs.boost
            pkgs.jsoncpp
            pkgs.libtins
            pkgs.libzip
            pkgs.spdlog
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
                  rospy
                  robot-localization
                  imu-filter-madgwick
                  # ROS deps of ws/src/faster-lio and ws/src/ouster-ros, built from
                  # source via `catkin build` in ws/ (see README.md)
                  catkin
                  roscpp
                  std-msgs
                  nav-msgs
                  tf
                  tf2-ros
                  tf2-eigen
                  pcl-ros
                  pcl-conversions
                  cv-bridge
                  nodelet
                  message-generation
                  message-runtime
                  eigen-conversions
                  std-srvs
                  topic-tools
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
            		echo "🔧 cd ws && catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF"
            		echo "   (testing is disabled: ouster_ros vendors its own FindGTest.cmake that doesn't"
            		echo "   report GTEST_FOUND in the casing catkin's test macros expect, upstream issue)"
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
