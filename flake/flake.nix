{
  description = "ROS 1 Noetic Development Environment";

  inputs = {
    #nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/ros1-25.05";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
    nixgl.url = "github:nix-community/nixGL";
    pythonDev.url = "path:./nix/python";
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
                    # ros_numpy 0.0.5 (last release, 2019) uses the bare `np.float`
                    # alias, removed in NumPy 2.0. Patch it at the source instead of
                    # pinning an ancient numpy, since numpy is shared with kalibr's
                    # numpy_eigen elsewhere in this shell.
                    ros-numpy = rosPrev.ros-numpy.overrideAttrs (old: {
                      postPatch = (old.postPatch or "") + ''
                        substituteInPlace src/ros_numpy/point_cloud2.py \
                          --replace-fail "dtype=np.float)" "dtype=float)"
                      '';
                    });
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
              # Rebuild top-level pcl against the same Boost that rosPackages.noetic is
              # built against (boost186), instead of nixpkgs' default (currently 1.87).
              # nix-ros-overlay's pcl-ros/pcl-conversions pull PCL via this same `final`
              # fixpoint, so this keeps every Boost in the environment at one version --
              # otherwise CMAKE_PREFIX_PATH ends up with two Boost installs and
              # find_package(Boost COMPONENTS python...) (e.g. kalibr's numpy_eigen)
              # resolves against whichever version has no matching boost_python build.
              pcl = prev.pcl.override { boost = final.rosPackages.noetic.boost186; };
            })
          ];
          config = {
            permittedInsecurePackages = [
              "freeimage-3.18.0-unstable-2024-04-18"
            ];
          };
        };
        # Named so its wrapped bin/ (each binary carries its own
        # ROS_PACKAGE_PATH via makeWrapper) can be forced to the front of
        # PATH in shellHook -- otherwise the unwrapped bin/ dirs that ride
        # in transitively from these same ROS packages' propagated build
        # inputs land earlier on PATH, and running e.g. `roscore` picks
        # that unwrapped copy, which has no ROS_PACKAGE_PATH set and fails
        # with "Resource not found: roslaunch".
        rosEnv =
          with pkgs.rosPackages.noetic;
          buildEnv {
            #underlay = true;
            paths = [
              ros-core
              roslaunch
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
            # Use the same Boost that rosPackages.noetic is built against (rather than
            # top-level pkgs.boost, a different version) -- otherwise CMAKE_PREFIX_PATH
            # ends up with two Boost installs and find_package(Boost COMPONENTS python...)
            # (e.g. kalibr's numpy_eigen) resolves the main config against one version
            # but can only find the boost_python component built for the other.
            pkgs.rosPackages.noetic.boost186
            pkgs.jsoncpp
            pkgs.libtins
            pkgs.libzip
            # kalibr's ethz_apriltag2 demo binary (apriltags_demo.cpp) needs libv4l2.h
            pkgs.libv4l
            # kalibr's numpy_eigen needs numpy/arrayobject.h; it's put on CPATH below
            # since numpy_eigen's own CMakeLists.txt does no numpy detection of its own.
            pkgs.python3Packages.numpy
            # Runtime deps of kalibr's Python tools (kalibr_calibrate_cameras and
            # friends), per kalibr's own Dockerfile_ros1_20_04.
            pkgs.python3Packages.scipy
            pkgs.python3Packages.matplotlib
            pkgs.python3Packages.python-igraph
            pkgs.python3Packages.pyx
            pkgs.python3Packages.numdifftools
            pkgs.python3Packages.pillow
            pkgs.python3Packages.opencv4
            pkgs.python3Packages.wxPython_4_2
            pkgs.python3Packages.tkinter
            pkgs.python3Packages.pyyaml
            # jupyter-all is nixpkgs' merged python3.withPackages-style jupyter
            # environment (jupyterlab + notebook + ipykernel with a unified
            # sys.prefix, so share/jupyter/lab/schemas resolves correctly --
            # listing jupyter/notebook/ipykernel individually alongside it
            # instead leaves their own standalone `bin/jupyter` wrapper first
            # on PATH, whose sys.prefix is the bare python3 interpreter with
            # no merged schemas dir, causing "Missing or misshapen translation
            # settings schema" 404s). rospy is built against pkgs.python3
            # (3.12) too, and PYTHONPATH (set by the ROS buildEnv's setup hook
            # below) is inherited by jupyter's kernel subprocess regardless of
            # which python3 derivation it's running under, so `import rospy`
            # still works in notebooks.
            pkgs.jupyter-all
            # kalibr's aslam_optimizer/sparse_block_matrix needs SuiteSparse (+ BLAS/LAPACK).
            # CMake's FindBLAS/FindLAPACK need a Fortran compiler to verify symbol
            # mangling, hence gfortran. SuiteSparse links against BLAS/LAPACK internally
            # but doesn't expose them to downstream find_package(BLAS) calls, so list
            # them explicitly too.
            pkgs.suitesparse
            pkgs.gfortran
            pkgs.blas
            pkgs.lapack
            pkgs.spdlog
            # script/utils.py (used by script/pc_odom_rgb_sync_node.py, which needs
            # the fruit_counting message package under ws/src/fruit_counting) needs
            # ros_numpy for PointCloud2<->numpy conversions and h5py for its hdf5
            # tracklet helpers.
            pkgs.rosPackages.noetic.ros-numpy
            pkgs.python3Packages.h5py
            # ... other non-ROS packages
            rosEnv
          ];
          shellHook = ''
                # Creates .venv via uv if it doesn't exist yet, then activates it.
                venv() {
                  [ -d .venv ] || uv venv
                  source .venv/bin/activate
                }
                # rosEnv's own ROS packages (rospy, tf, ...) drag their unwrapped
                # bin/ dirs onto PATH ahead of rosEnv/bin via propagated build
                # inputs, so force rosEnv/bin (whose binaries are wrapped with a
                # working ROS_PACKAGE_PATH) back to the front.
                export PATH="${rosEnv}/bin:$PATH"
            		export QT_QPA_PLATFORM=xcb
            		export DISABLE_ROS1_EOL_WARNINGS=1
            		# kalibr's numpy_eigen #includes <numpy/arrayobject.h> directly with no
            		# CMake-side detection of its own.
            		export CPATH="${pkgs.python3Packages.numpy}/${pkgs.python3.sitePackages}/numpy/_core/include''${CPATH:+:}$CPATH"
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
