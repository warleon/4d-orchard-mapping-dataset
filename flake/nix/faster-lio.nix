{ lib
, buildRosPackage
, fetchFromGitHub
, catkin
, roscpp
, rospy
, std-msgs
, geometry-msgs
, nav-msgs
, sensor-msgs
, tf
, pcl-ros
, message-generation
, message-runtime
, eigen-conversions
, eigen
, pcl
, glog
, yaml-cpp
, tbb
, python3
}:

let
  # faster-lio's CHECK_EQ(int, int) usage trips a header-ordering bug in
  # glog >=0.6 (MakeCheckOpValueString moved into google::logging::internal;
  # unqualified lookup at the CHECK_EQ template's definition point can't see
  # it, since it's declared later in the header -- a real upstream glog bug
  # under GCC's stricter two-phase lookup). Build against 0.5.0, which
  # predates that restructuring, instead of patching glog's own headers.
  glog_0_5 = glog.overrideAttrs (old: rec {
    version = "0.5.0";
    src = fetchFromGitHub {
      owner = "google";
      repo = "glog";
      rev = "v${version}";
      hash = "sha256-421K3q//HvvIybS5ZMRaQgjDW+zcDWp09DglVgQmAZw=";
    };
  });
in
buildRosPackage rec {
  pname = "faster-lio";
  version = "unstable-2024-01-01";

  src = fetchFromGitHub {
    owner = "gaoxiang12";
    repo = "faster-lio";
    rev = "main";
    hash = "sha256-4b3l44Ekk+ajAG74lDPA2W4dmKGVE/tjLNotIX7FAak=";
  };

  # faster-lio writes its accumulated map to `ROOT_DIR/PCD/scans.pcd`, where
  # ROOT_DIR is a compile-time macro normally pointing at the catkin_ws
  # source checkout. Under Nix that would resolve into the read-only
  # /nix/store, so the final map write would silently fail. Redirect it to a
  # fixed writable location instead (create it before running the node).
  postPatch = ''
    sed -i 's#add_definitions(-DROOT_DIR=.*#add_definitions(-DROOT_DIR=\"/tmp/faster-lio-output/\")#' CMakeLists.txt

    # Upstream builds run_mapping_online/run_mapping_offline and
    # libfaster_lio.so but never installs any of them (their own workflow
    # relies on catkin's devel-space, not `make install`). buildRosPackage
    # does an install-space build, so add the missing install rules
    # ourselves.
    cat >> app/CMakeLists.txt <<'EOF'
install(TARGETS run_mapping_online run_mapping_offline
  RUNTIME DESTINATION ''${CATKIN_PACKAGE_BIN_DESTINATION}
)
EOF

    cat >> src/CMakeLists.txt <<'EOF'
install(TARGETS faster_lio
  ARCHIVE DESTINATION ''${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ''${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ''${CATKIN_PACKAGE_BIN_DESTINATION}
)
EOF
  '';

  buildType = "catkin";

  # run_mapping_online/offline live in lib/faster_lio/ but libfaster_lio.so
  # is installed one level up in lib/ -- catkin's install-space layout
  # doesn't get an RPATH covering that sibling dir by default.
  postFixup = ''
    for exe in $out/lib/faster_lio/run_mapping_online $out/lib/faster_lio/run_mapping_offline; do
      patchelf --add-rpath $out/lib "$exe"
    done
  '';

  propagatedBuildInputs = [
    roscpp
    rospy
    std-msgs
    geometry-msgs
    nav-msgs
    sensor-msgs
    tf
    pcl-ros
    message-generation
    message-runtime
    eigen-conversions
  ];

  buildInputs = [
    eigen
    pcl
    glog_0_5
    yaml-cpp
    tbb
    python3
  ];

  nativeBuildInputs = [ catkin ];

  meta = with lib; {
    description = "Faster-LIO: lightweight tightly-coupled lidar-inertial odometry";
    homepage = "https://github.com/gaoxiang12/faster-lio";
    license = licenses.bsd3;
  };
}
