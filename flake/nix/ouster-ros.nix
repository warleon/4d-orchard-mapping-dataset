{ lib
, buildRosPackage
, fetchFromGitHub
, catkin
, roscpp
, std-msgs
, sensor-msgs
, geometry-msgs
, std-srvs
, tf2-ros
, tf2-eigen
, pcl-conversions
, cv-bridge
, nodelet
, message-generation
, message-runtime
, topic-tools
, eigen
, pcl
, opencv
, curl
, boost
, jsoncpp
, libtins
, libzip
, spdlog
}:

buildRosPackage rec {
  pname = "ouster-ros";
  version = "0.14.0";

  src = fetchFromGitHub {
    owner = "ouster-lidar";
    repo = "ouster-ros";
    rev = "v${version}";
    fetchSubmodules = true; # pulls in the bundled ouster-sdk (ouster_client/ouster_pcap) submodule
    hash = "sha256-3NSutQXxHw858TF4O7yJryJd/9tjQ5+r0sOcTGfJ5PU=";
  };

  buildType = "catkin";

  propagatedBuildInputs = [
    roscpp
    std-msgs
    sensor-msgs
    geometry-msgs
    std-srvs
    tf2-ros
    tf2-eigen
    pcl-conversions
    cv-bridge
    nodelet
    message-generation
    message-runtime
    topic-tools
  ];

  buildInputs = [
    eigen
    pcl
    opencv
    curl
    boost
    jsoncpp
    libtins
    libzip
    spdlog
  ];

  nativeBuildInputs = [ catkin ];

  meta = with lib; {
    description = "ROS driver for Ouster lidar sensors";
    homepage = "https://github.com/ouster-lidar/ouster-ros";
    license = licenses.bsd3;
  };
}
