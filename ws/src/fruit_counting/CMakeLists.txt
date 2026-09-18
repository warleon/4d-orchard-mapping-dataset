cmake_minimum_required(VERSION 3.0.2)
project(fruit_counting)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  nav_msgs
)

# Makes src/fruit_counting/ importable as the `fruit_counting` python package
# (fruit_counting.utils), so scripts/ nodes can import it reliably. rosrun's
# nix-wrapped executables run scripts via compile()+exec() rather than a plain
# `python3 <path>` invocation, so they don't add the script's own directory to
# sys.path -- a bare sibling `import utils` isn't reliable there. Must come
# before generate_messages().
catkin_python_setup()

add_message_files(
  FILES
  SyncedPcOdomImg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_install_python(
  PROGRAMS
  scripts/camera_info_publisher.py
  scripts/colorize_pc_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
