cmake_minimum_required(VERSION 2.8.3)
project(slip_detection)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    rostest
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    tactile_msgs
    schunk_sdh
)

install(PROGRAMS
  scripts/intensity_extractor_node
  scripts/tactile_flow_extractor_node
  scripts/sdh_tactile_adapter_node
  scripts/slip_detector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
