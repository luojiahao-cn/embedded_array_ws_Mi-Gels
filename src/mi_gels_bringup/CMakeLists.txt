cmake_minimum_required(VERSION 3.0.2)
project(mi_gels_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  signal_generator
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    sensor_msgs
    signal_generator
    std_msgs
    std_srvs
)

catkin_install_python(PROGRAMS
  scripts/experiment_orchestrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
