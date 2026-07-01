cmake_minimum_required(VERSION 3.0.2)
project(mi_gels_motion)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  moveit_commander
  moveit_msgs
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    moveit_commander
    moveit_msgs
    rospy
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs
)

catkin_install_python(PROGRAMS
  scripts/motion_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
