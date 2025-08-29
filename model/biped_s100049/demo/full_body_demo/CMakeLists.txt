cmake_minimum_required(VERSION 3.0.2)
project(full_body_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/timed_action_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 
