cmake_minimum_required(VERSION 3.8)
project(env)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs

  rospy
  std_msgs
  tf2_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
  LIBRARIES test_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
 script/sensor.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
 script/USV_control.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

