cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)
find_package(catkin REQUIRED

  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files( DIRECTORY msg
  FILES
  ais_msg.msg
  gps_msg.msg
  measure.msg
  measurements.msg
  measurement_info.msg
  string_w_header.msg
  nav.msg
#  Yolomsg.msg
  wall_msg.msg
  info_detected_object.msg
  objects_detected.msg
  Fusionmsg.msg
  CanalPointList.msg
  radar_range.msg
  radar_spoke.msg
)

## Generate services in the 'srv' folder
# add_service_files( DIRECTORY srv
#   FILES
#   CameraImg.srv
#   MapGenPoint.srv
#   CameraCrop.srv
#   MobileNetCrop.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs)
