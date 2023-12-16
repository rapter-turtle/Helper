cmake_minimum_required(VERSION 3.8)
project(guidance)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf2_msgs
  geometry_msgs
  nav_msgs
  tf
  core_msgs
)

catkin_package()


#add_executable(waypoint_guidance_gps src/waypoint_guidance_BEFORE.cpp)
#add_dependencies(waypoint_guidance sensor_msgs_generate_messages_cpp)

#add_executable(waypoint_guidance_lidar src/waypoint_guidance_lidar.cpp)
#add_dependencies(waypoint_guidance sensor_msgs_generate_messages_cpp) 

add_executable(waypoint_guidance_col_radar src/waypoint_guidance_col_ver2.cpp)
add_executable(waypoint_guidance_col_lidar src/waypoint_guidance_col_ver3.cpp)
add_executable(waypoint_guidance_col_lidar_ver2 src/waypoint_guidance_col_ver4.cpp)
add_executable(waypoint_guidance_col_lidar_ver3 src/waypoint_guidance_col_ver5.cpp)
add_executable(waypoint_guidance_col_gps src/waypoint_guidance_col.cpp)
add_executable(waypoint_guidance_col_gps_pod src/waypoint_guidance_col_pod.cpp)
add_executable(Cal_vel_node src/Cal_Nominal_vel.cpp)

add_dependencies(waypoint_guidance_col_radar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_guidance_col_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_guidance_col_lidar_ver2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_guidance_col_lidar_ver3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_guidance_col_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Cal_vel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypoint_guidance_col_gps_pod ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 catkin_package(
#  LIBRARIES guidance
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy core_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#target_link_libraries(waypoint_guidance_gps
  #${catkin_LIBRARIES}
  #${GeographicLib_LIBRARIES}
#)
#target_link_libraries(waypoint_guidance_lidar
  #${catkin_LIBRARIES}
  #${GeographicLib_LIBRARIES}
#)
target_link_libraries(waypoint_guidance_col_radar
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)
target_link_libraries(waypoint_guidance_col_lidar
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

target_link_libraries(waypoint_guidance_col_lidar_ver2
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

target_link_libraries(waypoint_guidance_col_lidar_ver3
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)



target_link_libraries(waypoint_guidance_col_gps
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

target_link_libraries(Cal_vel_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

target_link_libraries(waypoint_guidance_col_gps_pod
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)
