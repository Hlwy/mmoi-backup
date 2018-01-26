cmake_minimum_required(VERSION 2.8.3)
project(sensors_simulator)

set( ROS_CXX_DEPENDENCIES
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  gazebo_msgs
  tf
  message_generation
  genmsg
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(Boost REQUIRED)

add_message_files(
  FILES
  Range.msg
  StampedXYT.msg
  StampedCmd.msg
  StampedOptiTrackPoses.msg
)

generate_messages(
   DEPENDENCIES
   sensors_simulator
 )

catkin_package(
	INCLUDE_DIRS vrpn include
  LIBRARIES sensors_simulator
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
  DEPENDS Boost
)

include_directories(vrpn)
include_directories(
	${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(ros_vrpn_client src/ros_vrpn_client.cpp
	vrpn/vrpn_Connection.C vrpn/vrpn_Tracker.C vrpn/vrpn_Shared.C vrpn/vrpn_FileConnection.C
	vrpn/vrpn_BaseClass.C vrpn/vrpn_Serial.C)
target_link_libraries(ros_vrpn_client ${catkin_LIBRARIES})

# Publish range received from Gazebo environment
add_executable(gazebo_range_sim src/gazebo_range_node.cpp)
target_link_libraries(gazebo_range_sim ${catkin_LIBRARIES})
add_dependencies(gazebo_range_sim ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

# Publish range received from OptiTrack environment
add_executable(opti_range_sim src/opti_range_node.cpp)
target_link_libraries(opti_range_sim ${catkin_LIBRARIES})
add_dependencies(opti_range_sim ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

# Publish range received from OptiTrack environment
add_executable(opti_pose_data src/opti_data_node.cpp)
target_link_libraries(opti_pose_data ${catkin_LIBRARIES})
add_dependencies(opti_pose_data ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

# Publish stamped control inputs for Agent 1
add_executable(agent1_controls_sim src/stamped_controls_agent1_node.cpp)
target_link_libraries(agent1_controls_sim ${catkin_LIBRARIES})
add_dependencies(agent1_controls_sim ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

# Publish stamped control inputs for Agent 2
add_executable(agent2_controls_sim src/stamped_controls_agent2_node.cpp)
target_link_libraries(agent2_controls_sim ${catkin_LIBRARIES})
add_dependencies(agent2_controls_sim ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)
