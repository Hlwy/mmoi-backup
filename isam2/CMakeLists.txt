cmake_minimum_required(VERSION 2.8.3)
project(isam2)

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
# Find Armadillo
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)

find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)

add_message_files(
  FILES
  Range.msg
  StampedXYT.msg
  StampedCmd.msg
)

generate_messages(
   DEPENDENCIES
   isam2
 )

catkin_package(
     INCLUDE_DIRS include
     LIBRARIES isam2
     CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
     DEPENDS Boost
)

include_directories(
	${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LAPACK_INCLUDE_DIRS} ${BLAS_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR}
)

# Set up the MAIN iSAM2 class as a library
add_library(libisam2 src/isam2.cpp)
target_link_libraries(libisam2 ${catkin_LIBRARIES})
# target_link_libraries(libisam2 ${catkin_LIBRARIES} ${GTSAM_LIBRARIES})
add_dependencies(libisam2 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

# Set up the AGENT 1 iSAM2 class library
add_library(libAgent1 src/agent1_isam.cpp)
target_link_libraries(libAgent1 ${catkin_LIBRARIES})
# target_link_libraries(libAgent1 ${catkin_LIBRARIES} ${GTSAM_LIBRARIES})
add_dependencies(libAgent1 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

## Publishes the Dead-Reckoned path estimates for both agents w/o range data
#add_executable(dead_reckon_sim src/dead_reckoning.cpp)
#arget_link_libraries(dead_reckon_sim ${catkin_LIBRARIES} gtsam)
#add_dependencies(dead_reckon_sim ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

## Centralized iSAM2 estimation of agents' pose history
add_executable(isam2_node src/isam2_node.cpp)
target_link_libraries(isam2_node libisam2 gtsam)
target_link_libraries(isam2_node ${ARMADILLO_LIBRARIES})
add_dependencies(isam2_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

## De-centralized iSAM2  AGENT 1
add_executable(agent1_isam2_node src/agent1_isam2_node.cpp)
target_link_libraries(agent1_isam2_node libAgent1 gtsam)
target_link_libraries(agent1_isam2_node ${ARMADILLO_LIBRARIES})
add_dependencies(agent1_isam2_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)
