cmake_minimum_required(VERSION 2.8.3)
project(path_following_turtle)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set( ROS_CXX_DEPENDENCIES
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  gazebo_msgs
  tf
  dynamic_reconfigure
  message_generation
  genmsg
)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
#find_package(PCL REQUIRED QUIET)  ## Originally included for "bot_pid_kobuki" Package)
find_package(Boost REQUIRED)
# Find Armadillo
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(Armadillo REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/resetParams.cfg
)

add_message_files(
  FILES
  StampedXYT.msg
)

generate_messages(
   DEPENDENCIES
   path_following_turtle
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES path_following_turtle
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
    DEPENDS Boost)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LAPACK_INCLUDE_DIRS} ${BLAS_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

# Resets Gazebo models to initial starting location in Gazebo
add_executable(gazeboApp_resetModels src/gazebo_model_reset_node.cpp)
target_link_libraries(gazeboApp_resetModels ${catkin_LIBRARIES})
add_dependencies(gazeboApp_resetModels ${PROJECT_NAME}_gencfg)

# Retrieves states of agent 1 in Gazebo environment
add_executable(agent1_state_server src/agent1_state_getter_node.cpp)
target_link_libraries(agent1_state_server ${catkin_LIBRARIES})
add_dependencies(agent1_state_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} path_following_turtle_generate_messages_cpp)


# Retrieves states of agent 2 in Gazebo environment
add_executable(agent2_state_server src/agent2_state_getter_node.cpp)
target_link_libraries(agent2_state_server ${catkin_LIBRARIES})
add_dependencies(agent2_state_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} path_following_turtle_generate_messages_cpp)

## Declare C++ libraries needed for
add_library(coopNav_path_gen src/path_generator_functions.cpp)
add_library(coopNav_path_follower src/pure_pursuit_functions.cpp)

## Link Libraries needed to run libraries
target_link_libraries(coopNav_path_gen ${catkin_LIBRARIES})
target_link_libraries(coopNav_path_follower ${catkin_LIBRARIES})
target_link_libraries(coopNav_path_gen ${ARMADILLO_LIBRARIES})
target_link_libraries(coopNav_path_follower ${ARMADILLO_LIBRARIES})

## Gazebo-Simulated
# Agent 1 Path Following Controller
add_executable(agent1_straight_line src/agent_1_straight_line_node.cpp)
target_link_libraries(agent1_straight_line ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
# Agent 2 Path Following Controller
add_executable(agent2_straight_line src/agent_2_straight_line_node.cpp)
target_link_libraries(agent2_straight_line ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
# Agent 1 Path Following Controller
add_executable(agent1_wave src/agent_1_wave_follower.cpp)
target_link_libraries(agent1_wave ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
# Agent 2 Path Following Controller
add_executable(agent2_wave src/agent_2_wave_follower.cpp)
target_link_libraries(agent2_wave ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)

## OptiTrack-Simulated
# Agent 1 Path Following Controller
add_executable(opti_agent1_straight_line src/opti_agent_1_straight_line_node.cpp)
target_link_libraries(opti_agent1_straight_line ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
# Agent 2 Path Following Controller
add_executable(opti_agent2_straight_line src/opti_agent_2_straight_line_node.cpp)
target_link_libraries(opti_agent2_straight_line ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
# Agent 1 Path Following Controller
add_executable(opti_agent1_wave src/opti_agent_1_wave_follower.cpp)
target_link_libraries(opti_agent1_wave ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
# Agent 2 Path Following Controller
add_executable(opti_agent2_wave src/opti_agent_2_wave_follower.cpp)
target_link_libraries(opti_agent2_wave ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${catkin_LIBRARIES} coopNav_path_gen coopNav_path_follower)
