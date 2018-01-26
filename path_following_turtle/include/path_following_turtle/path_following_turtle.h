#ifndef PATH_FOLLOWING_TURTLE_INCLUDE_PATH_FOLLOWING_TURTLE_PATH_FOLLOWING_TURTLE_H_
#define PATH_FOLLOWING_TURTLE_INCLUDE_PATH_FOLLOWING_TURTLE_PATH_FOLLOWING_TURTLE_H_

/** SYSTEM FILES*/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <string>
#include <cmath>
#include <ctime>
#include <boost/bind.hpp>
#include <boost/function.hpp>

/** ROS FILES*/
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

/** DYNAMIC RECONFIGURE FILES*/
#include <dynamic_reconfigure/server.h>

/** PROJECT FILES*/

#include <armadillo>
#include "../path_follower/sys_params.h"
#include "../path_follower/path_generator_functions.h"
#include "../path_follower/pure_pursuit_functions.h"

#define _USE_MATH_DEFINES

/** Uncomment to debug the area of interest */
// #define DEBUG_PATH_GEN_OUTPUT
// #define DEBUG_PATH_GEN_VARIABLES
// #define DEBUG_PATH_OUTPUT
// #define DEBUG_DISTANCE_FUNCTION
// #define DEBUG_IS_EQUAL_CHECK

#endif /* PATH_FOLLOWING_TURTLE_INCLUDE_PATH_FOLLOWING_TURTLE_PATH_FOLLOWING_TURTLE_H_ */
