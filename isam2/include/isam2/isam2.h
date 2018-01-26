#ifndef ISAM2_INCLUDE_ISAM2_ISAM2_H_
#define ISAM2_INCLUDE_ISAM2_ISAM2_H_

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
#include "boost/foreach.hpp"
#include <fstream>
#include <iostream>

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <sensors_simulator/StampedCmd.h>
#include <sensors_simulator/StampedOptiTrackPoses.h>
#include <sensors_simulator/Range.h>

#include <isam2/StampedXYT.h>
#include <isam2/StampedCmd.h>
#include <isam2/Range.h>

/** PROJECT FILES*/

#include <armadillo>

// Both relative poses and recovered trajectory poses will be stored as Pose2 objects
#include <gtsam/geometry/Pose2.h>
// Range observations of the other agents will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>
// #include <gtsam/inference/Key.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// We want to use iSAM2 to solve the range-SLAM problem incrementally
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/RangeFactor.h> // GTSAM 3.2.1
// #include <gtsam/sam/RangeFactor.h> // GTSAM 4

// Setup the constants for easier conversions
#define _USE_MATH_DEFINES
#define DEG2RAD M_PI/180

using namespace gtsam;

class iSAM2
{
public:
	iSAM2(ros::NodeHandle nh, ros::NodeHandle nh_private);
	~iSAM2();

	// Counters
	int numCmdUno, numCmdDos, numOdomUno, numOdomDos, numRange, numTrue;
	int old_odom_counter_1, old_odom_counter_2;

	// Flags
	bool flag_odom1_initialized,flag_odom2_initialized, flag_use_isam;
	bool fresh_odom1, fresh_odom2;

	// Pose estimate publisher
	ros::Publisher  estimatePub;

	// Create a Factor Graph and Values to hold the new data
	NonlinearFactorGraph graph;
	Values initialEstimate;
	Values currentEstimate;
	Values rosOdomUno, rosOdomDos;
	Values trueUno, trueDos;
	Values odoUnoEstimates, odoDosEstimates;
	Values odomUnoValues, odomDosValues;
	Values rangeValues;

	// Define Noise Models for the various sensors (for iSAM2)
	noiseModel::Diagonal::shared_ptr priorNoise;
	noiseModel::Diagonal::shared_ptr cmdNoise;
	noiseModel::Diagonal::shared_ptr odomNoiseUno;
	noiseModel::Diagonal::shared_ptr odomNoiseDos;
	noiseModel::Isotropic::shared_ptr rangeNoise;

	// Define the odometry inputs as a Pose2
	Pose2 unoPose0, unoLastPose;
	Pose2 dosPose0, dosLastPose;
	Pose2 odometryUno, odometryDos;

  	// Storage of Dynamic model variables
	double old_x1, old_y1, old_yaw1, old_x2, old_y2, old_yaw2;
	double cur_x1, cur_y1, cur_yaw1, cur_x2, cur_y2, cur_yaw2;
	double sum_covX1, sum_covY1, sum_covYaw1, sum_covX2, sum_covY2, sum_covYaw2;

	// Storage for initial conditions
	double x1_init, y1_init, yaw1_init, x2_init, y2_init, yaw2_init;
	double range_init;

	// Performance Analysis: Sum-Squared Error (SSE) & Mean-Squared Error (MSE)
	double SSE_1, SSE_2, MSE_1, MSE_2;

	/**Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
     * and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
     * structure is available that allows the user to set various properties, such as the relinearization threshold
     * and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
     * will approach the batch result.*/
     ISAM2Params parameters;
     ISAM2 isam;

private:

	/* Ros specific member variables */
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nhPrivate;

	ros::Subscriber cmdUnoSub;
	ros::Subscriber cmdDosSub;
	ros::Subscriber odomUnoSub;
	ros::Subscriber odomDosSub;
	ros::Subscriber trueSub;
	ros::Subscriber rangeSub;

	void agent1CmdCallback(const sensors_simulator::StampedCmd::ConstPtr& msg);
	void agent2CmdCallback(const sensors_simulator::StampedCmd::ConstPtr& msg);
	void agent1OdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void agent2OdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void truthCallback(const sensors_simulator::StampedOptiTrackPoses::ConstPtr& msg);
	void rangeCallback(const sensors_simulator::Range::ConstPtr& msg);

	void updater(void);
};

#endif /* ISAM2_INCLUDE_ISAM2_ISAM2_H_ */
