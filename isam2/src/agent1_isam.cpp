#include "../include/isam2/isam2.h"

using namespace std;
using namespace gtsam;
using namespace arma;
namespace NM = gtsam::noiseModel;

iSAM2::iSAM2(ros::NodeHandle nh, ros::NodeHandle nh_private)
		: m_nh(nh), m_nhPrivate(nh_private)
{
	std::string whoAmI;
	std::string cmdUno_topic_name = "bot1_stamped_cmds";
	std::string cmdDos_topic_name = "bot2_stamped_cmds";
	std::string odomUno_topic_name = "agent1/robot_pose_ekf/odom_combined";
	std::string odomDos_topic_name = "agent2/robot_pose_ekf/odom_combined";
	std::string truth_topic_name = "opti_data";
	std::string range_topic_name = "opti_range";

	/** Define our iSAM2 parameters */
     parameters.relinearizeThreshold = 0.01;
     parameters.relinearizeSkip = 1;
 	ISAM2 isam(parameters);

	// Counters
	numCmdUno = 0; 	// Number of collected velocity commands for Agent 1
	numCmdDos = 0;		// Number of collected velocity commands for Agent 2
	numOdomUno = 0;	// Number of Odometry signals received for Agent 1
	numOdomDos = 0; 	// Number of Odometry signals received for Agent 2
	numRange = 0; 		// Number of collected Range measurements
	numTrue = 0;		// Number of Ground Truth Measurements received

	// Global odom counters to be used for connecting range measurements to agent poses
	old_odom_counter_1 = 0;
	old_odom_counter_2 = 0;

	/** Initial Poses: OptiTrack Waves */
	x1_init = 1.709; 		x2_init = -1.3073;
	y1_init = -2.1478; 		y2_init = -2.7493;
	yaw1_init = 1.831532;	yaw2_init = 1.764686;
	range_init = 3.075647;

	// /** Initial Poses: OptiTrack Straight Lines */
	// x1_init = 1.759;		x2_init = -1.282;
	// y1_init = -2.153;		y2_init = -2.746;
	// yaw1_init = 1.781318;	yaw2_init = 1.777531;
	// range_init = 3.097;

	// /** Initial Poses: Gazebo Simulations */
	// x1_init = 0;		x2_init = 0;
	// y1_init = 5;		y2_init = -5;
	// yaw1_init = 0;		yaw2_init = 0;
	// range_init = 10;

	/** Subscribers to topics that contain relevent data */
	// cmdUnoSub = m_nh.subscribe(cmdUno_topic_name, 250, &iSAM2::agent1CmdCallback, this);
	// cmdDosSub = m_nh.subscribe(cmdDos_topic_name, 250, &iSAM2::agent2CmdCallback, this);
	odomUnoSub = m_nh.subscribe(odomUno_topic_name, 250, &iSAM2::agent1OdomCallback, this);
	odomDosSub = m_nh.subscribe(odomDos_topic_name, 250, &iSAM2::agent2OdomCallback, this);
	trueSub = m_nh.subscribe(truth_topic_name, 1000, &iSAM2::truthCallback, this);
	rangeSub = m_nh.subscribe(range_topic_name, 1000, &iSAM2::rangeCallback, this);

	// Publisher for the online iSAM2 estimated pose
	estimatePub = m_nh.advertise<isam2::StampedXYT>("pose_estimate",1000);

	// Set Noise parameters
	double priorSig = 0.0001;	// Std dev of priors (meters)
	double sigmaR = 0.01; 		// Std dev of range measurements (meters)
	// Initialize noise models to be used by GTSAM libraries
	Vector priorSigmas = Vector3(priorSig,priorSig,priorSig);
	priorNoise = NM::Diagonal::Sigmas(priorSigmas); //prior
	rangeNoise = NM::Isotropic::Sigma(1, sigmaR); // non-robust

	// Store Initial Poses to a dynamic global variable
	old_x1 = x1_init;		old_x2 = x2_init;
	old_y1 = y1_init;		old_y2 = y2_init;
	old_yaw1 = yaw1_init;	old_yaw2 = yaw2_init;

	cur_x2 = x2_init; cur_y2 = y2_init; cur_yaw2 = yaw2_init;

	// Add prior poses to the factor graph
	unoPose0 = Pose2(old_x1, old_y1, old_yaw1);
	dosPose0 = Pose2(old_x2, old_y2, old_yaw2);
	graph.push_back(PriorFactor<Pose2>(Symbol('A', 0), unoPose0, priorNoise));
	graph.push_back(PriorFactor<Pose2>(Symbol('B', 0), dosPose0, priorNoise));
	// Add initial known range
	graph.push_back(RangeFactor<Pose2, Pose2>(Symbol('A', 0), Symbol('B', 0), range_init, rangeNoise));

	// Store Initial poses into the 'Values' containers for later analysis
	initialEstimate.insert(Symbol('A', 0), unoPose0);
	initialEstimate.insert(symbol('B', 0), dosPose0);
	// Print out stored variables for debugging
	initialEstimate.print("Initial Pose: ");

	// Update the last known pose for future calculations
	unoLastPose = unoPose0;
	dosLastPose = dosPose0;

	// Setup 'initialization' flags to ensure factor graph is loaded with proper keys
	flag_odom1_initialized = false;
    	flag_odom2_initialized = false;
	flag_use_isam = true;
	fresh_odom1 = false;
	fresh_odom2 = false;

	// Performance Analysis Values
	SSE_1 = 0; SSE_2 = 0; MSE_1 = 0; MSE_2 = 0;

	// Initialize Covariance summers
	sum_covX2 = 0; sum_covY2 = 0; sum_covYaw2 = 0;

	// Key keyd = Key(4683743612465315840);
	// Symbol key_debug = Symbol(keyd);
	// key_debug.print("Debugging Key: ");
}

iSAM2::~iSAM2()
{
}


void iSAM2::agent1OdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	int temp_counter;
	double temp_x, temp_y, qx, qy, qz, qw, yaw;
	double x, y, dist_traveled;
	double covX, covY, covYaw;
	double dx, dy, dyaw;
	Pose2 newPose;
	/** NOTE:
		Odometry is given as the dead-reckoned path of Turtlebot w.r.t its
	starting position (where it is when its turned on) the following variables
	are used in order to rotate the dead-reckoned path into the OptiTrack
	frame (angle of rotation was found experimentally)
	*/
	fmat R, rawOdom1, rotOdom1;
	fcolvec odom_x, odom_y;
	double rotDeg, rotTheta;

	// Angle needed to rotate into the OptiTrack reference frame
	rotDeg = 103.5; // in degrees
	rotTheta = rotDeg * DEG2RAD;

	// Update Global counter
	numOdomUno = numOdomUno + 1;

	// Load up our incoming data locally
	temp_counter = msg->header.seq;

	temp_x = msg->pose.pose.position.x;
	temp_y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;

	covX = msg->pose.covariance[0];
	covY = msg->pose.covariance[7];
	covYaw = msg->pose.covariance[35];

	// Setup the noise model
	Vector odoCov = Vector3(covX,covY,covYaw);
	odomNoiseUno = NM::Diagonal::Variances(odoCov);

	// Calculate yaw from quaternions
	yaw = atan2(2*qw*qz+2*qx*qy,1-2*qy*qy-2*qz*qz);

	// Rotate x/y coordinates into OptiTrack Frame
	R << cos(rotTheta) << -sin(rotTheta) << endr
	  << sin(rotTheta) << cos(rotTheta) << endr;

	rawOdom1 << temp_x << endr << temp_y << endr;
  	rotOdom1 = R * rawOdom1;

	// Extract data from matrix into usable form
	rotOdom1 = rotOdom1.t();
	odom_x = rotOdom1.col(0);
	odom_y = rotOdom1.col(1);

	// Put Rotated Odom data into global frame
    	x = x1_init + odom_x(0);
    	y = y1_init + odom_y(0);
    	yaw = yaw1_init + yaw;

	// Calculate change in pose since our last recorded odometry pose
	dx = x - old_x1;
	dy = y - old_y1;
	dyaw = yaw - old_yaw1;
	dist_traveled = sqrt(dx*dx + dy*dy);

	// BetweenFactor was found to work when only using the change in pose relative to the body frame
	odometryUno = Pose2(dist_traveled, 0, dyaw);

	// Store important variables for later analysis
	// TODO: re-organization of what we want to use for later analysis
	newPose = Pose2(x, y, yaw);
	rosOdomUno.insert(temp_counter, newPose);
	odomUnoValues.insert(temp_counter, odometryUno);

	// Ensure we load the BetweenFactor in the factor graph with proper linking of keys
    	if(flag_odom1_initialized) {
        	graph.push_back(BetweenFactor<Pose2>(Symbol('A',old_odom_counter_1), Symbol('A',temp_counter), odometryUno, odomNoiseUno));
	} else {
        	graph.push_back(BetweenFactor<Pose2>(Symbol('A',0), Symbol('A',temp_counter), odometryUno, odomNoiseUno));
	}

	/** NOTE:
		Using simulated control inputs from the changes in the estimated pose
	recieved from the dead-reckoned path easily obtained from Turtlebot,
	predict our new pose estimate from our last known pose
	*/
    	Pose2 predPose1 = unoLastPose.compose(odometryUno);

	// Add this predicted pose to our initial estimate
	initialEstimate.insert(Symbol('A', temp_counter), predPose1);

	/** Print out any debug values */
	// cout << "[AGENT 1] Covariance: " << covX << "	" << covY << " 	" << covYaw << endl;

	// Update Global variables
	old_x1 = x;
	old_y1 = y;
	old_yaw1 = yaw;
	unoLastPose = predPose1;

	old_odom_counter_1 = temp_counter;
	flag_odom1_initialized = true;
	fresh_odom1 = true;
}


void iSAM2::agent2OdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	int temp_counter;
	double temp_x, temp_y, qx, qy, qz, qw, yaw;
	double covX, covY, covYaw;
	/** NOTE:
		Odometry is given as the dead-reckoned path of Turtlebot w.r.t its
	starting position (where it is when its turned on) the following variables
	are used in order to rotate the dead-reckoned path into the OptiTrack
	frame (angle of rotation was found experimentally)
	*/
	fmat R, rawOdom2, rotOdom2;
	fcolvec odom_x, odom_y;
	double rotDeg, rotTheta;

	// Angle needed to rotate into the OptiTrack reference frame
	rotDeg = 103.5; // in degrees
	rotTheta = rotDeg * DEG2RAD;

	// Load up our incoming data locally
	temp_counter = msg->header.seq;

	temp_x = msg->pose.pose.position.x;
	temp_y = msg->pose.pose.position.y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;

	covX = msg->pose.covariance[0];
	covY = msg->pose.covariance[7];
	covYaw = msg->pose.covariance[35];

	// Calculate yaw from quaternions
	yaw = atan2(2*qw*qz+2*qx*qy,1-2*qy*qy-2*qz*qz);

	// Rotate x/y coordinates into OptiTrack Frame
	R << cos(rotTheta) << -sin(rotTheta) << endr
	  << sin(rotTheta) << cos(rotTheta) << endr;

	rawOdom2 << temp_x << endr << temp_y << endr;
  	rotOdom2 = R * rawOdom2;

	// Extract data from matrix into usable form
	rotOdom2 = rotOdom2.t();
	odom_x = rotOdom2.col(0);
	odom_y = rotOdom2.col(1);

	// Sum up the covariance since agent 2's last known position
	sum_covX2 = sum_covX2 + covX;
	sum_covY2 = sum_covY2 + covY;
	sum_covYaw2 = sum_covYaw2 + covYaw;

	// Put Rotated Odom data into global frame AND ...
	// Update currently known estimate to be used for when data is exchanged
    	cur_x2 = x2_init + odom_x(0);
    	cur_y2 = y2_init + odom_y(0);
    	cur_yaw2 = yaw2_init + yaw;

	// Debugging variable
	Pose2 curPose = Pose2(cur_x2,cur_y2,cur_yaw2);
	// curPose.print("AGENT 2 Current Pose: ");

	// Update Global flags
	fresh_odom2 = true;
	flag_odom2_initialized = true;
}


void iSAM2::rangeCallback(const sensors_simulator::Range::ConstPtr& msg)
{
	double range_in, true_x1, true_y1, true_x2, true_y2;
	double xhat1, yhat1;
	double error1, e_dx1, e_dy1;
	double dx2,dy2,dyaw2, dist_traveled;

#ifdef TIMINGS
	tictoc_print_();
#endif
	// Update Global counter
  	numRange = numRange + 1;

	// Load up incoming data
	range_in = msg->range;
	true_x1 = msg->x1;
	true_y1 = msg->y1;
	true_x2 = msg->x2;
	true_y2 = msg->y2;

	// cout << fresh_odom1 << fresh_odom2 << endl;

	if(flag_use_isam && fresh_odom1 && fresh_odom2){

		// ======================================
		//	BEGIN OTHER AGENT ODOM TRANSFER
		// ======================================
		gttic_(add_odom_2);
		numOdomDos = numOdomDos + 1;
		// Calculate change in pose since our last recorded odometry pose
		dx2 = cur_x2 - old_x2;
		dy2 = cur_y2 - old_y2;
		dyaw2 = cur_yaw2 - old_yaw2;
		dist_traveled = sqrt(dx2*dx2 + dy2*dy2);

		// Setup the noise model
		Vector odoCov2 = Vector3(sum_covX2,sum_covY2,sum_covYaw2);
		odomNoiseDos = NM::Diagonal::Variances(odoCov2);

		// BetweenFactor was found to work when only using the change in pose relative to the body frame
		odometryDos = Pose2(dist_traveled, 0, dyaw2);

		// odometryDos.print("AGENT 2 Delta: ");
		// cout << "AGENT 2 Covariance: " << odoCov2 << endl;

		// Store important variables for later analysis
		// TODO: re-organization of what we want to use for later analysis
		Pose2 newPose = Pose2(cur_x2, cur_y2, cur_yaw2);
		rosOdomDos.insert(numOdomDos, newPose);
		odomDosValues.insert(numOdomDos, odometryDos);

		// Load agent 2's between factor into the factor graph
	     graph.push_back(BetweenFactor<Pose2>(Symbol('B',numOdomDos - 1), Symbol('B',numOdomDos), odometryDos, odomNoiseDos));

		/** NOTE:
			Using simulated control inputs from the changes in the estimated pose
		recieved from the dead-reckoned path easily obtained from Turtlebot,
		predict our new pose estimate from our last known pose
		*/
	    	Pose2 predPose2 = dosLastPose.compose(odometryDos);
		// predPose2.print("PREDICTED POSE: ");

		// Add this predicted pose to our initial estimate
		initialEstimate.insert(Symbol('B', numOdomDos), predPose2);

		// Update Global variables
		old_x2 = cur_x2;
		old_y2 = cur_y2;
		old_yaw2 = cur_yaw2;
		dosLastPose = predPose2;

		// Reset agent 2 covariances
		sum_covX2 = 0;
		sum_covY2 = 0;
		sum_covYaw2 = 0;

		gttoc_(add_odom_2);
		// =============================================
		gttic_(adding_range);
		// Add Range factor to the factor graph
	  	graph.push_back(RangeFactor<Pose2, Pose2>(Symbol('A', old_odom_counter_1), Symbol('B', numOdomDos), range_in, rangeNoise));
		gttoc_(adding_range);

		// ======================================
		//	     BEGIN iSAM2 ESTIMATION
		// ======================================

		// If we are using iSAM2 then update our state estimate whenever we are sending data between
		gttic_(isam_update);
		// Perform one iSAM2 update step and get the current estimated poses
		isam.update(graph, initialEstimate);
		gttoc_(isam_update);
		// initialEstimate.print("Initial Estimate: ");
		// graph.print("Factor Graph: ");
		gttic_(calculate_estimate);
		currentEstimate = isam.calculateEstimate();
		// cout << "GOT HERE!!!!!" << endl;
		gttoc_(calculate_estimate);
		// Print out estimate for debug
		// currentEstimate.print("Current Estimate");

		// Retrieve most recent estimate of each agent
		Pose2 pose1 = currentEstimate.at<Pose2>(Symbol('A', old_odom_counter_1));
		xhat1 = pose1.x();
		yhat1 = pose1.y();

		// Calculate the estimate error compared to truth
		e_dx1 = true_x1 - xhat1;	e_dy1 = true_y1 - yhat1;
		error1 = sqrt(e_dx1*e_dx1 + e_dy1*e_dy1);

		SSE_1 = SSE_1 + error1*error1;

		pose1.print("Agent 1 Estimate: ");
		// cout << "Agent 1 Error: " << error1 << endl;
		cout << "=============" << endl;

		// Reset factor graph and initial estimate values for next update step
		graph.resize(0);
		initialEstimate.clear();

		fresh_odom1 = false;
		fresh_odom2 = false;

	}

}

void iSAM2::updater(void){
	// gttic_(update);
	// // Update iSAM with the new factors
	// isam.update(graph, initialEstimate);
	// // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
	// // If accuracy is desired at the expense of time, update(*) can be called additional times
	// // to perform multiple optimizer iterations every step.
	// isam.update();
	// gttoc_(update);
	// gttic_(calculateEstimate);
	// currentEstimate = isam.calculateEstimate();
	// gttoc_(calculateEstimate);
	// cout << "****************************************************" << endl;
	// currentEstimate.print("Current estimate: ");
	//
	// // Print timings
	// tictoc_print_();
	//
	// // Clear the factor graph and values for the next iteration
	// graph.resize(0);
	// initialEstimate.clear();

}


void iSAM2::agent1CmdCallback(const sensors_simulator::StampedCmd::ConstPtr& msg){
	double tv, tw, dx, dy, dt;
	double delta_dist, delta_yaw;

	// Update Global counter
	numCmdUno = numCmdUno + 1;
	// cout << "[ODOM]: # N: " << numN << "		# M: " << numM << endl;

	// Load up our incoming data locally
	tv = msg->v;
	tw = msg->w;
	dt = msg->dt;

	// Calculate change in pose (FROM DYNAMICAL MODEL) since our last received control inputs
	dx = cos(old_yaw1) * dt * tv;
	dy = sin(old_yaw1) * dt * tv;
	delta_dist = sqrt(dx * dx + dy * dy);
	delta_yaw = dt * tw;

	old_x1 = old_x1 + dx;
	old_y1 = old_y1 + dy;
	old_yaw1 = old_yaw1 + delta_yaw;

	// cout << "Temp Time: " << temp_time << "		Dt: " << dt << "		Vel: " << tv << "		Ang. Rate: " << tw << endl;
	// cout << "Dt: " << dt << "		Vel: " << tv << "		Ang. Rate: " << tw << endl;
	// cout << "X: " << old_x << "		Y: " << old_y << endl;

	// Predict next pose based on how far we think we went and add as initial estimate
	odometryUno = Pose2(dx, dy, delta_yaw);
	Pose2 predictedPose = unoLastPose.compose(odometryUno);
	odoUnoEstimates.insert(numCmdUno, predictedPose);

	// Update the last known pose as our most recent dead-reckoned estimate
	unoLastPose = predictedPose;

	// Add odometry factor to the 'Values' of our initial pose estimate and the factor graph
	initialEstimate.insert(Symbol('A', numCmdUno), predictedPose);
	graph.push_back(BetweenFactor<Pose2>(Symbol('A', numCmdUno-1), Symbol('A', numCmdUno), odometryUno, cmdNoise));
}

void iSAM2::agent2CmdCallback(const sensors_simulator::StampedCmd::ConstPtr& msg) {
	double tv, tw, dx, dy, dt;
	double delta_dist, delta_yaw;

	// Update Global counter
	numCmdDos = numCmdDos + 1;
	// cout << "[ODOM]: # N: " << numN << "		# M: " << numM << endl;

	// Load up our incoming data locally
	tv = msg->v;
	tw = msg->w;
	dt = msg->dt;

	// Calculate change in pose (FROM DYNAMICAL MODEL) since our last received control inputs
	dx = cos(old_yaw2) * dt * tv;
	dy = sin(old_yaw2) * dt * tv;
	delta_dist = sqrt(dx * dx + dy * dy);
	delta_yaw = dt * tw;

	old_x2 = old_x2 + dx;
	old_y2 = old_y2 + dy;
	old_yaw2 = old_yaw2 + delta_yaw;

	// cout << "Temp Time: " << temp_time << "		Dt: " << dt << "		Vel: " << tv << "		Ang. Rate: " << tw << endl;
	// cout << "Dt: " << dt << "		Vel: " << tv << "		Ang. Rate: " << tw << endl;
	// cout << "X: " << old_x << "		Y: " << old_y << endl;

	// Predict next pose based on how far we think we went and add as initial estimate
	odometryDos = Pose2(dx, dy, delta_yaw);
	Pose2 predictedPose = dosLastPose.compose(odometryDos);
	odoDosEstimates.insert(numCmdDos, predictedPose);

	// Update the last known pose as our most recent dead-reckoned estimate
	dosLastPose = predictedPose;

	// Add odometry factor to the 'Values' of our initial pose estimate and the factor graph
	initialEstimate.insert(Symbol('B', numCmdDos), predictedPose);
	graph.push_back(BetweenFactor<Pose2>(Symbol('B', numCmdDos-1), Symbol('B', numCmdDos), odometryDos, cmdNoise));
}



void iSAM2::truthCallback(const sensors_simulator::StampedOptiTrackPoses::ConstPtr& msg) {
	double x1, y1, yaw1, x2, y2, yaw2;
	Pose2 truePoseUno, truePoseDos;

	// Update Global counter
	numTrue = numTrue + 1;

	// Load up msg data
	x1 = msg->x1;			x2 = msg->x2;
	y1 = msg->y1;			y2 = msg->y2;
	yaw1 = msg->yaw1;		yaw2 = msg->yaw2;

	// Load up the Pose info
	truePoseUno = Pose2(x1, y1, yaw1);
	truePoseDos = Pose2(x2, y2, yaw2);

	// Store Ground Truth Values for later analysis
	trueUno.insert(numTrue, truePoseUno);
	trueDos.insert(numTrue, truePoseDos);

}
