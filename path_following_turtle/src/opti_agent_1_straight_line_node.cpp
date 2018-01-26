#include <ros/ros.h>
#include "../include/path_following_turtle/path_following_turtle.h"

PathGen PP;
PurePursuit PURSUITER;

// argc: counter for command line arguments
// argv: array containing command line arguments
// THESE COULD POSSIBLY BE USED LATER
int main(int argc, char** argv) {

	// ===========================	VARIABLE INITIALIZATION ==============================
	// ROS HANDLES
	ros::init(argc, argv, "agent_1_node");
	ros::NodeHandle nh_agent1;
	std::string model_name;

	// GET MODEL NAME FROM PARAMETER SERVER
	if(nh_agent1.getParam("/bot1_name", model_name)) {}

	ros::Publisher pub_vel = nh_agent1.advertise<geometry_msgs::Twist>( model_name + "/mobile_base/commands/velocity", 5);

	// ROS MESSAGE TYPE HANDLES
	geometry_msgs::Twist move_base;

	// VARIABLES FOR STORAGE
	float delta, goalRadius, distanceToGoal;
	float dt = DT; double time_controls, time_start, time_end, timeAdd, time_old, time_update;
	fmat path, equidistOutputs, path2follow, robotGoal, controls_in, row_inserter, time_inserter;
	fmat curPose, updatedPose, currentLocation, updatedLocation, PoseControlHistory;
	fvec delta_vec;
	ucolvec ProjPointIdx;
	float x_cor, y_cor, yaw, x_cor_new, y_cor_new, yaw_new;

	bool flag_otherAgent_ready, flag_Iam_ready;
	// ===========================	======================= ==============================

	// Wait for a Non-Zero Sim Time to begin
	// bool time_flag = 0;
	// while(!time_flag) {
	// 	double secs =ros::Time::now().toSec();
	// 	if(secs <= 0){ time_flag = 0;}
	// 	else if(secs > 0) { time_flag = 1;}
	// }

	// Retrieve the states of the agent from OPTITRACK
	if(nh_agent1.getParam("/OptiTrack/Agent_1_Yaw", yaw)) {}
	if(nh_agent1.getParam("/OptiTrack/Agent_1_X_Coordinate", x_cor)) {}
	if(nh_agent1.getParam("/OptiTrack/Agent_1_Y_Coordinate", y_cor)) {}

	curPose << x_cor << y_cor << yaw << endr;

	//============== FOR LINE FOLLOWER =================
	path = PP.line_follower(x_cor,y_cor,0.467,4.776);
	delta = 0.01;
	path2follow = path;
	path2follow.save("VAR_Agent_1_Path-2-Follow.csv", csv_ascii);
	robotGoal = path2follow.tail_rows(1); // For Line following
	//============== =============== =================

	goalRadius = 0.1;

	currentLocation = curPose.head_cols(2);
	distanceToGoal = norm(currentLocation - robotGoal);

	controls_in << 0 << 0 << endr;

	// Wait until both agents are ready to begin
	if(nh_agent1.getParam("/Agent_2_Ready", flag_otherAgent_ready)) {}
	flag_Iam_ready = 1;
	nh_agent1.setParam("/Agent_1_Ready",flag_Iam_ready);
	while(!flag_otherAgent_ready){
		if(nh_agent1.getParam("/Agent_2_Ready", flag_otherAgent_ready)) {}
		cout << "Agent 2 Not Ready" << endl;
	}
	nh_agent1.setParam("/Agent_1_Ready",0);

  	time_start = ros::Time::now().toSec();
	time_inserter << time_start << endr;

	PoseControlHistory = join_rows(time_inserter, curPose);
	PoseControlHistory = join_rows(PoseControlHistory, controls_in);

	robotGoal.print("Agent 1's Goal: ");
	currentLocation.print("Agent 1 Current Location: ");
	cout << "distanceToGoal: " << distanceToGoal << endl;
	PoseControlHistory.print("Initial Insert");

	while(distanceToGoal > goalRadius) {

		//============== UPDATE POSE =================
		// Retrieve the states of the agent from OPTITRACK
		if(nh_agent1.getParam("OptiTrack/Agent_1_Yaw", yaw)) {}
		if(nh_agent1.getParam("OptiTrack/Agent_1_X_Coordinate", x_cor)) {}
		if(nh_agent1.getParam("OptiTrack/Agent_1_Y_Coordinate", y_cor)) {}

		curPose << x_cor << y_cor << yaw << endr;
		//============== =============== =============

		//============== COMPUTE CONTROL INPUTS =================
		controls_in = PURSUITER.step_controls(path2follow, curPose, delta, ProjPointIdx);

		float v = as_scalar(controls_in.col(0));
		float w = as_scalar(controls_in.col(1));
		//==============     ===============      ===============


		//============== DRIVE ROBOT =================
		move_base.linear.x = v; move_base.linear.y = 0; move_base.linear.z = 0;
		move_base.angular.x = 0; move_base.angular.y = 0; move_base.angular.z = w;
		pub_vel.publish(move_base);
		//============== =============== =================

		//========== CHECK THE CURRENT POSE ==========
		// Retrieve the states of the agent from OPTITRACK
		if(nh_agent1.getParam("OptiTrack/Agent_1_Yaw", yaw_new)) {}
		if(nh_agent1.getParam("OptiTrack/Agent_1_X_Coordinate", x_cor_new)) {}
		if(nh_agent1.getParam("OptiTrack/Agent_1_Y_Coordinate", y_cor_new)) {}

		timeAdd = ros::Time::now().toSec();

		updatedPose << x_cor_new << y_cor_new << yaw_new << endr;
		updatedLocation = updatedPose.head_cols(2);
		//============== =============== =================

		//============== STORE THE NEW DATA ==============
		timeAdd = timeAdd - time_start;

		time_inserter << timeAdd << endr;
		row_inserter = join_rows(time_inserter, curPose);
		row_inserter = join_rows(row_inserter, controls_in);
		PoseControlHistory = join_cols(PoseControlHistory, row_inserter);
		//============== =============== =================

		// RE-CHECK DISTANCE FOR LOOP EXIT
		distanceToGoal = norm(updatedLocation - robotGoal);

		// cout << "Distance to Goal: " << distanceToGoal << endl;

		if(!ros::ok()){
			break;
		}

	}

	ros::shutdown();

	// ROS_INFO("Agent 1 Data Saved");
	// OUTPUT POSE AND CONTROLS HISTORY TO EXCEL FILE
	// PoseControlHistory.save("VAR_Agent_1_History.csv", csv_ascii);

	return 0;

}
