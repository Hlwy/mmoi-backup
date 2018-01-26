#include <ros/ros.h>
#include <../include/sensors_simulator/sensors_simulator.h>
#include <sensors_simulator/StampedXYT.h>
#include <sensors_simulator/StampedOptiTrackPoses.h>
#include <geometry_msgs/TransformStamped.h>
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

// Global storage of each agent's pose information
double x1_out, y1_out, z1_out, yaw1_out;
double x2_out, y2_out, z2_out, yaw2_out;

// Store valuable information from ROS topics when topics are available
void agent1Callback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	double x_temp, y_temp, z_temp;
	double qx_temp, qy_temp, qz_temp, qw_temp;
	double yaw_temp;

	// Useful for if I want to republish for time keeping purposes //TODO: change how pointers are used to get values
	// target_state->target.header.frame_id = "OptiTrack";
	// target_state->target.child_frame_id = frame_id;
	// target_state->target.header.stamp = ros::Time::now();

	// Collect OptiTrack pose info  and store it
	x_temp = msg->transform.translation.x;
	y_temp = msg->transform.translation.y;
	z_temp = msg->transform.translation.z;

	qx_temp = msg->transform.rotation.x;
	qy_temp = msg->transform.rotation.y;
	qz_temp = msg->transform.rotation.z;
	qw_temp = msg->transform.rotation.w;

	// Calculate the yaw
	yaw_temp = atan2(2*qw_temp*qz_temp+2*qx_temp*qy_temp,1-2*qy_temp*qy_temp-2*qz_temp*qz_temp);

	// Store global important variables
	x1_out = x_temp;
	y1_out = y_temp;
	z1_out = z_temp;
	yaw1_out = yaw_temp;

}

void agent2Callback(const geometry_msgs::TransformStamped::ConstPtr& msg2)
{
	double x_temp2, y_temp2, z_temp2;
	double qx_temp2, qy_temp2, qz_temp2, qw_temp2;
	double yaw_temp2;


	// Useful for if I want to republish for time keeping purposes //TODO: change how pointers are used to get values
	// target_state->target.header.frame_id = "OptiTrack";
	// target_state->target.child_frame_id = frame_id;
	// target_state->target.header.stamp = ros::Time::now();

	// Collect OptiTrack pose info  and store it
	x_temp2 = msg2->transform.translation.x;
	y_temp2 = msg2->transform.translation.y;
	z_temp2 = msg2->transform.translation.z;

	qx_temp2 = msg2->transform.rotation.x;
	qy_temp2 = msg2->transform.rotation.y;
	qz_temp2 = msg2->transform.rotation.z;
	qw_temp2 = msg2->transform.rotation.w;

	// Calculate the yaw
	yaw_temp2 = atan2(2*qw_temp2*qz_temp2+2*qx_temp2*qy_temp2,1-2*qy_temp2*qy_temp2-2*qz_temp2*qz_temp2);

	// Store global important variables
	x2_out = x_temp2;
	y2_out = y_temp2;
	z2_out = z_temp2;
	yaw2_out = yaw_temp2;

}

int main(int argc, char** argv) {

	// Initialize the ROS Node
	ros::init(argc, argv, "optiTrack_pubber");
	ros::NodeHandle n;
	// Initialize a publisher to make the range measurements available
	ros::Publisher opti_pub = n.advertise<sensors_simulator::StampedOptiTrackPoses>("opti_data",1000);

	// Initialize ROS message to publish the optiTrack Data
	sensors_simulator::StampedOptiTrackPoses opti_out;

  	/** Setup ROS subscribers to listen to agent pose information
	*
	*		NOTE: Make sure that the topic name (Ex. 'bot1' or 'bot2') is the same as
	*  			  the name given to the agent in the OptiTrack environment.
	*/
	ros::Subscriber sub1 = n.subscribe("bot1/pose", 1000, agent1Callback);
	ros::Subscriber sub2 = n.subscribe("bot2/pose", 1000, agent2Callback);

	ros::Rate rate(500);

	// Counter to count how many range measurements have been published to ROS
  	int count = 0;
	double yaw1_deg, yaw2_deg;

	while (ros::ok()) {

		// Calculate the yaws in degrees for investigation
		yaw1_deg = yaw1_out * (180/M_PI);
		yaw2_deg = yaw2_out * (180/M_PI);

		// cout << "------------------------------------------------------------------" << endl;
		// Print out values for debugging
		cout << "X1: " << x1_out << "		Y1: " << y1_out << "    Z1: " << z1_out << "		YAW1: " << yaw1_deg << endl;
		cout << "X2: " << x2_out << "		Y2: " << y2_out << "    Z2: " << z2_out << "		YAW2: " << yaw2_deg << endl;
		// cout << "==========================================================" << endl;

		// Load up the ROS msg for publishing
		opti_out.seq = count;
		opti_out.stamp = ros::Time::now();
		opti_out.agent1_id = "Agent1";
		opti_out.agent2_id = "Agent2";
		opti_out.x1 = x1_out;
		opti_out.y1 = y1_out;
		opti_out.z1 = z1_out;
		opti_out.yaw1 = yaw1_out;
		opti_out.x2 = x2_out;
		opti_out.y2 = y2_out;
		opti_out.z2 = z2_out;
		opti_out.yaw2 = yaw2_out;

		// Insert the ROS msg to the output queue
    		opti_pub.publish(opti_out);

		n.setParam("OptiTrack/Agent_1_X_Coordinate", x1_out);
		n.setParam("OptiTrack/Agent_1_Y_Coordinate", y1_out);
		n.setParam("OptiTrack/Agent_1_Yaw", yaw1_out);

		n.setParam("OptiTrack/Agent_2_X_Coordinate", x2_out);
		n.setParam("OptiTrack/Agent_2_Y_Coordinate", y2_out);
		n.setParam("OptiTrack/Agent_2_Yaw", yaw2_out);

		// Run ROS modules once
		ros::spinOnce();
		rate.sleep();
		++count;
	}

	cout << "EXIT" << endl;
	ros::shutdown();
	return 0;
}
