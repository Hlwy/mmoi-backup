/**
		Currently only changes (x,y) Coordinates for models

		Future Work:
								1) Instead of explicitly changing multiple parameters, have one parameter contain all pose information
								2) Easier manipulation of yaw (remove necessity to know individual Quaternion information )

*/
#include <ros/ros.h>
#include <../include/path_following_turtle/path_following_turtle.h>
#include <path_following_turtle/resetParamsConfig.h>

using namespace std;

void resetCallback(PFT::resetParamsConfig& config, uint32_t level)
{

	/**Section:
								Update Internal Vars from Dynamic Reconfigure Server
	*/

	bool flag_reset = config.flag_reset_gazebo;

	string modelName1;
	string modelName2;

	double intern_x1 = config.initial_x1;
	double intern_x2 = config.initial_x2;

	double intern_y1 = config.initial_y1;
	double intern_y2 = config.initial_y2;

	/**Section:
								Execute Model Reset if User-Specified
	*/
	if(flag_reset) {

		/**Section:
									ROS Initializations
		*/

		ros::NodeHandle nh_reset;
		ros::ServiceClient client = nh_reset.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

		nh_reset.getParam("/bot1_name",modelName1);
		nh_reset.getParam("/bot2_name",modelName2);


		/**Section:
									Robot Reset Pose Definition
		*/

		//Robot 1 Position
		geometry_msgs::Point start_position;
		start_position.x = intern_x1;
		start_position.y = intern_y1;
		start_position.z = 0.0;
		//Robot 2 Position
		geometry_msgs::Point start_position2;
		start_position2.x = intern_x2;
		start_position2.y = intern_y2;
		start_position2.z = 0.0;

		//Robot 1 Orientation
		geometry_msgs::Quaternion start_orientation;
		start_orientation.x = 0;
		start_orientation.y = 0;
		start_orientation.z = 0;
		start_orientation.w = 1;
		//Robot 2 Orientation
		geometry_msgs::Quaternion start_orientation2;
		start_orientation2.x = 0.0;
		start_orientation2.y = 0.0;
		start_orientation2.z = 0.0;
		start_orientation2.w = 1.0;

		/**Section:
									Build Msg for Gazebo's "set_model_state" Client
		*/

		//Robot 1 ModelState: [model_name & Pose(Pose + Orientation)]
		geometry_msgs::Pose start_pose;
		gazebo_msgs::ModelState start_modelstate;

		start_pose.position = start_position;
		start_pose.orientation = start_orientation;
		start_modelstate.model_name = modelName1;
		start_modelstate.pose = start_pose;

		//Robot 2 ModelState: [model_name & Pose(Pose + Orientation)]
		geometry_msgs::Pose start_pose2;
		gazebo_msgs::ModelState start_modelstate2;

		start_pose2.position = start_position2;
		start_pose2.orientation = start_orientation2;
		start_modelstate2.model_name = modelName2;
		start_modelstate2.pose = start_pose2;

		/**Section:
									Send ModelState Msg to Gazebo for Reset Execution
		*/

		//Execute Reset for Robot 1
		gazebo_msgs::SetModelState srv;
		srv.request.model_state = start_modelstate;

		//Execute Reset for Robot 2
		gazebo_msgs::SetModelState srv2;
		srv2.request.model_state = start_modelstate2;

		/**Debug Section:
									Inform ROS System and Set flag_reset back to null
		*/

		if(client.call(srv))	{
		ROS_INFO("Agent Reset");
		}	else	{
		ROS_ERROR("Failed to reset robot Error msg:%s",srv.response.status_message.c_str());
		}

		if(client.call(srv2))	{
		ROS_INFO("Agent Reset");
		}	else	{
		ROS_ERROR("Failed to reset robot Error msg:%s",srv.response.status_message.c_str());
		}

	}

	ROS_INFO("Waiting");
}




int main(int argc, char** argv) {

	ros::init(argc, argv, "gazeboApp_resetModels");


	/**	Set Up the Dynamic Reconfigure Server
		@discription: Allows for easier manipulation of what robot to reset and where
	*/
	dynamic_reconfigure::Server<PFT::resetParamsConfig> server;
	dynamic_reconfigure::Server<PFT::resetParamsConfig>::CallbackType f;
	f = boost::bind(&resetCallback, _1, _2);
	server.setCallback(f);


	ros::spin();

	return 0;

}
