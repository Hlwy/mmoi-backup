#include <ros/ros.h>
#include <../include/path_following_turtle/path_following_turtle.h>
#include <path_following_turtle/StampedXYT.h>

using namespace std;

int main(int argc, char** argv) {

	string modelName;
	double time_current;

	ros::init(argc, argv, "agent2_state_server_node");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	ros::Publisher pose_pub = n.advertise<path_following_turtle::StampedXYT>("pose2_pubber",1000);

	while(!client.waitForExistence()){}

	ROS_INFO("Retrieving Agent 2's State... ");

	int count = 0;

	while(client.exists()) {
		n.getParam("/bot2_name",modelName);
		string relativeEntityName = "world";

		path_following_turtle::StampedXYT xyt;

		gazebo_msgs::GetModelState getModelState;
		getModelState.request.model_name = modelName;
		getModelState.request.relative_entity_name = relativeEntityName;
		client.call(getModelState);

		time_current = ros::Time::now().toSec();
		xyt.stamp = ros::Time::now();
		xyt.seq = count;

		geometry_msgs::Point pp;
		geometry_msgs::Quaternion qq;

		pp = getModelState.response.pose.position;
		qq = getModelState.response.pose.orientation;

		float xPos = pp.x;
		float yPos = pp.y;
		float x = qq.x;
		float y = qq.y;
		float z = qq.z;
		float w = qq.w;
		float yaw = atan2(2*x*y+2*z*w,1-2*y*y-2*z*z);

		xyt.agent_id = modelName;
		xyt.x = xPos;
		xyt.y = yPos;
		xyt.yaw = yaw;

		pose_pub.publish(xyt);

		n.setParam("GazeboInfo/Agent_2_X_Coordinate", xPos);
		n.setParam("GazeboInfo/Agent_2_Y_Coordinate", yPos);
		n.setParam("GazeboInfo/Agent_2_Yaw", yaw);
		n.setParam("GazeboInfo/Agent_2_Pose_Publish_Time", time_current);

		ROS_INFO("[Agent 2]:  Time: %3.2f    X: %3.2f		Y: %3.2f		Yaw: %3.2f\n", time_current, xPos, yPos, yaw);

		++count;
	}

	ros::spin();

	ROS_ERROR("Agent 2's State Retrieval Failed ");

	return 0;
}
