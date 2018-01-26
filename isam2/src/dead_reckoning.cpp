#include <ros/ros.h>
#include "../include/isam2/isam2.h"

using namespace std;

string agent1_topic_name = "agent1/mobile_base/commands/velocity";
string agent2_topic_name = "agent2/mobile_base/commands/velocity";

// iSAM2::StampedXYT::ConstPtr estimated_xyt;
// iSAM2::Range::ConstPtr range_measured;
// iSAM2::StampedCmd::ConstPtr cmds;

float v_old, w_old;
float est_x = 0;
float est_y = 5;
float est_yaw = 0;

ros::Time temp_time;

void commandsCallback(const geometry_msgs::Twist::ConstPtr& msg1)
{
     float tv,tw;
     ros::Duration tdt;
     double dt;

     tv = msg1->linear.x;
     tw = msg1->angular.z;
     tdt = ros::Time::now() - temp_time;
     dt = tdt.toSec();
     // Assume dt = 0 if it is too high (i.e. assume that we just started)
     if(dt > 10){
          dt = 0;
     }

     est_x = est_x + cos(est_yaw) * dt * tv;
     est_y = est_y + sin(est_yaw) * dt * tv;
     est_yaw = est_yaw + dt * tw;

     temp_time = ros::Time::now();
     cout << "Dt: " << dt << "    Velocity: " << tv << "    Omega: " << tw;
     cout << "   X: " << est_x << "    Y: " << est_y << "    Theta: " << est_yaw << endl;
}

// void sensorCallback(const iSAM2::Range::ConstPtr& msg2)
// {
//   range_measured = msg2;
// }

int main(int argc, char** argv) {

	ros::init(argc, argv, "dead_reckoning_pubber");
	ros::NodeHandle n;
	ros::Publisher DR_pub = n.advertise<isam2::StampedXYT>("dead_reckoned_estimate",1000);
     ros::Subscriber sub1 = n.subscribe(agent1_topic_name, 1000, commandsCallback);
     // ros::Subscriber sub2 = n.subscribe("pose2_pubber", 1000, sensorCallback);

     isam2::StampedXYT pose_estimate;

	ros::Rate loop_rate(200);

	int count = 0;

	while (ros::ok()) {
          // If new control inputs available, store them internally and calculate new state (using Dead-Reckoning)

          pose_estimate.seq = count;
		pose_estimate.stamp = ros::Time::now();
		pose_estimate.x = est_x;
         pose_estimate.y = est_y;
         pose_estimate.yaw = est_yaw;

         DR_pub.publish(pose_estimate);

		ros::spinOnce();
          loop_rate.sleep();
		++count;
	}

	return 0;
}
