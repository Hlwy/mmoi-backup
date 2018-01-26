#include <ros/ros.h>
#include <../include/sensors_simulator/sensors_simulator.h>
#include <sensors_simulator/StampedXYT.h>
#include <sensors_simulator/Range.h>
#include <sensors_simulator/StampedOptiTrackPoses.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

// Global storage of each agent's pose information
double x1_in, y1_in, x2_in, y2_in;

// Store valuable information from ROS topics when topics are available
void optiCallback(const sensors_simulator::StampedOptiTrackPoses::ConstPtr& msg)
{

	// Collect OptiTrack pose info  and store it
	x1_in = msg->x1;
	y1_in = msg->y1;
	x2_in = msg->x2;
	y2_in = msg->y2;

}

int main(int argc, char** argv) {

	// Initialize the ROS Node
	ros::init(argc, argv, "opti_range_pubber");
	ros::NodeHandle n;
	// Initialize a publisher to make the range measurements available
	ros::Publisher range_pub = n.advertise<sensors_simulator::Range>("opti_range",1000);
	/** Setup ROS subscribers to listen to agent pose information
	*
	*		NOTE: Make sure that the topic name (Ex. 'bot1' or 'bot2') is the same as
	*  			  the name given to the agent in the OptiTrack environment.
	*/
  	ros::Subscriber sub1 = n.subscribe("opti_data", 1000, optiCallback);

	// Counter to count how many range measurements have been published to ROS
  	int count = 0;
  	// Output rate of our "simulated range sensor"
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		// Initialize ROS message to publish the range measurement
		sensors_simulator::Range range_out;

		// Calculate the range
		double dx = x2_in - x1_in;
		double dy = y2_in - y1_in;
		double range = sqrt(dx * dx + dy * dy);

		// Terminal output for debugging
	    	cout << "X1: " << x1_in << "    Y1: " << y1_in << endl;
		cout << "X2: " << x2_in << "    Y2: " << y2_in << endl;
		// cout << "Delta X: " << dx << "    Delta Y: " << dy << endl;
		cout << "Range: " << range << endl;
		cout << "------------------------------------------------------------------" << endl;

		// Load up the ROS msg for publishing
		range_out.seq = count;
		range_out.stamp = ros::Time::now();
    		range_out.frame_id = "OptiTrack";
		range_out.range = range;
		range_out.x1 = x1_in;
		range_out.y1 = y1_in;
		range_out.x2 = x2_in;
		range_out.y2 = y2_in;

		// Insert the ROS msg to the output queue
    		range_pub.publish(range_out);

		// Run ROS modules once
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
