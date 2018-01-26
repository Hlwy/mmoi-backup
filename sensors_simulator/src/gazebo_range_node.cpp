#include <ros/ros.h>
#include <../include/sensors_simulator/sensors_simulator.h>
#include <sensors_simulator/StampedXYT.h>
#include <sensors_simulator/Range.h>

using namespace std;

sensors_simulator::StampedXYT::ConstPtr xy1;
sensors_simulator::StampedXYT::ConstPtr xy2;

/** Callback functions that store each agent's pose information when there is
data available on the corresponding ROS topic */
void agent1Callback(const sensors_simulator::StampedXYT::ConstPtr& msg1){
  xy1 = msg1;}

void agent2Callback(const sensors_simulator::StampedXYT::ConstPtr& msg2){
  xy2 = msg2;}


int main(int argc, char** argv) {

  // Initialize the ROS Node
	ros::init(argc, argv, "gazebo_range_pubber");
	ros::NodeHandle n;
  // Setup a ROS publisher to make the range measurement available
	ros::Publisher range_pub = n.advertise<sensors_simulator::Range>("gazebo_range",1000);

  /** Setup the ROS subscribers to take in each agent's pose information when it
  is ready */
  ros::Subscriber sub1 = n.subscribe("pose1_pubber", 1000, agent1Callback);
  ros::Subscriber sub2 = n.subscribe("pose2_pubber", 1000, agent2Callback);

  // Initialize variable storage for range calculations
  double xPos1, yPos1, xPos2, yPos2;
  // Counter to count how many range measurements have been published to ROS
  int count = 0;
  // Output rate of our "simulated range sensor"
	ros::Rate loop_rate(20);

	while (ros::ok()) {
    // Initialize ROS message to publish the range measurement
		sensors_simulator::Range range_out;

    // Wait until pose information has been received to update agent data
    if(xy1){
      xPos1 = xy1->x;
  		yPos1 = xy1->y;
    }
		if(xy2){
      xPos2 = xy2->x;
  		yPos2 = xy2->y;
    }

    // Calculate the range
		double dx = xPos2 - xPos1;
		double dy = yPos2 - yPos1;
		double range = sqrt(dx * dx + dy * dy);

    // Terminal output for debugging
    cout << "X1: " << xPos1 << "    Y1: " << yPos1 << endl;
    cout << "X2: " << xPos2 << "    Y2: " << yPos2 << endl;
    cout << "Delta X: " << dx << "    Delta Y: " << dy << endl;
    cout << "Range: " << range << endl;

    // Load up the ROS msg for publishing
    range_out.seq = count;
		range_out.stamp = ros::Time::now();
    range_out.frame_id = "Gazebo";
		range_out.range = range;
    range_out.x1 = xPos1;
    range_out.y1 = yPos1;
    range_out.x2 = xPos2;
    range_out.y2 = yPos2;

    // Publish range information if some data is available
    if(xy1 || xy2){
      range_pub.publish(range_out);
    }

    // Run ROS modules once
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;
}
