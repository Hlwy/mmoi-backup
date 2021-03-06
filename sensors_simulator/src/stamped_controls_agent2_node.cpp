/** Description:
*               Repackages command velocities sent to the agent for following a
* certain path by adding in additional variables useful for validating further
* estimation algorithms
*/

#include <ros/ros.h>
#include <../include/sensors_simulator/sensors_simulator.h>
#include <sensors_simulator/StampedCmd.h>

using namespace std;

// Initialize global variable storage
geometry_msgs::Twist::ConstPtr cmds;
double tv,tw,dt;
ros::Duration tdt;
ros::Time temp_t2;
bool new_data = false;

// Callback to be called every time a command velocity is available
void agent2Callback(const geometry_msgs::Twist::ConstPtr& msg2)
{
  // Store the available command velocities
  tv = msg2->linear.x;
  tw = msg2->angular.z;
  // Calculate how much time has past since last command velocity
  tdt = ros::Time::now() - temp_t2;
  dt = tdt.toSec();
  /** If too much time has past since last available command, assume we are at
  * an initial location and store a change in time as '0' corresponding to the
  * notion of "we just started therefore we haven't moved in time"
  */
  if(dt > 10){
    dt = 0;
  }
  // Store our current time for use in calculating the next dt
  temp_t2 = ros::Time::now();

  // Terminal output for debugging
  cout << "T2: " << temp_t2 << "    Dt2: " << dt << "    Vel 2: " << tv << "   Ang. Rate 2: " << tw << endl;

  // Notify that new data is available
  new_data = true;
}

int main(int argc, char** argv) {

  // Initialize ROS Node
	ros::init(argc, argv, "bot2_cmd_stamper");
	ros::NodeHandle n;
  // Initialize a publisher for the re-packaged control input data
  ros::Publisher cmd_pub = n.advertise<sensors_simulator::StampedCmd>("bot2_stamped_cmds",1000);
  // Setup a ROS subscriber to listen for available control inputs
  ros::Subscriber sub2 = n.subscribe("agent2/mobile_base/commands/velocity", 1000, agent2Callback);

  // Output rate for the re-packaged data
	ros::Rate loop_rate(500);
  // Initialize counter to count how many times we have published the new data
  int count = 0;

	while (ros::ok()) {
    // Setup to ROS msg to publish data to
    sensors_simulator::StampedCmd cmd_out;

    /** If new data is available then update and publish new data, otherwise do
    * nothing
    */
    if(new_data){

      // Load ROS msg with new data
      cmd_out.seq = count;
      cmd_out.stamp = ros::Time::now();
      cmd_out.dt = dt;
      cmd_out.v = tv;
      cmd_out.w = tw;

      // Queue new data
      cmd_pub.publish(cmd_out);

      // Terminal output for debugging
      cout << "Publishing Agent 2 Commands..." << endl;
      // Update Counter
      ++count;
      // New data has been stored, notify that we are waiting for newer data
      new_data = false;
    }

    // Publish queued data and wait
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
