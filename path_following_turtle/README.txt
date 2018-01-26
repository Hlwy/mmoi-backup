#####################################
 Installation of Necessary Libraries
#####################################

1) Download LAPACK:   

sudo apt-get install liblapack-dev liblapack-doc-man liblapack-doc liblapack-pic liblapack3 liblapack-test liblapack3gf liblapacke liblapacke-dev

2) Download BLAS: 

sudo apt-get install libblas-dev

3) Download OpenBLAS: 

sudo apt-get install libopenblas-dev

4) Download ARPACK: [OPTIONAL]

5) Download SuperLU: [OPTIONAL]

6) Download Armadillo:
	i) cd catkin_ws/armadillo-7.200.2
	ii) cmake .
	iii) make
	iv) sudo make install

7) Download GTSAM:
	i) cd catkin_ws/gtsam-3.2.1
	ii) mkdir build
	iii) cd build
	iv) cmake ..
	v) make check
	vi) make install

#####################################
        ROS Launch Run Order
#####################################

Terminal 1: Turtlebot Bringup
	roslaunch path_following_turtle baseline.launch

Terminal 2: (Optional) Load the gazebo App for easier reset of gazebo models
	rosrun path_following_turtle gazeboApp_resetModels

Terminal 3: Rosbag Recorder
	roslaunch path_following_turtle bagger.launch

Terminal 4: Starts Agent path following algorithm
	roslaunch path_following_turtle PFT.launch


#####################################
	      Path Generation
#####################################

Currently, any changes to what path you want the robots to follow has to be done
in 'path_generator_functions.cpp'.

/** NOTE: line_follower function is buggy as far as the actual generation of the path to follow */
/** TODO:
    Add Function for to follow coordinate matrix as a path
    OptiTrack Following
*/

#####################################
      ROS Configuration File 	// IN PROGRESS
#####################################

If running a de-centralized ROS network (Ex. running multiple turtlebot agents from a "base-station" computer) then you will need to edit/update the environment variable configuration file (turtle_env.sh) on each turtlebot.

To successfully run the agents from a base computer, you need to make sure that the 'turtle_env.sh' file is executable before running.

#####################################
	     Gazebo Simulation
#####################################

/** Reset Simulated Robot Models */
From a terminal run:
	rosrun path_following_turtle gazeboApp_resetModels

From another terminal run:
	rosrun dynamic_reconfigure dynparam set /gazeboApp_resetModels flag_reset_gazebo True && rosrun dynamic_reconfigure dynparam set /gazeboApp_resetModels flag_reset_gazebo False


/** Change Robot Reset Coordinates (x, y) */
From terminal run:
	rosrun dynamic_reconfigure dynparam set /gazeboApp_resetModels initial_x# x_value
	rosrun dynamic_reconfigure dynparam set /gazeboApp_resetModels initial_y# y_value

with '#' in the intial_x# and initial_y#, as the identifier of the coordinates cooresponding to the robot #

#####################################
	 Topics to Record
#####################################

## Truth Data
/clock
/gazebo/model_states
/pose1_pubber
/pose2_pubber
/tf
/tf_static

## Measurement Data
/agent1/robot_pose_ekf/odom_combined
/agent1/odom
/agent1/mobile_base/sensors/imu_data
/agent1/mobile_base/commands/velocity

/agent2/robot_pose_ekf/odom_combined
/agent2/odom
/agent2/mobile_base/sensors/imu_data
/agent2/mobile_base/commands/velocity

## Estimator Values
/bot1_stamped_cmds
/bot2_stamped_cmds
/bot1_pose_estimate
/bot2_pose_estimate

#####################################
      ROS BAG Post Processing
#####################################

To convert the recorded topic from a rosbag to a data file run in terminal:
	rostopic echo -b file.bag -p /topic > data.txt

#####################################
	   Data Filtering
#####################################

To filter data in a .txt file, in the commandline type:
	awk '(NR==1) || ($field# > val ) ' file_in > file out

This function will create file_out with rows from file_in that meet the criteria specified
where:
	NR: 	beginning row in file_in
	field#: Column in file_in to evaluate criteria
	val:	value of criteria to meet
