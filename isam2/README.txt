#####################################
            Description
#####################################

This package will run the iSAM2 estimation algorithm adapted to the multi-agent cooperative navigation scenario. For more information on the iSAM2 algorithm, please refer to documentation available at https://collab.cc.gatech.edu/borg/gtsam?destination=node%2F299.

#####################################
	     Running the ROS Node
#####################################

If you are simulating the iSAM2 algorithm as standalone then in a terminal:
  rosrun isam2 isam2

#####################################
	            Inputs
#####################################

The iSAM2 algorithm takes in the available range measurement and the command velocities for the specific agent.

#####################################
	            Outputs
#####################################

The outputs of the iSAM2 node is the estimated pose and the corresponding  covariance associated with that pose
