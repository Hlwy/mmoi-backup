#####################################
            Description
#####################################

This node is responsible for simulating the range measurements between different robots. Range measurements can be obtained for simulated robot models in Gazebo, or from actual robots specified by the name-specific "Rigid Body" markers present in the OptiTrack environment.

#####################################
    Range Measurement Simulation
#####################################

Gazebo Simulation:
------------------

In a terminal run:

  roslaunch sensors_simulator gazebo.launch


OptiTrack Simulation:
---------------------

  1) Setup your robots by initializing the "Rigid Body" markers, in the OptiTrack software, for each robot.

  2) Give a specific name to the "Rigid Body" marker corresponding to a specific robot, for all robots.

  3) Make sure the robot names you used are updated in the roslaunch file (optiTrack.launch).

  4) In a terminal, run:
  roslaunch sensors_simulator optiTrack.launch
