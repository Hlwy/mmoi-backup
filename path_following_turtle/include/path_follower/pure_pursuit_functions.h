#ifndef pure_pursuit_functions_h
#define pure_pursuit_functions_h

#include <armadillo>
#include "sys_params.h"

using namespace arma;

class PurePursuit
{
public:
	/** Function to saturate the angular velocity command sent to the robot
		@param [omega]:		Angular velocity

		@returns [sat_vel]:	Saturated angular velocity
	*/
	float saturate(fmat omega);

	/** Function to find the closest point on the entire path to current agent pose
		@param [path]:		Path the robot is going to reference itself to
		@param [pose]:		Robots pose

		@returns [nearIdx]:	Saturated angular velocity
	*/
	uword findNearest(fmat path, fmat pose);

	/** Function to find the closest point in the next few points
		@param [path]:			Path the robot is going to reference itself to
		@param [pose]:			Robots pose
		@param [lookAheadIdxDist]:	How much the lookAheadIdx  increments
		@param [ProjPointIdx]:		Index of the projected point

		@returns [ProjPointIdx]:	Updates index of the project point
	*/
	uword updateProj(fmat path, fmat pose, float lookAheadIdxDist, uword ProjPointIdx);


	/** Function to compute the control commands to be sent to the robots
		@param [path]:		Path the robot is going to reference itself to
		@param [curPose]:	Robot's current pose
		@param [deltaDist]:	Distance b/w the discretized path's waypoints
		@param [ProjPointIdx]:	Index of the projected point

		@returns [v]:		Control input for Linear Velocity 	[m/s]
		@returns [w]:		Control input for Angular Velocity	[rad/s]
	*/
	fmat step_controls(fmat path, fmat curPose, float deltaDist, ucolvec ProjPointIdx);


	/** Function: "drives", or updates the robot's current pose by integrating kinematic eqs.
		@param [control_inputs]:	[v, w] Control Inputs used to control the robot
		@param [dt]:			Time Step
		@param [lastPose]:		Last pose of the robot, [x_old, y_old, theta_old]

		@RETURNS: 	Matrix containing robot pose history

		@column [x]:			X-coordinate of the robot
		@column [y]:			Y-coordinate of the robot
		@column [theta]:		Heading of the robot
	*/
	fmat drive_robot(fmat control_inputs, float dt, fmat lastPose);


	/** Function: Runs the PurePursuit Controller for the robots
		@param [path2follow]:	[v, w] Control Inputs used to control the robot
		@param [delta]:		Distance b/w the discretized path's (path to follow) waypoints
		@param [goalRadius]:	Radius around the projected point for the robot to reach
		@param [initialPose]:	Initial Pose of the robot

		@RETURNS: 		Matrix containing robot pose and control input history

		@column [x]:		X-coordinate of the robot
		@column [y]:		Y-coordinate of the robot
		@column [theta]:	Heading of the robot
		@column [v]:		Control input for Linear Velocity 	[m/s]
		@column [w]:		Control input for Angular Velocity	[rad/s]
	*/
	fmat PurePursuitController(fmat path2follow, float delta, float goalRadius, fmat initialPose);

};
#endif
