#ifndef sys_params_h
#define sys_params_h

/** File_Description:
	@disc: This file is used to easily find and change the 
	       parameters of the physical system, as well as some user-defined virtual, parameters used throughout the various header files
*/


/** lookAheadDist: 
	@disc: The lookahead distance changes the response of the
	       controller. Higher lookahead distance produces smooth
	       paths but the robot takes larger turns at corners.
	       Smaller lookahead distance closely follows the path and
	       robot takes sharp turns but may produce oscillations 
	       in the path.

	@default: 1.0
*/
#define LOOKAHEAD_DIST 0.5


/** maxAngularVelocity:  
	@disc: 	Maximum angular velocity robot is capable of
	@units: [rad/s]
*/
#define MAX_ANGULAR_VELOCITY 3.14

/** maxLinearVelocity:  
	@disc: 	Maximum linear velocity robot is capable of
	@units: [m/s]
*/
#define MAX_LINEAR_VELOCITY 0.65 //0.65


/** deltaTime:  
	@disc: 	Time step
	@units: [s]
*/
#define DT 0.1

#endif
