#ifndef PATH_GENERATOR_FUNCTIONS_H
#define PATH_GENERATOR_FUNCTIONS_H

#include <armadillo>
#include "sys_params.h"

using namespace arma;

#define INITIAL_X1 1.708
#define INITIAL_Y1 -2.122
#define INITIAL_X2 -1.2612
#define INITIAL_Y2 -2.764

// #define DEBUG_PATH_OUTPUT

class PathGen
{
public:

	// Function to compare two float numbers for equality
	bool isEqualFloat(float, float);

	// Function to determine the Euclidean distance between two points
	float distance(fmat, fmat);

	/** Function allowing the user to easily modify a sine path for an Agent to follow
		@param [A]:    Amplitude of the sine wave
		@param [b]:    DC offset

		@returns  :    Returns [x_pts y_pts] , iX2 matrix w/ x and y coordinates of user-defined sine wave
	*/
	fmat sine_follower(float A, float b);


	/** Function allowing the user to easily modify a cosine path for an Agent to follow
		@param [A]:    Amplitude of the cosine wave
		@param [b]:    DC offset

		@returns  :    Returns [x_pts y_pts] , iX2 matrix w/ x and y coordinates of user-defined cosine wave
	*/
	fmat cos_follower(float A, float b);

	/** Function allowing the user to easily modify a line path for an Agent to follow
		@param [startX]:   Starting x point of the line
		@param [startY]:   Starting y point of the line
		@param [endX]:     Ending x point of the line
		@param [endY]:     Ending y point of the line

		@returns  :    Returns [x_pts y_pts] , iX2 matrix w/ x and y coordinates of user-defined  line
	*/
	fmat line_follower(float startX, float startY, float endX, float endY);

	/** Function allowing the user to easily modify a sine path for an Agent to follow in the OptiTrack environment
		@param [A]:    	Amplitude of the sine wave
		@param [b]:    	DC offset
		@param [theta]:	Angle (degrees) needed to rotate path

		@returns  :    Returns [x_pts y_pts] , iX2 matrix w/ x and y coordinates of user-defined sine wave
	*/
	fmat opti_sine_follower(float A, float b, float theta);


	/** Function allowing the user to easily modify a cosine path for an Agent to follow in the OptiTrack environment
		@param [A]:    	Amplitude of the cosine wave
		@param [b]:    	DC offset
		@param [theta]:	Angle (degrees) needed to rotate path

		@returns  :    Returns [x_pts y_pts] , iX2 matrix w/ x and y coordinates of user-defined cosine wave
	*/
	fmat opti_cos_follower(float A, float b, float theta);


	/** Function to discretized a predefined path into equidistant waypoints
		@param [Path_in]:    Path Agent must follow

		@returns [outputs] :    [x, y, delta] , iX3 matrix w/ discretized x and y coordinates of user-defined path and the uniform distance between them
	*/
	fmat equidistantPoints(fmat Path_in);
};

#endif /* PATH_GENERATOR_FUNCTIONS_H */
