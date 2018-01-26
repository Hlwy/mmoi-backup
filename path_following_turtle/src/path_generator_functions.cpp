#include <../include/path_follower/path_generator_functions.h>

using namespace std;

bool PathGen::isEqualFloat(float a, float b) {
	float tolerance;
	bool flag_val;

	tolerance =  eps(a);

	flag_val = fabs(a-b) < tolerance;

#ifdef DEBUG_IS_EQUAL_CHECK
	printf("isEqualFloat Inputs:    [a] = %.3f	[b] = %.3f	\r\n", a,b);
	printf("isEqualFloat intermediate values:    [tolerance] = %.3f	\r\n", tolerance);
	printf("isEqualFloat Outputs:    [flag_val] = %d	\r\n", flag_val);
	printf("\r\n");
#endif

	return flag_val;
}


float PathGen::distance(fmat ptA, fmat ptB) {
	fmat dist;			// Initialize matrix for output
	dist = norm(ptA - ptB);		// Perform calculation

#ifdef DEBUG_DISTANCE_FUNCTION
	printf("distance Inputs: \r\n");
	ptA.print("	ptA: ");
	ptB.print("	ptB: ");
	printf("distance Outputs: \r\n");
	ptA.print("	dist: ");
	printf("\r\n");
#endif
	float val_out = as_scalar(dist);
	return val_out;

}

fmat PathGen::sine_follower(float A, float b) {
	// INITIALIZE VARIABLES FOR USAGE
	fcolvec path_x, path_y;
	fmat sine_path, x_val, y_val;

	// CREATE PATH POINTS
	for(int i = 0; i < 360; i++){
		float rad = (2 * datum::pi * i) / 360; // DEGREES IN RAD
		x_val = rad;
		y_val = A * sin(rad) + b;
		path_x = join_vert(path_x,x_val);
		path_y = join_vert(path_y,y_val);
	}

	sine_path = join_horiz(path_x, path_y);

#ifdef DEBUG_PATH_OUTPUT
	sine_path.save("VAR_sine_path.csv", csv_ascii);
#endif

	return sine_path;
}


fmat PathGen::cos_follower(float A, float b) {
	// INITIALIZE VARIABLES FOR USAGE
	fcolvec path_x, path_y;
	fmat cos_path, x_val, y_val;

	// CREATE PATH POINTS
	for(int i = 0; i < 360; i++){
		float rad = (2 * datum::pi * i) / 360; // DEGREES IN RAD
		x_val = rad;
		y_val = A * cos(rad) + b;
		path_x = join_vert(path_x,x_val);
		path_y = join_vert(path_y,y_val);
	}

	cos_path = join_horiz(path_x, path_y);

#ifdef DEBUG_PATH_OUTPUT
	cos_path.save("VAR_cos_path.csv", csv_ascii);
#endif

	return cos_path;
}

fmat PathGen::line_follower(float startX, float startY, float endX, float endY) {
	// INITIALIZE VARIABLES FOR USAGE
	fcolvec path_x, path_y;
	fmat line_path, x_val, y_val;
	float slope, dx, dy;

	dx = endX - startX;
	dy = endY - startY;
	slope = dy/dx;

	// CREATE PATH POINTS
	for(int i = 0; i < 100; i++){
		x_val = (float) i * dx * 0.01;
		y_val = slope * x_val + startY;
		path_x = join_vert(path_x+startX,x_val);
		path_y = join_vert(path_y,y_val);
	}

	line_path = join_horiz(path_x, path_y);

#ifdef DEBUG_PATH_OUTPUT
	line_path.save("VAR_line_path.csv", csv_ascii);
#endif

	return line_path;
}

fmat PathGen::opti_sine_follower(float A, float b, float theta) {
	// INITIALIZE VARIABLES FOR USAGE
	fcolvec path_x, path_y;
	fmat sine_path, rawPath, rotPath, rotPathNew;
	float x_val, y_val;
	float xInit = INITIAL_X1;
	float yInit = INITIAL_Y1;
	// Rotation maxtrix to put in OptiTrack frame
	fmat R;
	float thetaR = (2 * datum::pi * theta) / 360; // DEGREES IN RAD
	R << cos(thetaR) << -sin(thetaR) << endr
	  << sin(thetaR) << cos(thetaR) << endr;

	// CREATE PATH POINTS
	for(int i = 1; i <= 360; i++){
		float rad = (2 * datum::pi * i) / 360; // DEGREES IN RAD
		x_val = rad;
		y_val = A * sin(rad) + b;

		// Setup the virtual path coordinates
		rawPath << x_val << endr << y_val << endr;
		// Rotate the virtual path coordinates into the OptiTrack environment
		rotPathNew = R * rawPath;
		rotPathNew = rotPathNew.t();
		// Add the OptiTrack coordinates to the path matrix
		rotPath = join_vert(rotPath,rotPathNew);
	}

	// Seperate the rotated coordinates for translation
	path_x = rotPath.col(0);
	path_y = rotPath.col(1);
	float dx = xInit - path_x(0);
	float dy = yInit - path_y(0);

	// Translate the coordinates to line up in the OptiTrack environment
	path_x = path_x + dx;
	path_y = path_y + dy;

	sine_path = join_horiz(path_x, path_y);

#ifdef DEBUG_PATH_OUTPUT
	R.print("Rotation Matrix:");
	cout << "DX: " << dx << "	DY: " << dy << endl;
	rotPath.save("VAR_raw_rotPath.csv", csv_ascii);
	sine_path.save("VAR_opti_sine_path.csv", csv_ascii);
#endif

	return sine_path;
}


fmat PathGen::opti_cos_follower(float A, float b, float theta) {
	// INITIALIZE VARIABLES FOR USAGE
	fcolvec path_x, path_y;
	fmat cos_path, rawPath, rotPath, rotPathNew;
	float x_val, y_val;
	float xInit = INITIAL_X2;
	float yInit = INITIAL_Y2;
	// Rotation maxtrix to put in OptiTrack frame
	fmat R;
	float thetaR = (2 * datum::pi * theta) / 360; // DEGREES IN RAD
	R << cos(thetaR) << -sin(thetaR) << endr
	  << sin(thetaR) << cos(thetaR) << endr;

	// CREATE PATH POINTS
	for(int i = 1; i <= 360; i++){
		float rad = (2 * datum::pi * i) / 360; // DEGREES IN RAD
		x_val = rad;
		y_val = A * cos(rad) + b;

		// Setup the virtual path coordinates
		rawPath << x_val << endr << y_val << endr;
		// Rotate the virtual path coordinates into the OptiTrack environment
		rotPathNew = R * rawPath;
		rotPathNew = rotPathNew.t();
		// Add the OptiTrack coordinates to the path matrix
		rotPath = join_vert(rotPath,rotPathNew);
	}

	// Seperate the rotated coordinates for translation
	path_x = rotPath.col(0);
	path_y = rotPath.col(1);
	float dx = xInit - path_x(0);
	float dy = yInit - path_y(0);

	// Translate the coordinates to line up in the OptiTrack environment
	path_x = path_x + dx;
	path_y = path_y + dy;

	cos_path = join_horiz(path_x, path_y);

#ifdef DEBUG_PATH_OUTPUT
	R.print("Rotation Matrix:");
	cout << "DX: " << dx << "	DY: " << dy << endl;
	rotPath.save("VAR_raw_rotPath.csv", csv_ascii);
	cos_path.save("VAR_opti_cos_path.csv", csv_ascii);
#endif

	return cos_path;
}

fmat PathGen::equidistantPoints(fmat Path_in) {

	// INITIALIZE PLACEHOLDERS FOR ALL MATRICES AND VECTORS
	int i, j, k, count, num_row_path_in, num_row_path;
	fvec x, y, xcomputed, ycomputed;
	fmat path_in, path, path_out;
	fmat dPathIn, distPathIn;
	fmat sortedDist;
	fmat dPath, distPath;
	float dPoints;
	float minimumDelta, maximumDelta, minPathDist, maxDelta, delta_val;

	path_in = Path_in; // This is done for parameter passing

	// COMPUTE THE MINIMUM DELTA DISTANCE BASED ON POINTS FROM ORIGINAL PATH INPUT
	x = path_in(0,0);
	y = path_in(0,1);

	dPathIn = diff(path_in);
	distPathIn = sqrt(pow(dPathIn.col(0),2) + pow(dPathIn.col(1),2));
	sortedDist = sort(distPathIn);
	sortedDist = sortedDist( sortedDist > 0);

	minimumDelta = as_scalar(0.0001 * mean(sortedDist));
	maximumDelta = as_scalar(0.005 * mean(sortedDist));

	// REMOVE POINTS THAT ARE CLOSER THAN minimumDelta DISTANCE
	path = path_in.row(0);
	num_row_path_in = path_in.n_rows - 1;
	j = 0;

	for (i = 0; i < num_row_path_in; i++) {
		dPoints = distance(path.row(j),path_in.row(i+1));

		if(dPoints >= minimumDelta) {
			path = join_cols(path, path_in.row(i+1));
			j++;
		}
	}

	dPath = diff(path);
	distPath = sqrt(pow(dPath.col(0),2) + pow(dPath.col(1),2));

	minPathDist = as_scalar(min(distPath));
	maxDelta = as_scalar(min(maximumDelta, minPathDist));

	// OUTPUT: final delta distance b/w all equidistant points
	delta_val = max(minimumDelta, maxDelta);

	num_row_path = path.n_rows - 1;

	// LOOP THRU POINTS OF PATH TO COMPUTE DISCRETIZED POINTS IN B/W
	for(i = 0; i < num_row_path; i++) {
		// If the x-coordinate is not the same
		if(~isEqualFloat(path(i,0), path(i+1,0))) {
			xcomputed = linspace<fmat>(path(i,0), path(i+1,0), ceil(norm(path.row(i)-path.row(i+1))/delta_val));
        		ycomputed = linspace<fmat>(path(i,1), path(i+1,1), xcomputed.n_elem);
		} // If the y-coordinate is not the same
		else if(~isEqualFloat(path(i,1), path(i+1,1))) {
			ycomputed = linspace<fmat>(path(i,1), path(i+1,1), ceil(norm(path.row(i)-path.row(i+1))/delta_val));
        		xcomputed = linspace<fmat>(path(i,0), path(i+1,0), ycomputed.n_elem);
		} else {
			xcomputed = path(i,0);
        		ycomputed = path(i,1);
		}

		if( isEqualFloat(as_scalar(x.tail(1)), as_scalar(xcomputed.head(1))) && isEqualFloat(as_scalar(y.tail(1)), as_scalar(ycomputed.head(1))) ) {
			int range_x = xcomputed.n_rows - 1;
			int range_y = ycomputed.n_rows - 1;
			x = join_vert(x.rows(span::all),xcomputed.tail_rows(range_x));
			y = join_vert(y.rows(span::all),ycomputed.tail_rows(range_y));
		} else {
			x = join_vert(x.rows(span::all),xcomputed);
			y = join_vert(y.rows(span::all),ycomputed);
		}

	}

	path_out = join_horiz(x,y);
	path_out.shed_row(0);		// USED ONLY DUE TO DUPLICATED INITIAL VALUE !!!UNABLE TO DETERMINE WHY TOLERANCE IS EQUAL TO EXACTLY 0 WHEN USING isEqualFloat FUNCTION

	int delt_size = path_out.n_rows;
	fmat delta = zeros<fmat>(delt_size, 1);
	delta.head_rows(1) = delta_val;

#ifdef DEBUG_PATH_GEN_VARIABLES
	cout << "path_in Size: " << size(path_in) << endl;
	cout << "dPathIn Size: " << size(dPathIn) << endl;
	cout << "distPathIn Size: " << size(distPathIn) << endl;
	cout << "sortedDist Size: " << size(sortedDist) << endl;
	cout << "Minimum Delta: " << minimumDelta << endl;
	cout << "Maximum Delta: " << maximumDelta << endl;

	cout << "Post Path Size: " << size(path) << endl;
	cout << "dPath Size: " << size(dPath) << endl;
	cout << "distPath Size: " << size(distPath) << endl;
	cout << "Minimum Path Distance: " << minPathDist << endl;
	cout << "Max Delta: " << maxDelta << endl;
	cout << "Delta: " << delta << endl;

	path_in.save("VAR_path_in.csv", csv_ascii);
	dPathIn.save("VAR_dPathIn.csv", csv_ascii);
	distPathIn.save("VAR_distPathIn.csv", csv_ascii);
	sortedDist.save("VAR_sortedDist.csv", csv_ascii);
#endif

	fmat outputs;
	outputs = join_horiz(path_out, delta);

#ifdef DEBUG_PATH_GEN_OUTPUT
	cout << "x out Size: " << size(x) << endl;
	cout << "y out Size: " << size(y) << endl;
	cout << "path out Size: " << size(path_out) << endl;
	cout << "outputs Size: " << size(outputs) << endl;

	x.save("VAR_x_out.csv", csv_ascii);
	y.save("VAR_y_out.csv", csv_ascii);
	path_out.save("VAR_path_out.csv", csv_ascii);
	outputs.save("VAR_outputs.csv", csv_ascii);
#endif

	// FUNCTION OUTPUT
	return outputs;
}
