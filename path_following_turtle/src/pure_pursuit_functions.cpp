#include <../include/path_follower/pure_pursuit_functions.h>

using namespace std;

float PurePursuit::saturate(fmat omega) {
	fmat sat_vel;

	if(as_scalar(abs(omega)) > MAX_ANGULAR_VELOCITY) {
		sat_vel = sign(omega) * MAX_ANGULAR_VELOCITY;}
	else{
		sat_vel = omega;}
	float vel_out = as_scalar(sat_vel);

	return vel_out;
}

uword PurePursuit::findNearest(fmat path, fmat pose) {
	fmat x_pts, y_pts, dist;
	mat minIdx;

	x_pts = pow((path.col(0) - pose(0,0)),2);
	y_pts = pow((path.col(1) - pose(0,1)),2);
	dist = sqrt(x_pts + y_pts);

	uword MinRowIdx, MinColIdx, nearIdx;
	float min_val = dist.min(nearIdx);

	//cout << "min val: " << min_val << "	@: " << MinRowIdx << ", " << MinColIdx << endl;

	return nearIdx;

}


uword PurePursuit::updateProj(fmat path, fmat pose, float lookAheadIdxDist, uword ProjPointIdx) {

	fmat lookforward, dist;
	float searchDist = 2 * lookAheadIdxDist;

	if(ProjPointIdx + searchDist < path.n_rows) {
		lookforward = ProjPointIdx + searchDist;
	} else {
		lookforward = path.n_rows;
	}

	fmat x_val, y_val;
	x_val = square(path(span(ProjPointIdx,as_scalar(lookforward)),0) - pose(0,0));
	y_val = square(path(span(ProjPointIdx,as_scalar(lookforward)),1) - pose(0,1));

	dist = sqrt(x_val + y_val);

	uword MinRowIdx, MinColIdx, curProjection, nearIdx;
	float min_val = dist.min(curProjection);
	ProjPointIdx = ProjPointIdx + curProjection;
/**
	cout << "x val size: " << size(x_val) << endl;
	cout << "y val size: " << size(y_val) << endl;

	x_val.print("X val: ");
	y_val.print("Y val: ");

	cout << "dist size: " << size(dist) << endl;
	dist.print("dist: ");

	//cout << "min val: " << min_val << "	min val @: " << MinRowIdx << ", " << MinColIdx << endl;
	cout << "curProjection: " << curProjection << endl;

	cout << "ProjPointIdx: " << ProjPointIdx << endl;
*/
	return nearIdx;

}


fmat PurePursuit::step_controls(fmat path, fmat curPose, float deltaDist, ucolvec ProjPointIdx) {

	fmat LastPose, lookAheadPoint;
	uword idxInsertVal;
	ucolvec lastProjIdx, idxInserter;
	fmat w, controls;

	LastPose << curPose(0) << curPose(1) << curPose(2) << endr;

	int lookAheadIdxDist = round(LOOKAHEAD_DIST/deltaDist);

	if(ProjPointIdx.is_empty()) { 		// First time, check all points and find the nearest point as the projection
		idxInsertVal = findNearest(path, curPose);
	} else {
		lastProjIdx = ProjPointIdx.tail(1);
		//cout << "	lastProjIdx: " << as_scalar(lastProjIdx) << endl;
		idxInsertVal = updateProj(path, curPose, lookAheadIdxDist, as_scalar(lastProjIdx));
	}

	idxInserter = idxInsertVal;
	ProjPointIdx = join_cols(ProjPointIdx, idxInserter);
	//cout << "	Index Value Inserted: " << idxInsertVal << endl;
	//cout << "	ProjIdx size after Insertion: " << size(ProjPointIdx) << endl;


	// Compute carrot point based on projection. If near end, then the end point is the carrot point
	int lookAheadIdx = as_scalar(ProjPointIdx.tail(1)) + lookAheadIdxDist;
	if(lookAheadIdx < path.n_rows) {
		lookAheadPoint = path.row(lookAheadIdx);
	} else {
		lookAheadPoint = path.tail_rows(1);
	}
	// curPose.print("Agent 2 Current Location: ");

	// Angle between robot heading and the line connecting robot and the carrot point
	float slope = atan2((lookAheadPoint(1) - curPose(1)), (lookAheadPoint(0) - curPose(0)));
	float alpha = slope - curPose(2);

	// cout << "LookAheadPoint: " << lookAheadPoint << "			Slope to Path: " << slope << "				Alpha: " << alpha << endl;

	// Angular velocity command for a differential drive robot is equal to the desired curvature to be followed by the robot.
	// Using eq. (2) on page 11 of Reference [1] (from matlab documentation on the PurePursuit.m)
	w = (2 * sin(alpha))/LOOKAHEAD_DIST;

	// Pick a constant rotation when robot is facing in the opposite direction of the path
	if(abs(abs(alpha) - datum::pi) < 1e-12) {
		w = sign(w) * 1;
	}

	float w_val = saturate(w);
	float v_val = MAX_LINEAR_VELOCITY;

	controls << v_val << w_val << endr;

	//controls.print("Controls: ");
	//cout << "Controls Size: " << size(controls) << endl;

	return controls;
}



fmat PurePursuit::drive_robot(fmat control_inputs, float dt, fmat LastPose) {
	// Extract values as a different typed variable from the input matrices
	float v = as_scalar(control_inputs.col(0));
	float w = as_scalar(control_inputs.col(1));

	//cout << "v: " << v << "	w: " << w << endl;

	float x_old = as_scalar(LastPose.col(0));
	float y_old = as_scalar(LastPose.col(1));
	float theta_old = as_scalar(LastPose.col(2));

	// Update the robot's pose using the kinematic equations
	float x_new = x_old + v * cos(theta_old) * dt;
	float y_new = y_old + v * sin(theta_old) * dt;
	float theta_new = theta_old + w * dt;

	// Re-package the new pose into a matrix
	fmat updatedPose;
	updatedPose << x_new << y_new << theta_new << endr;

	//updatedPose.print("New Pose: ");

	return updatedPose;
}


fmat PurePursuit::PurePursuitController(fmat path2follow, float delta, float goalRadius, fmat initialPose) {
	ucolvec ProjPointIdx;
	fmat controls_in, curPose, row_inserter;
	fmat PoseControlHistory, robotGoal, currentLocation;

	float dt = DT;

	robotGoal = path2follow.tail_rows(1);
	robotGoal.print("Robot Goal: ");

	curPose = initialPose;

	currentLocation = curPose.head_cols(2);
	currentLocation.print("Current Location: ");

	float distanceToGoal = norm(currentLocation - robotGoal);
	cout << "distanceToGoal: " << distanceToGoal << endl;

	controls_in << 0 << 0 << endr;

	PoseControlHistory = join_rows(curPose, controls_in);

	while(distanceToGoal > goalRadius) {

		controls_in = step_controls(path2follow, curPose, delta, ProjPointIdx);
		curPose = drive_robot(controls_in, dt, curPose);
		currentLocation = curPose.head_cols(2);

		row_inserter = join_rows(curPose, controls_in);
		row_inserter.print(" [x, y, theta, v, w]: ");

		PoseControlHistory = join_cols(PoseControlHistory, row_inserter);

		distanceToGoal = norm(currentLocation - robotGoal);
	}

	PoseControlHistory.save("VAR_Robot_History.csv", csv_ascii);

	return PoseControlHistory;
}
