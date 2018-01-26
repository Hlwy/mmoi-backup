




// SECTION: Initialization
//     Storage variables
//     iSAM2 Setup
//
// END SECTION



// Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
// and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
// structure is available that allows the user to set various properties, such as the relinearization threshold
// and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
// will approach the batch result.
ISAM2Params parameters;
parameters.relinearizeThreshold = 0.01;
parameters.relinearizeSkip = 1;
ISAM2 isam(parameters);


// SECTION: Retrieve measurements
//     Inter-Agent Range:
//                         range (m)
//     Propagated States:
//                         linear displacement (m)
//                         angular displacement (rad)
//                         time displacement (sec)
// END SECTION



// SECTION: Update Nonlinear Graph
//      @description: Add all available data to graph to retrieve the current state estimate
//
//      @output StampedEstimatedPoseWithCovariance.msg:
//              [timestamp x(m) y(m) yaw(rad) covariance(matrix)]
//
// END SECTION

/* EXAMPLE: VisualISAM2Example.cpp
*
* // Create a Factor Graph and Values to hold the new data
* NonlinearFactorGraph graph;
* Values initialEstimate;
*
* // Loop over the different poses, adding the observations to iSAM incrementally
* for (size_t i = 0; i < poses.size(); ++i) {
*
*   // Add factors for each landmark observation
*   for (size_t j = 0; j < points.size(); ++j) {
*     SimpleCamera camera(poses[i], *K);
*     Point2 measurement = camera.project(points[j]);
*     graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
*   }
*
*   // Add an initial guess for the current pose
*   // Intentionally initialize the variables off from the ground truth
*   initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::rodriguez(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
*
*   // If this is the first iteration, add a prior on the first pose to set the coordinate frame
*   // and a prior on the first landmark to set the scale
*   // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
*   // adding it to iSAM.
*   if( i == 0) {
*     // Add a prior on pose x0
*     noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1))); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
*     graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));
*
*     // Add a prior on landmark l0
*     noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
*     graph.push_back(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph
*
*     // Add initial guesses to all observed landmarks
*     // Intentionally initialize the variables off from the ground truth
*     for (size_t j = 0; j < points.size(); ++j)
*       initialEstimate.insert(Symbol('l', j), points[j].compose(Point3(-0.25, 0.20, 0.15)));
*
*   } else {
*     // Update iSAM with the new factors
*     isam.update(graph, initialEstimate);
*     // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
*     // If accuracy is desired at the expense of time, update(*) can be called additional times
*     // to perform multiple optimizer iterations every step.
*     isam.update();
*     Values currentEstimate = isam.calculateEstimate();
*     cout << "****************************************************" << endl;
*     cout << "Frame " << i << ": " << endl;
*     currentEstimate.print("Current estimate: ");
*
*     // Clear the factor graph and values for the next iteration
*     graph.resize(0);
*     initialEstimate.clear();
*   }
* }
*
 END EXAMPLE*/

// Add variables
// TODO

// Optimize/Relinearize/Etc.
// TODO

// Obtain Current Estimate
// TODO

// Plot the covariance of the last pose
Marginals marginals(*graph, result);
cout.precision(2);
cout << "\nP3:\n" << marginals.marginalCovariance(99) << endl;

// Construct Topic Message to be published
// TODO




int main(int argc, char** argv) {

	ros::init(argc, argv, "isam2_pubber");
	ros::NodeHandle n;
	ros::Publisher pose_estimate_pub = n.advertise<isam2::StampedXYT>("isam2_estimate",1000);
  ros::Subscriber sub1 = n.subscribe(agent1_topic_name, 1000, commandsCallback);
  // ros::Subscriber sub2 = n.subscribe("pose2_pubber", 1000, sensorCallback);

  isam2::StampedXYT pose_estimate;

	ros::Rate loop_rate(200);

	int count = 0;

	while (ros::ok()) {
    // If new control inputs available, store them internally and calculate new state (using iSAM2)

    pose_estimate.seq = count;
		pose_estimate.stamp = ros::Time::now();
		pose_estimate.x = est_x;
    pose_estimate.y = est_y;
    pose_estimate.yaw = est_yaw;

    pose_estimate_pub.publish(pose_estimate);

		ros::spinOnce();
    loop_rate.sleep();
		++count;
	}

	return 0;
}
