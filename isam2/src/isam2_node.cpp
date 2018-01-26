#include <ros/ros.h>
#include "../include/isam2/isam2.h"

// // Standard headers, added last, so we know headers above work on their own
// #include <fstream>
// #include <iostream>
//
// // Finally, once all of the factors have been added to our factor graph, we will want to
// // solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// // GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// // standard Levenberg-Marquardt solver
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
//
// // Once the optimized values have been calculated, we can also calculate the marginal covariance
// // of desired variables
// #include <gtsam/nonlinear/Marginals.h>
// #include <isam2/StampedXYT.h>

// #include "boost/foreach.hpp"
#define foreach BOOST_FOREACH

using namespace std;
using namespace gtsam;


void writeResults(Values &results, string outputFile)
{
  ofstream resultFile(outputFile.c_str());

  Values::ConstFiltered<Pose2> result_poses = results.filter<Pose2>();
  foreach (const Values::ConstFiltered<Pose2>::KeyValuePair& key_value, result_poses) {
    Pose2 p = key_value.value;
    string k = Symbol(key_value.key);
    resultFile << k << " " << p.x() << " " << p.y() << " " << p.theta() << endl;
  }
}




int main (int argc, char** argv) {

     double newx, newy, newyaw;
     Pose2 estimatedPose;

     Values estimatedValues, result;
     isam2::StampedXYT estimated_pose_out;

     ros::init(argc, argv, "ISAM2");
     ros::NodeHandle nh;
     ros::NodeHandle nh_private("~");
     iSAM2 estimator(nh, nh_private);

     ros::Rate loop_rate(10);

     int count = 0;

     while(ros::ok()) {

          // Spin Once to get ROS data BEFORE we do an iSAM2 step
          ros::spinOnce();
          loop_rate.sleep();
          ++count;

     }

     cout << "Error 1: " << estimator.SSE_1 << "  Error 2: " << estimator.SSE_2 << endl;

     // if(estimator.flag_use_isam){
     //      result = estimator.isam.calculateEstimate();
     // } else {
     //      result = LevenbergMarquardtOptimizer(estimator.graph, estimator.initialEstimate).optimize();
     // }
     //
     // double error = estimator.graph.error(result);

     ofstream os("BATCH_iSAM2.dot");
     estimator.graph.saveGraph(os, estimator.initialEstimate);
     writeResults(estimator.currentEstimate, "results_iSAM2.csv");

     // estimator.initialEstimate.print("BEFORE estimates: ");
     // estimator.graph.print("BEFORE: ");
     // estimatedValues.print("estimated Values: ");

     return(0);
}
