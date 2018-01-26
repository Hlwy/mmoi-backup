#include <ros/ros.h>
#include "../include/isam2/isam2.h"

// Standard headers, added last, so we know headers above work on their own
#include <fstream>
#include <iostream>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>
#include <isam2/StampedXYT.h>

#include "boost/foreach.hpp"
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

     ros::init(argc, argv, "AGENT_1_ISAM2");
     ros::NodeHandle nh;
     ros::NodeHandle nh_private("~");
     iSAM2 estimator(nh, nh_private);

     ros::Rate loop_rate(10);

     int count = 0;

     while(ros::ok()) {

          // cout << "****************************************************" << endl;
          // cout << "Count: " << count << endl;

          // Spin Once to get ROS data BEFORE we do an iSAM2 step
          ros::spinOnce();

          // // estimator.currentEstimate.print("Current estimate: ");
          // // estimator.graph.saveGraph(cout, estimator.currentEstimate);
          // estimator.graph.saveGraph(cout, estimator.initialEstimate);
          //
          // // Print timings
          // // tictoc_print_();
          //
          // // Publish Current Estimated Pose to ROS
          // estimated_pose_out.seq = count;
     	// estimated_pose_out.stamp = ros::Time::now();
     	// estimated_pose_out.x = estimatedPose.x();
          // estimated_pose_out.y = estimatedPose.y();
          // estimated_pose_out.yaw = estimatedPose.theta();
          //
          // estimator.estimatePub.publish(estimated_pose_out);
          //
          // estimator.initialEstimate.print("BEFORE estimates: ");
          // ros::spinOnce();
          // // loop_rate.sleep();
          //
          // // cout << "             ************************           " << endl;
          // // estimator.initialEstimate.print("Initial estimate: ");
          // // // estimator.currentEstimate.print("Current estimate: ");
          // //
          // // // newx = estimator.currentEstimate.at(symbol('A',numN)).x;
          // // // newy = estimator.currentEstimate.at(symbol('A',numN)).y;
          // // // newyaw = estimator.currentEstimate.at(symbol('A',numN)).theta;
          // // //
          // // // cout << "New X: " << newx << "    New Y: " << newy << "    New Theta: " << newyaw << endl;
          // //
          // // cout << "             ************************           " << endl;
          //
          //
          // // estimator.graph.print("AFTER: ");
          // gttic_(update);
          // // cout << "Updating.... " << endl;
          // // Update iSAM with the new factors
          // isam.update(estimator.graph, estimator.initialEstimate);
          // // isam.printStats();
          // /** Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
          // * If accuracy is desired at the expense of time, update(*) can be called additional times
          // * to perform multiple optimizer iterations every step.*/
          // // cout << "Updating One More Time.... " << endl;
          // // isam.update();
          // gttoc_(update);
          // gttic_(calculateEstimate);
          // // cout << "Calculating Estimate.... " << endl;
          // estimator.currentEstimate = isam.calculateEstimate();
          // gttoc_(calculateEstimate);
          // // cout << "             ************************           " << endl;
          // // estimator.currentEstimate.print("CURRENT estimates: ");
          // // estimatedPose = estimator.currentEstimate.at<Pose2>(Symbol('A',numN));
          //
          // estimatedValues.insert(Symbol('E',count),estimatedPose);
          // estimator.initialEstimate.print("BEFORE estimates: ");
          // estimator.graph.print("AFTER: ");
          // ros::spinOnce();
          loop_rate.sleep();
          ++count;

          // // Clear the factor graph and values for the next iteration
          // estimator.graph.resize(0);
          // estimator.graph = NonlinearFactorGraph();
          // estimator.initialEstimate = Values();
          // estimator.initialEstimate.clear();

          // estimator.numOdomUno = 0;
          // estimator.numOdomDos = 0;
     }

     cout << "Error 1: " << estimator.SSE_1 << endl;

     // if(estimator.flag_use_isam){
     //      result = estimator.isam.calculateEstimate();
     // } else {
     //      result = LevenbergMarquardtOptimizer(estimator.graph, estimator.initialEstimate).optimize();
     // }
     //
     // double error = estimator.graph.error(result);

     writeResults(estimator.currentEstimate, "results_agent1_isam2.csv");

     // estimator.initialEstimate.print("BEFORE estimates: ");
     // estimator.graph.print("BEFORE: ");
     // estimatedValues.print("estimated Values: ");
     // ofstream os("iSAM2.dot");
     // estimator.graph.saveGraph(os, estimator.initialEstimate);

     return(0);
}
