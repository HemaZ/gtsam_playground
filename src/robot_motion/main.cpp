#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

int main() {
  // Create a non-linear factor graph, since the orientation part is non-linear.
  gtsam::NonlinearFactorGraph nlGraph;

  // Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  gtsam::Pose2 priorMean(0, 0, 0);
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{0.3, 0.3, 0.1});
  nlGraph.add(gtsam::PriorFactor<gtsam::Pose2>(1, priorMean, priorNoise));

  // Add Odometry factors
  gtsam::Pose2 odom(2.0, 0.0, 0.0);
  gtsam::noiseModel::Diagonal::shared_ptr odomNoise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

  nlGraph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, odom, odomNoise));
  nlGraph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, odom, odomNoise));

  nlGraph.print();

  // Create the data structure to hold the initialEstimate estimate to the
  // solution
  gtsam::Values initials;
  initials.insert(1, gtsam::Pose2(0.5, 0.2, 0.3));
  initials.insert(2, gtsam::Pose2(2.5, 0.2, 0.3));
  initials.insert(3, gtsam::Pose2(4.5, 0.2, 0.3));
  initials.print("\nInitial Estimate:\n"); // print

  // Optimize the graph
  gtsam::Values results =
      gtsam::LevenbergMarquardtOptimizer(nlGraph, initials).optimize();
  results.print("Final Result:\n");

  // Query the marginals
  std::cout.precision(2);
  gtsam::Marginals marginals(nlGraph, results);
  std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << std::endl;
  std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << std::endl;
  std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << std::endl;

  return 0;
}