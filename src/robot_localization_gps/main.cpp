#include <boost/optional.hpp>
#include <boost/pointer_cast.hpp>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

class UnaryFactor : public NoiseModelFactor1<Pose2> {
  Point2 m_position{0.0, 0.0};

public:
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  UnaryFactor(Key j, Point2 position, const SharedNoiseModel &model)
      : NoiseModelFactor1(model, j), m_position(position) {}
  virtual ~UnaryFactor() {}
  Vector evaluateError(const Pose2 &q,
                       OptionalMatrixTypeT<Pose2> H = nullptr) const override {
    if (H) {
      // the Jacobians are :
      // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
      // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
      (*H) = Matrix(2, 3);
      (*H) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    }
    return Vector2(q.x() - m_position.x(), q.y() - m_position.y());
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
  }
};

int main() {
  // Create a non-linear factor graph, since the orientation part is non-linear.
  gtsam::NonlinearFactorGraph nlGraph;

  // Add Odometry factors
  gtsam::Pose2 odom(2.0, 0.0, 0.0);
  gtsam::noiseModel::Diagonal::shared_ptr odomNoise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
  nlGraph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, odom, odomNoise));
  nlGraph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, odom, odomNoise));

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  noiseModel::Diagonal::shared_ptr unaryNoise =
      noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
  nlGraph.add(UnaryFactor(1, Point2(0.0, 0.0), unaryNoise));
  nlGraph.add(UnaryFactor(2, Point2(2.0, 0.0), unaryNoise));
  nlGraph.add(UnaryFactor(3, Point2(4.0, 0.0), unaryNoise));

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
  std::cout << "x1 covariance:\n"
            << marginals.marginalCovariance(1) << std::endl;
  std::cout << "x2 covariance:\n"
            << marginals.marginalCovariance(2) << std::endl;
  std::cout << "x3 covariance:\n"
            << marginals.marginalCovariance(3) << std::endl;

  return 0;
}