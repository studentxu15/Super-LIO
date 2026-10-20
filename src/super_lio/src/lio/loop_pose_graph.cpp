#include "lio/loop_pose_graph.h"
#include <cmath>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2UpdateParams.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace LI2Sup {
namespace {
gtsam::Pose3 convert(const LoopPoseGraph::Pose &p) {
  return gtsam::Pose3(gtsam::Rot3(Eigen::Quaterniond(p.linear()).normalized()),
                      p.translation());
}
bool valid(const LoopPoseGraph::Pose &p) {
  return p.matrix().allFinite() &&
         (p.linear().transpose() * p.linear() - Eigen::Matrix3d::Identity())
                 .norm() < 1e-3 &&
         std::abs(p.linear().determinant() - 1.) < 1e-3;
}
} // namespace
struct LoopPoseGraph::Impl {
  std::unique_ptr<gtsam::ISAM2> solver;
  size_t nodes = 0, factors = 0;
  std::string error;
  Impl()
      : solver(std::make_unique<gtsam::ISAM2>([] {
          gtsam::ISAM2Params p;
          p.relinearizeThreshold = 0.01;
          p.relinearizeSkip = 1;
          p.factorization = gtsam::ISAM2Params::CHOLESKY;
          return p;
        }())) {}
};
LoopPoseGraph::LoopPoseGraph() : impl_(std::make_unique<Impl>()) {}
LoopPoseGraph::~LoopPoseGraph() = default;
const std::string &LoopPoseGraph::lastError() const { return impl_->error; }
bool LoopPoseGraph::optimize(std::vector<Pose> &poses,
                             const std::vector<Edge> &edges,
                             double &initial_error, double &final_error) {
  impl_->error.clear();
  if (poses.empty() || poses.size() < impl_->nodes ||
      edges.size() < impl_->factors) {
    impl_->error = "graph does not extend the committed prefix";
    return false;
  }
  for (size_t i = 0; i < poses.size(); ++i)
    if (!valid(poses[i])) {
      impl_->error = "non-rigid or non-finite pose " + std::to_string(i);
      return false;
    }
  for (const auto &e : edges)
    if (e.a < 0 || e.b < 0 || e.a == e.b || size_t(e.a) >= poses.size() ||
        size_t(e.b) >= poses.size() || !valid(e.z)) {
      impl_->error =
          "invalid edge " + std::to_string(e.a) + " -> " + std::to_string(e.b);
      return false;
    }
  try {
    // Trial copy provides rollback if an incremental update or validation
    // fails. Graph factors are immutable; the old solver remains usable on
    // failure.
    auto trial = std::make_unique<gtsam::ISAM2>(*impl_->solver);
    gtsam::NonlinearFactorGraph added;
    gtsam::Values initial, all_initial;
    for (size_t i = 0; i < poses.size(); ++i) {
      all_initial.insert(i, convert(poses[i]));
      if (i >= impl_->nodes)
        initial.insert(i, convert(poses[i]));
    }
    if (impl_->nodes == 0)
      added.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          0, convert(poses[0]), gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    // Pose3 tangent ordering is rotation FIRST, translation SECOND.
    const auto odom = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << .01, .01, .01, .05, .05, .05).finished());
    const auto loop = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.),
        gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector6() << .05, .05, .05, .1, .1, .1).finished()));
    for (size_t i = impl_->factors; i < edges.size(); ++i) {
      const auto &e = edges[i];
      gtsam::SharedNoiseModel noise = e.loop ? gtsam::SharedNoiseModel(loop)
                                             : gtsam::SharedNoiseModel(odom);
      added.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          e.a, e.b, convert(e.z), noise);
    }
    gtsam::NonlinearFactorGraph combined = trial->getFactorsUnsafe();
    combined.push_back(added);
    initial_error = combined.error(all_initial);
    gtsam::ISAM2UpdateParams update;
    update.force_relinearize = true;
    update.forceFullSolve = true;
    trial->update(added, initial, update);
    // A loop can move distant nodes. Revisit nonlinear linearizations until the
    // cost stabilizes, with a bounded iteration budget on the backend thread.
    double previous = initial_error;
    gtsam::Values estimate;
    for (int i = 0; i < 10; ++i) {
      estimate = trial->calculateEstimate();
      final_error = combined.error(estimate);
      if (i > 0 &&
          std::abs(previous - final_error) < 1e-6 * std::max(1., previous))
        break;
      previous = final_error;
      trial->update(gtsam::NonlinearFactorGraph(), gtsam::Values(), update);
    }
    estimate = trial->calculateEstimate();
    final_error = combined.error(estimate);
    if (!std::isfinite(initial_error) || !std::isfinite(final_error) ||
        final_error > initial_error + 1e-6 * std::max(1., initial_error)) {
      impl_->error = "non-finite or increased graph cost";
      return false;
    }
    std::vector<Pose> optimized;
    optimized.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      Pose p = Pose::Identity();
      p.matrix() = estimate.at<gtsam::Pose3>(i).matrix();
      if (!valid(p)) {
        impl_->error = "invalid optimized pose";
        return false;
      }
      optimized.push_back(p);
    }
    impl_->solver = std::move(trial);
    impl_->nodes = poses.size();
    impl_->factors = edges.size();
    poses = std::move(optimized);
    return true;
  } catch (const std::exception &e) {
    impl_->error = e.what();
    return false;
  }
}
} // namespace LI2Sup
