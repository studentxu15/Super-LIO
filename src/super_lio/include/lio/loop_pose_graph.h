#pragma once
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

namespace LI2Sup {
// ROS/PCL-independent, transactional incremental pose graph.
class LoopPoseGraph {
public:
  using Pose = Eigen::Isometry3d;
  struct Edge {
    int a, b;
    Pose z;
    bool loop;
  };
  LoopPoseGraph();
  ~LoopPoseGraph();
  // New nodes/edges must extend the committed prefix. Failure changes neither
  // the committed iSAM2 state nor the caller's pose estimates.
  bool optimize(std::vector<Pose> &poses, const std::vector<Edge> &edges,
                double &initial_error, double &final_error);

  const std::string &lastError() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace LI2Sup
