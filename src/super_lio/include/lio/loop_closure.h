#pragma once
#include "common/ds.h"
#include <condition_variable>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace LI2Sup {
// Owns a background pose graph. The local ESKF and local voxel map are never
// corrected.
class LoopClosure {
public:
  explicit LoopClosure(rclcpp::Node &node);
  ~LoopClosure();
  void submit(const NavState &state, const BASIC::CloudPtr &body_cloud);
  void finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace LI2Sup
