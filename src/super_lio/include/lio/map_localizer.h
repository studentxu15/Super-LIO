#pragma once
#include "common/ds.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
namespace LI2Sup {
class MapLocalizer {
public:
  explicit MapLocalizer(rclcpp::Node &node);
  ~MapLocalizer();
  void submit(const NavState &state, const BASIC::CloudPtr &cloud);
  void finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace LI2Sup
