

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lio/super_lio_reloc.h"
#include "ros/ROSWrapper.h"

using namespace LI2Sup;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();

    auto lio = std::make_shared<SuperLIO>();
    lio->setROSWrapper(data_wrapper);
    lio->init();

    auto timer = data_wrapper->create_wall_timer(
        std::chrono::milliseconds(2), [lio]() { lio->process(); },
        data_wrapper->getSensorCallbackGroup());

    rclcpp::spin(data_wrapper);

    lio->finishLoopClosure();
    lio->saveMap();
    lio->printTimeRecord();

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("super_lio"),
                 "Startup or execution failed: %s", e.what());
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
