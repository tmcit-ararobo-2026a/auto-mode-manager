// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>

namespace auto_mode_manager {
/** @brief Foundation node without any operation or actuator publisher. */
class AutoModeManagerNode : public rclcpp::Node {
public:
  /** @brief Start in Standby; no automatic motion until contracts are implemented. */
  AutoModeManagerNode() : Node("auto_mode_manager") {
    RCLCPP_INFO(get_logger(), "Standby: contracts unresolved, no automatic output");
  }
};
}  // namespace auto_mode_manager

/** @brief Run the independent manager foundation. */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auto_mode_manager::AutoModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
