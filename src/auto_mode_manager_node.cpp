// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <random>
#include "auto_mode_manager/msg/auto_mode_status.hpp"

namespace auto_mode_manager {
/** @brief Foundation node without any operation or actuator publisher. */
class AutoModeManagerNode : public rclcpp::Node {
public:
  /** @brief Start in Standby; no automatic motion until contracts are implemented. */
  AutoModeManagerNode() : Node("auto_mode_manager") {
    std::random_device random;
    producer_session_ = (static_cast<uint64_t>(random()) << 32) | random();
    if (producer_session_ == 0) {
      producer_session_ = 1;
    }
    status_publisher_ = create_publisher<msg::AutoModeStatus>("/auto_mode/status", 10);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { publish_status(); });
    RCLCPP_INFO(get_logger(), "Standby: contracts unresolved, no automatic output");
  }
private:
  /** @brief Report foundation state without implying safety or a board mode ACK. */
  void publish_status() {
    msg::AutoModeStatus status;
    status.stamp = now();
    status.producer_session = producer_session_;
    status.update_sequence = ++update_sequence_;
    status.navigation_status = msg::AutoModeStatus::STANDBY;
    status.automatic_output_enabled = false;
    status.contracts_confirmed = false;
    status.reason = "Contracts unresolved; automatic output disabled";
    status_publisher_->publish(status);
  }
  uint64_t producer_session_{};
  uint64_t update_sequence_{};
  rclcpp::Publisher<msg::AutoModeStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace auto_mode_manager

/** @brief Run the independent manager foundation. */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auto_mode_manager::AutoModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
