// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "auto_mode_manager/control.hpp"
#include "auto_mode_manager/msg/auto_mode_status.hpp"
#include "robot_port/msg/robot_command.hpp"
#include "robot_port/msg/robot_operation.hpp"
#include "gn10_navigation/msg/control_lease.hpp"
#include "gn10_navigation/msg/path_velocity.hpp"
#include "gn10_navigation/srv/compute_path.hpp"
#include "gn10_navigation/action/follow_path.hpp"
#include "field_target_pub/msg/target_set.hpp"
#include "lidar_perception_system/msg/moving_bucket_track.hpp"
#include "shooter_calculator/msg/aim_target.hpp"
#include "shooter_calculator/msg/shot_solution.hpp"

namespace auto_mode_manager {
using Command = robot_port::msg::RobotCommand;
using Operation = robot_port::msg::RobotOperation;
using Status = msg::AutoModeStatus;
using Track = lidar_perception_system::msg::MovingBucketTrack;
using Targets = field_target_pub::msg::TargetSet;
using Aim = shooter_calculator::msg::AimTarget;
using Shot = shooter_calculator::msg::ShotSolution;
using Lease = gn10_navigation::msg::ControlLease;
using Velocity = gn10_navigation::msg::PathVelocity;
using Plan = gn10_navigation::srv::ComputePath;
using Follow = gn10_navigation::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<Follow>;
using Twist = geometry_msgs::msg::Twist;

class AutoModeManager : public rclcpp::Node {
public:
  AutoModeManager() : Node("auto_mode_manager"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    map_frame_ = declare_parameter<std::string>("map_frame", "map", descriptor);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link", descriptor);
    track_topic_ = declare_parameter<std::string>("track_topic", "/moving_bucket_detector/target", descriptor);
    yaw_gain_ = declare_parameter<double>("yaw_gain", 1.0, descriptor);
    max_yaw_speed_ = declare_parameter<double>("max_yaw_speed_rad_s", 0.5, descriptor);
    yaw_acceleration_ = declare_parameter<double>("yaw_acceleration_rad_s2", 1.0, descriptor);
    angle_sign_ = declare_parameter<int>("angle_sign", 1, descriptor);
    angle_offset_ = declare_parameter<double>("angle_offset_deg", 0.0, descriptor);
    if (map_frame_.empty() || base_frame_.empty() || track_topic_.empty() ||
        !std::isfinite(yaw_gain_) || yaw_gain_ <= 0 ||
        !std::isfinite(max_yaw_speed_) || max_yaw_speed_ <= 0 ||
        !std::isfinite(yaw_acceleration_) || yaw_acceleration_ <= 0 ||
        (angle_sign_ != -1 && angle_sign_ != 1) || !std::isfinite(angle_offset_)) {
      throw std::invalid_argument("Invalid frame, topic, yaw controller or angle conversion parameter");
    }
    std::random_device random;
    session_ = (static_cast<uint64_t>(random()) << 32) | random();
    if (session_ == 0) { session_ = 1; }
    status_pub_ = create_publisher<Status>("/auto_mode/status", 10);
    operation_pub_ = create_publisher<Operation>("/robot/operation", 10);
    aim_pub_ = create_publisher<Aim>("/auto_mode/aim_target", 10);
    lease_pub_ = create_publisher<Lease>("/navigation/control_lease", 10);
    planner_ = create_client<Plan>("/navigation/compute_path");
    follower_ = rclcpp_action::create_client<Follow>(this, "/navigation/follow_path");
    command_sub_ = create_subscription<Command>("/robot/command", 100,
      [this](Command::ConstSharedPtr message) { observe_command(*message); });
    targets_sub_ = create_subscription<Targets>("/field/targets", 10,
      [this](Targets::ConstSharedPtr message) { targets_ = message; });
    track_sub_ = create_subscription<Track>(track_topic_, 10,
      [this](Track::ConstSharedPtr message) { track_ = message; });
    shot_sub_ = create_subscription<Shot>("/shooter/solution", 10,
      [this](Shot::ConstSharedPtr message) {
        if (execution_ && message->producer_session == session_ &&
            message->generation == execution_->generation && message->target_id == target_) {
          shot_ = message;
        }
      });
    velocity_sub_ = create_subscription<Velocity>("/navigation/path_velocity", 10,
      [this](Velocity::ConstSharedPtr message) {
        if (execution_ && execution_->following && !execution_->cancelled &&
            message->producer_session == session_ && message->generation == execution_->generation) {
          velocity_ = message->valid && message->frame_id == base_frame_ && finite(message->velocity)
            ? message->velocity : Twist{};
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "Coordinator ready; observe Sleep/Standby then Move to start");
  }

private:
  struct Execution {
    uint64_t generation{};
    bool cancelled{false}, goal_pending{false}, following{false};
    std::optional<int64_t> plan_request_id;
    GoalHandle::SharedPtr handle;
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PointStamped aim;
    bool aim_valid{false};
  };

  static bool finite(const geometry_msgs::msg::Point & point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }
  static bool finite(const Twist & twist) {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
      std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
      std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  }
  static bool valid_pose(const geometry_msgs::msg::Pose & pose) {
    const auto & q = pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return finite(pose.position) && std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
  }
  bool pose(geometry_msgs::msg::PoseStamped & result) {
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      result.header = tf.header;
      result.pose.position.x = tf.transform.translation.x;
      result.pose.position.y = tf.transform.translation.y;
      result.pose.position.z = tf.transform.translation.z;
      result.pose.orientation = tf.transform.rotation;
      return valid_pose(result.pose);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
  bool track_valid() const {
    return track_ && track_->valid && track_->header.frame_id == map_frame_ &&
      finite(track_->position) && std::isfinite(track_->velocity.x) &&
      std::isfinite(track_->velocity.y) && std::isfinite(track_->velocity.z) &&
      std::isfinite(track_->confidence) && track_->confidence >= 0 && track_->confidence <= 1 &&
      std::all_of(track_->state_covariance.begin(), track_->state_covariance.end(),
                  [](double value) { return std::isfinite(value); });
  }
  bool current(const std::shared_ptr<Execution> & context) const {
    return execution_ == context && !context->cancelled;
  }
  void cancel_handle(const std::shared_ptr<Execution> & context) {
    if (context->handle) {
      follower_->async_cancel_goal(context->handle);
    }
  }
  void stop(uint8_t state, const std::string & reason) {
    state_ = state;
    reason_ = reason;
    velocity_ = Twist{};
    yaw_rate_ = 0;
    shot_.reset();
    if (execution_) {
      auto old = execution_;
      old->cancelled = true;
      execution_.reset();
      if (old->plan_request_id) { planner_->remove_pending_request(*old->plan_request_id); }
      if (old->goal_pending || old->handle) {
        cancelling_ = old;
        cancel_handle(old);
      }
    }
    // Do not wait for an Action acknowledgement before clearing ROS outputs.
    publish_outputs(Twist{}, Aim{});
  }
  void observe_command(const Command & command) {
    const bool move = request_.observe(command.navigation_command);
    if (command.navigation_command == Command::SLEEP || command.navigation_command == Command::STANDBY) {
      stop(command.navigation_command == Command::SLEEP ? Status::STANDBY : Status::READY,
           command.navigation_command == Command::SLEEP ? "Sleep" : "Ready for a new Move");
      return;
    }
    if (command.navigation_command != Command::MOVE || command.target_id > 7) {
      stop(Status::FAIL, "Invalid navigation command or target ID");
      return;
    }
    if (!move) { return; }
    if (execution_ || cancelling_ || startup_cancelling_) {
      reason_ = "Move consumed: previous execution has not ended";
      return;
    }
    geometry_msgs::msg::PoseStamped start;
    if (!pose(start)) {
      stop(Status::FAIL, "Move consumed: robot pose TF unavailable or invalid");
      return;
    }
    auto context = std::make_shared<Execution>();
    context->generation = ++generation_;
    target_ = command.target_id;
    velocity_ = Twist{};
    yaw_rate_ = 0;
    shot_.reset();
    if (target_ == 7) {
      if (!track_valid()) {
        stop(Status::FAIL, "Move consumed: tracking target unavailable or invalid");
        return;
      }
      execution_ = context;
      state_ = Status::TRACKING;
      reason_ = "Yaw tracking";
      return;
    }
    const field_target_pub::msg::FieldTarget * target = nullptr;
    if (targets_) {
      for (const auto & item : targets_->targets) {
        if (item.target_id == target_) { target = &item; break; }
      }
    }
    if (!target || !target->stop_pose_valid || target->stop_pose.header.frame_id != map_frame_ ||
        !valid_pose(target->stop_pose.pose)) {
      stop(Status::FAIL, "Move consumed: requested stop pose unavailable or invalid");
      return;
    }
    if (!planner_->service_is_ready() || !follower_->action_server_is_ready() || !startup_cleaned_) {
      stop(Status::FAIL, "Move consumed: navigation unavailable or startup cleanup pending");
      return;
    }
    context->aim = target->aim_point;
    context->aim_valid = target->aim_valid && target->aim_point.header.frame_id == map_frame_ &&
      finite(target->aim_point.point);
    execution_ = context;
    state_ = Status::MOVING;
    reason_ = "Planning";
    auto request = std::make_shared<Plan::Request>();
    request->start = start;
    request->goal = target->stop_pose;
    request->generation = context->generation;
    const auto pending = planner_->async_send_request(request, [this, context](rclcpp::Client<Plan>::SharedFuture future) {
      context->plan_request_id.reset();
      if (!current(context)) { return; }
      try {
        const auto result = future.get();
        if (!result->success || result->generation != context->generation || result->path.poses.empty()) {
          stop(Status::FAIL, "Path planning failed: " + result->reason);
          return;
        }
        context->path = result->path;
        // Publish the existing follower's mandatory permit before sending its goal.
        reason_ = "Starting path follower";
      } catch (const std::exception & error) {
        stop(Status::FAIL, std::string("Path planning failed: ") + error.what());
      }
    });
    context->plan_request_id = pending.request_id;
  }
  void follow(const std::shared_ptr<Execution> & context) {
    Follow::Goal goal;
    goal.path = context->path;
    goal.producer_session = session_;
    goal.generation = context->generation;
    context->goal_pending = true;
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this, context](GoalHandle::SharedPtr handle) {
      context->goal_pending = false;
      context->handle = handle;
      if (!handle) {
        if (cancelling_ == context) { cancelling_.reset(); }
        if (current(context)) { stop(Status::FAIL, "FollowPath goal rejected"); }
      } else if (!current(context)) {
        cancel_handle(context);
      } else {
        context->following = true;
        reason_ = "Following path";
      }
    };
    options.result_callback = [this, context](const GoalHandle::WrappedResult & result) {
      context->handle.reset();
      if (cancelling_ == context) { cancelling_.reset(); }
      if (!current(context)) { return; }
      context->following = false;
      velocity_ = Twist{};
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->success && result.result->generation == context->generation) {
        state_ = Status::GOAL;
        reason_ = "Goal reached";
      } else {
        stop(Status::FAIL, "Follower ended: " + (result.result ? result.result->reason : "no result"));
      }
    };
    follower_->async_send_goal(goal, options);
  }

  Aim aim_target() {
    Aim aim;
    aim.header.stamp = now();
    aim.header.frame_id = map_frame_;
    aim.producer_session = session_;
    aim.generation = generation_;
    aim.target_id = target_;
    if (!execution_) { return aim; }
    if (state_ == Status::TRACKING && track_valid()) {
      aim.valid = true;
      aim.position = track_->position;
      aim.predicted = track_->observation_mode == Track::PREDICTED;
      aim.last_observation_stamp = track_->last_observation_stamp;
      aim.velocity_m_s = track_->velocity;
      aim.confidence = track_->confidence;
      for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
          aim.position_covariance[row*3+column] = track_->state_covariance[row*6+column];
        }
      }
    } else if ((state_ == Status::MOVING || state_ == Status::GOAL) && execution_->aim_valid) {
      aim.valid = true;
      aim.position = execution_->aim.point;
      aim.last_observation_stamp = aim.header.stamp;
      aim.confidence = 1;
    }
    return aim;
  }
  void publish_lease() {
    Lease lease;
    lease.stamp = now();
    lease.producer_session = session_;
    lease.generation = generation_;
    lease.update_sequence = ++lease_sequence_;
    lease.enabled = execution_ && state_ == Status::MOVING;
    lease_pub_->publish(lease);
  }
  void publish_outputs(const Twist & twist, Aim aim) {
    // Correlation fields remain mandatory in the unchanged bridge and shooter APIs.
    aim.header.stamp = now();
    aim.header.frame_id = map_frame_;
    aim.producer_session = session_;
    aim.generation = generation_;
    aim.target_id = target_;
    aim_pub_->publish(aim);
    Operation operation;
    operation.stamp = now();
    operation.producer_session = session_;
    operation.generation = generation_;
    operation.update_sequence = ++operation_sequence_;
    // Idle snapshots stay neutral so a newly started bridge can acquire this producer.
    operation.navigation_status = state_ == Status::READY ? Status::STANDBY : state_;
    operation.target_id = execution_ ? target_ : 0;
    operation.velocity_x_m_s = twist.linear.x;
    operation.velocity_y_m_s = twist.linear.y;
    operation.velocity_yaw_rad_s = twist.angular.z;
    geometry_msgs::msg::PoseStamped robot;
    if (aim.valid && pose(robot)) {
      const auto error = bearing(robot.pose.position.x, robot.pose.position.y,
        tf2::getYaw(robot.pose.orientation), aim.position.x, aim.position.y);
      const auto angle = error ? wire_angle(*error, angle_sign_, angle_offset_) : std::nullopt;
      const int index = target_ == 7 ? 0 : target_ >= 1 && target_ <= 3 ? target_ :
        target_ == 6 ? 4 : target_ == 4 || target_ == 5 ? 5 : -1;
      if (angle && index >= 0) { operation.angles_deg[index] = static_cast<int8_t>(*angle); }
    }
    const bool shot_valid = aim.valid && shot_ && shot_->valid && shot_->in_calibration_range &&
      std::isfinite(shot_->belt_speed_m_s) && std::abs(shot_->belt_speed_m_s) <= 3.4e38 &&
      (state_ == Status::MOVING || state_ == Status::GOAL);
    if (shot_valid) { operation.belt_launcher_speed_m_s = static_cast<float>(shot_->belt_speed_m_s); }
    operation_pub_->publish(operation);
    Status status;
    status.stamp = now();
    status.navigation_status = state_;
    status.target_id = execution_ ? target_ : 0;
    status.reason = reason_;
    if (execution_ && target_ >= 1 && target_ <= 6 && !shot_valid) {
      status.reason += "; shot unavailable: " + (shot_ ? shot_->reason : "waiting for aim/calibration/solution");
    }
    status_pub_->publish(status);
    publish_lease();
  }
  void tick() {
    // Once per process, clear an Action left by an earlier manager instance.
    if (!startup_cleaned_ && !startup_cancelling_ && !execution_ && follower_->action_server_is_ready()) {
      startup_cancelling_ = true;
      follower_->async_cancel_all_goals([this](auto) {
        startup_cancelling_ = false;
        startup_cleaned_ = true;
      });
    }
    const auto context = execution_;
    if (context && state_ == Status::MOVING && !context->path.poses.empty() &&
        !context->goal_pending && !context->following) {
      // One permit-only tick precedes the Action to avoid discovery/order races.
      if (permit_sent_) { follow(context); }
      else { publish_lease(); permit_sent_ = true; }
    } else {
      permit_sent_ = false;
    }
    Twist twist;
    if (context && state_ == Status::MOVING && context->following) { twist = velocity_; }
    if (context && state_ == Status::TRACKING) {
      geometry_msgs::msg::PoseStamped robot;
      const bool available = track_valid() && pose(robot);
      const auto error = available ? bearing(robot.pose.position.x, robot.pose.position.y,
        tf2::getYaw(robot.pose.orientation), track_->position.x, track_->position.y) : std::nullopt;
      if (error) {
        yaw_rate_ = yaw_velocity(*error, yaw_rate_, yaw_gain_, max_yaw_speed_, yaw_acceleration_);
        twist.angular.z = yaw_rate_;
        reason_ = "Yaw tracking";
      } else {
        yaw_rate_ = 0;
        reason_ = "Tracking input unavailable or bearing undefined";
      }
    }
    publish_outputs(twist, aim_target());
  }

  std::string map_frame_, base_frame_, track_topic_, reason_{"Standby; observe Sleep/Standby then Move"};
  double yaw_gain_{}, max_yaw_speed_{}, yaw_acceleration_{}, angle_offset_{}, yaw_rate_{};
  int angle_sign_{};
  uint64_t session_{}, generation_{1}, operation_sequence_{}, lease_sequence_{};
  uint8_t state_{Status::STANDBY}, target_{};
  bool startup_cleaned_{false}, startup_cancelling_{false}, permit_sent_{false};
  MoveRequest request_;
  Twist velocity_;
  Targets::ConstSharedPtr targets_;
  Track::ConstSharedPtr track_;
  Shot::ConstSharedPtr shot_;
  std::shared_ptr<Execution> execution_, cancelling_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<Operation>::SharedPtr operation_pub_;
  rclcpp::Publisher<Aim>::SharedPtr aim_pub_;
  rclcpp::Publisher<Lease>::SharedPtr lease_pub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<Targets>::SharedPtr targets_sub_;
  rclcpp::Subscription<Track>::SharedPtr track_sub_;
  rclcpp::Subscription<Shot>::SharedPtr shot_sub_;
  rclcpp::Subscription<Velocity>::SharedPtr velocity_sub_;
  rclcpp::Client<Plan>::SharedPtr planner_;
  rclcpp_action::Client<Follow>::SharedPtr follower_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace auto_mode_manager

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<auto_mode_manager::AutoModeManager>());
  rclcpp::shutdown();
  return 0;
}
