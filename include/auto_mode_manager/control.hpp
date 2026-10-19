// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

namespace auto_mode_manager {
// A Move is an edge after an observed Sleep/Standby, never a queued request.
class MoveRequest {
public:
  bool observe(unsigned command) {
    const bool start = released_ && command == 2;
    released_ = command == 0 || command == 1;
    return start;
  }
private:
  bool released_{false};
};

inline double wrap(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::optional<double> bearing(double x, double y, double yaw,
                                     double target_x, double target_y) {
  const double dx = target_x - x, dy = target_y - y;
  if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(yaw) ||
      std::hypot(dx, dy) < 1e-6) {
    return std::nullopt;
  }
  return wrap(std::atan2(dy, dx) - yaw);
}

inline double yaw_velocity(double error, double previous, double gain,
                           double max_speed, double acceleration) {
  const double desired = std::clamp(gain * error, -max_speed, max_speed);
  const double step = acceleration * 0.02;
  return std::clamp(desired, previous - step, previous + step);
}

// Wire angles are signed integer degrees; an unrepresentable bearing is omitted.
inline std::optional<int> wire_angle(double error, int sign, double offset) {
  const double degrees = sign * error * 180.0 / std::acos(-1.0) + offset;
  if (!std::isfinite(degrees) || std::abs(degrees) > 20.0) {
    return std::nullopt;
  }
  return static_cast<int>(std::round(degrees));
}
}  // namespace auto_mode_manager
