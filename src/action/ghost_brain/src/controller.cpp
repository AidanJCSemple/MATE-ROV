// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ghost_brain/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ghost_brain
{
namespace
{

using geometry_msgs::msg::Point;

constexpr double kMaxForward = 1.0;
constexpr double kMaxLeft = 0.6;
constexpr double kMaxUp = 0.6;
constexpr double kMaxYawRate = 1.5;

// Chase a point this far ahead along the route, so corners come out smooth
constexpr double kLookahead = 0.7;

double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

Point lerp(const Point & a, const Point & b, double t)
{
  Point p;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.z = a.z + t * (b.z - a.z);
  return p;
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

geometry_msgs::msg::Twist Controller::compute_velocity(const geometry_msgs::msg::Pose & pose, const Path & path)
{
  geometry_msgs::msg::Twist command;

  if (path.empty()) {
    return command;
  }

  const auto & q = pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const Point & here = pose.position;

  // 1. The closest point on the route
  size_t segment = 0;
  Point cursor = path[0];

  if (path.size() > 1) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const Point & a = path[i];
      const Point & b = path[i + 1];
      const double abx = b.x - a.x;
      const double aby = b.y - a.y;
      const double abz = b.z - a.z;
      const double length2 = abx * abx + aby * aby + abz * abz;

      double t = 0.0;
      if (length2 > 1e-12) {
        t = ((here.x - a.x) * abx + (here.y - a.y) * aby + (here.z - a.z) * abz) / length2;
        t = std::clamp(t, 0.0, 1.0);
      }

      const Point closest = lerp(a, b, t);
      const double d = distance(closest, here);
      if (d < best) {
        best = d;
        segment = i;
        cursor = closest;
      }
    }
  }

  // 2. The point a little way ahead of it, and how much route is left
  Point target = cursor;
  double to_end = 0.0;

  if (path.size() > 1) {
    size_t k = segment + 1;

    to_end = distance(cursor, path[k]);
    for (size_t i = k; i + 1 < path.size(); ++i) {
      to_end += distance(path[i], path[i + 1]);
    }

    double travel = kLookahead;
    Point from = cursor;
    while (true) {
      const double d = distance(from, path[k]);
      if (d >= travel) {
        target = lerp(from, path[k], d > 1e-9 ? travel / d : 0.0);
        break;
      }
      travel -= d;
      from = path[k];
      if (k + 1 >= path.size()) {
        target = path[k];
        break;
      }
      ++k;
    }
  }

  const double stop_distance = path.size() > 1 ? to_end : distance(path[0], here);

  const double ex = target.x - here.x;
  const double ey = target.y - here.y;
  const double ez = target.z - here.z;
  const double error = std::hypot(ex, ey, ez);

  // Close enough to the end of the route: hover
  if (error < 1e-6 || (error < 0.05 && stop_distance < 0.1)) {
    return command;
  }

  // 3. Speed: fast along the route, slowing down toward its end
  double speed = std::min(kMaxForward, std::max(0.25, 1.5 * stop_distance));

  // 4. Face where we are going, so the camera looks ahead
  const double horizontal = std::hypot(ex, ey);
  if (horizontal > 0.2) {
    const double heading_error = wrap_angle(std::atan2(ey, ex) - yaw);
    command.angular.z = std::clamp(2.0 * heading_error, -kMaxYawRate, kMaxYawRate);
    // Not yet facing it: ease off while turning
    speed *= 0.5 + 0.5 * std::max(0.0, std::cos(heading_error));
  }

  // 5. Velocity toward the point, in the ghost's own frame
  const double vx = ex / error * speed;
  const double vy = ey / error * speed;
  const double vz = ez / error * speed;

  const double forward = vx * std::cos(yaw) + vy * std::sin(yaw);
  const double left = -vx * std::sin(yaw) + vy * std::cos(yaw);
  const double up = vz;

  // Keep within each axis's limit without changing the direction
  double scale = 1.0;
  if (std::abs(forward) > kMaxForward) {
    scale = std::min(scale, kMaxForward / std::abs(forward));
  }
  if (std::abs(left) > kMaxLeft) {
    scale = std::min(scale, kMaxLeft / std::abs(left));
  }
  if (std::abs(up) > kMaxUp) {
    scale = std::min(scale, kMaxUp / std::abs(up));
  }

  command.linear.x = forward * scale;
  command.linear.y = left * scale;
  command.linear.z = up * scale;

  return command;
}

}  // namespace ghost_brain