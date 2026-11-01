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

#include "ghost_brain/localizer.hpp"
#include <cmath>

namespace ghost_brain
{

void Localizer::reset(const geometry_msgs::msg::Pose & start)
{
  pose_ = start;
    last_velocity_time_ns_ = 0;
    have_velocity_time_ = false;
}

void Localizer::on_velocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  const int64_t current_time_ns =
    static_cast<int64_t>(velocity.header.stamp.sec) * 1000000000LL +
    velocity.header.stamp.nanosec;

  if (!have_velocity_time_) {
    last_velocity_time_ns_ = current_time_ns;
    have_velocity_time_ = true;
    return;
  }

  if (current_time_ns <= last_velocity_time_ns_) {
    return;
  }

  const double dt =
    static_cast<double>(current_time_ns - last_velocity_time_ns_) / 1e9;

  last_velocity_time_ns_ = current_time_ns;

    const auto & q = pose_.orientation;

  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double vx = velocity.twist.linear.x;
  const double vy = velocity.twist.linear.y;
  const double vz = velocity.twist.linear.z;

  const double map_vx = vx * std::cos(yaw) - vy * std::sin(yaw);
  const double map_vy = vx * std::sin(yaw) + vy * std::cos(yaw);

  pose_.position.x += map_vx * dt;
  pose_.position.y += map_vy * dt;
  pose_.position.z += vz * dt;
}

void Localizer::on_imu(const sensor_msgs::msg::Imu & imu)
{
  // The compass gives the heading directly
  pose_.orientation = imu.orientation;
}

void Localizer::on_sonar(const sensor_msgs::msg::LaserScan & scan, const nav_msgs::msg::OccupancyGrid & map)
{
  // TODO(hackathon): bonus! Use the walls the sonar sees to correct the drift
  (void)scan;
  (void)map;
}

}  // namespace ghost_brain
