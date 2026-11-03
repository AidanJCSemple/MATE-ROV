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

#include "ghost_brain/ghost_brain_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ghost_brain
{
namespace
{

constexpr double kPersonMaxAgeS = 5.0;     // a person unseen this long is no longer where we saw them
constexpr double kStuckDistanceM = 0.3;    // less movement than this ...
constexpr double kStuckTimeS = 4.0;        // ... for this long means stuck
constexpr double kArrivalRadiusM = 0.45;   // close enough by our estimate to expect a catch
constexpr double kDwellTimeS = 1.5;        // how long to wait there before trying something else
constexpr double kPersonValue = 10.0;
constexpr double kCandyValue = 1.0;
constexpr double kSwitchRatio = 1.5;       // a new target must be this much better to take over
constexpr double kExploreHeightM = 1.5;

double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

double distance_xy(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

// How far each map cell is from the start by flying around walls, in meters. -1 means unreachable.
// Furniture counts as blocked here (the map does not know its height), so this is only a guide.
std::vector<float> travel_map(
  const nav_msgs::msg::OccupancyGrid & map, const geometry_msgs::msg::Point & start)
{
  std::vector<float> dist;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double res = map.info.resolution;

  if (width <= 0 || height <= 0 || res <= 0.0 ||
    map.data.size() != static_cast<size_t>(width) * static_cast<size_t>(height))
  {
    return dist;
  }
  dist.assign(map.data.size(), -1.0f);

  const int start_col = static_cast<int>(std::floor((start.x - map.info.origin.position.x) / res));
  const int start_row = static_cast<int>(std::floor((start.y - map.info.origin.position.y) / res));

  // The ghost may be hugging furniture: begin from the nearest free cell
  int first_col = -1;
  int first_row = -1;
  for (int ring = 0; ring <= 3 && first_col < 0; ++ring) {
    for (int dr = -ring; dr <= ring && first_col < 0; ++dr) {
      for (int dc = -ring; dc <= ring; ++dc) {
        if (std::max(std::abs(dr), std::abs(dc)) != ring) {
          continue;
        }
        const int c = start_col + dc;
        const int r = start_row + dr;
        if (c >= 0 && r >= 0 && c < width && r < height &&
          map.data[static_cast<size_t>(r) * width + c] == 0)
        {
          first_col = c;
          first_row = r;
          break;
        }
      }
    }
  }
  if (first_col < 0) {
    return dist;
  }

  std::vector<size_t> queue;
  queue.reserve(dist.size());
  const size_t first = static_cast<size_t>(first_row) * width + first_col;
  dist[first] = 0.0f;
  queue.push_back(first);

  const int col_steps[4] = {1, -1, 0, 0};
  const int row_steps[4] = {0, 0, 1, -1};

  for (size_t head = 0; head < queue.size(); ++head) {
    const size_t current = queue[head];
    const int col = static_cast<int>(current % static_cast<size_t>(width));
    const int row = static_cast<int>(current / static_cast<size_t>(width));

    for (int direction = 0; direction < 4; ++direction) {
      const int next_col = col + col_steps[direction];
      const int next_row = row + row_steps[direction];
      if (next_col < 0 || next_row < 0 || next_col >= width || next_row >= height) {
        continue;
      }
      const size_t next = static_cast<size_t>(next_row) * width + next_col;
      if (map.data[next] != 0 || dist[next] >= 0.0f) {
        continue;
      }
      dist[next] = dist[current] + static_cast<float>(res);
      queue.push_back(next);
    }
  }

  return dist;
}

// The travel distance to a point: through the cheapest free cell within 0.6 m of it.
// -1 if there is none (the point is inside furniture, or cut off)
double travel_to(
  const nav_msgs::msg::OccupancyGrid & map, const std::vector<float> & travel,
  const geometry_msgs::msg::Point & p)
{
  if (travel.empty()) {
    return -1.0;
  }
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const int col0 = static_cast<int>(std::floor((p.x - map.info.origin.position.x) / res));
  const int row0 = static_cast<int>(std::floor((p.y - map.info.origin.position.y) / res));

  double best = -1.0;
  for (int dr = -6; dr <= 6; ++dr) {
    for (int dc = -6; dc <= 6; ++dc) {
      const int c = col0 + dc;
      const int r = row0 + dr;
      if (c < 0 || r < 0 || c >= width || r >= height) {
        continue;
      }
      const float value = travel[static_cast<size_t>(r) * width + c];
      if (value < 0.0f) {
        continue;
      }
      const double offset = std::hypot(dc, dr) * res;
      if (offset > 0.6) {
        continue;
      }
      const double total = value + offset;
      if (best < 0.0 || total < best) {
        best = total;
      }
    }
  }
  return best;
}

}  // namespace

GhostBrainNode::GhostBrainNode(const rclcpp::NodeOptions & options)
: LifecycleNode("ghost_brain", options)
{
  declare_parameter<double>("control_rate_hz", 20.0);
  declare_parameter<double>("replan_period_s", 0.5);
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_link");
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_configure(const rclcpp_lifecycle::State &)
{
  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  replan_period_ = rclcpp::Duration::from_seconds(get_parameter("replan_period_s").as_double());
  map_frame_ = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  if (control_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "control_rate_hz must be positive");
    return CallbackReturn::FAILURE;
  }

  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TwistStamped;
  using haunted_house_msgs::msg::GameState;
  using nav_msgs::msg::OccupancyGrid;
  using sensor_msgs::msg::CameraInfo;
  using sensor_msgs::msg::Image;
  using sensor_msgs::msg::Imu;
  using sensor_msgs::msg::LaserScan;
  using vision_msgs::msg::Detection2DArray;

  // The house publishes these once and keeps them for anyone who joins later
  const auto latched = rclcpp::QoS(1).transient_local();
  map_sub_ = create_subscription<OccupancyGrid>("map", latched, [this](const OccupancyGrid & msg) {map_ = msg;});
  obstacles_sub_ = create_subscription<haunted_house_msgs::msg::BoxArray>(
    "haunted_house/obstacles", latched,
    [this](const haunted_house_msgs::msg::BoxArray & msg) {obstacles_ = msg;});
  start_sub_ = create_subscription<PoseStamped>("haunted_house/start_pose", latched, [this](const PoseStamped & msg) {
      if (!start_pose_) {
        localizer_.reset(msg.pose);
      }
      start_pose_ = msg;
    });

  state_sub_ = create_subscription<GameState>(
    "haunted_house/state", 10, [this](const GameState & msg) {on_state(msg);});
  velocity_sub_ = create_subscription<TwistStamped>(
    "ghost/velocity", 10, [this](const TwistStamped & msg) {localizer_.on_velocity(msg);});
  imu_sub_ = create_subscription<Imu>("ghost/imu", 10, [this](const Imu & msg) {localizer_.on_imu(msg);});
  sonar_sub_ = create_subscription<LaserScan>("sonar/scan", 10, [this](const LaserScan & msg) {
      if (map_) {
        localizer_.on_sonar(msg, *map_);
      }
    });
  depth_sub_ = create_subscription<Image>("camera/depth", 10, [this](const Image & msg) {depth_ = msg;});
  camera_info_sub_ =
    create_subscription<CameraInfo>("camera/camera_info", 10, [this](const CameraInfo & msg) {camera_info_ = msg;});
  detections_sub_ = create_subscription<Detection2DArray>(
    "camera/detections", 10, [this](const Detection2DArray & msg) {on_detections(msg);});

  cmd_vel_pub_ = create_publisher<TwistStamped>("cmd_vel", 10);
  pose_pub_ = create_publisher<PoseStamped>("ghost_brain/pose", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("ghost_brain/path", 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ghost_brain/markers", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "Configured: %.0f Hz", control_rate_hz_);
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_pub_->on_activate();
  pose_pub_->on_activate();
  path_pub_->on_activate();
  markers_pub_->on_activate();
  last_plan_time_ = now();
  progress_time_ = now();
  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / control_rate_hz_));
  timer_ = create_wall_timer(period, [this]() {update();});

  RCLCPP_INFO(get_logger(), "Activated: boo!");
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publish_command(geometry_msgs::msg::Twist{});
  cmd_vel_pub_->on_deactivate();
  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  markers_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_resources();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

GhostBrainNode::CallbackReturn GhostBrainNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_resources();
  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void GhostBrainNode::on_state(const haunted_house_msgs::msg::GameState & state)
{
  using haunted_house_msgs::msg::GameState;

  // A new game: the ghost is back at the start, and everything has moved
  const bool new_game = state.phase == GameState::WAITING && (!state_ || state_->phase != GameState::WAITING);
  if (new_game && start_pose_) {
    localizer_.reset(start_pose_->pose);
    tracker_.reset();
    release_target();
    retry_after_.clear();
    attempts_.clear();
    seen_.clear();
    explore_goal_.reset();
    explore_cell_ = -1;
    last_candies_eaten_ = 0;
    last_people_haunted_ = 0;
    have_counts_ = true;
    RCLCPP_INFO(get_logger(), "New game: back to the start");
  }

  // The score going up is the proof that something was collected
  if (state.phase != GameState::WAITING) {
    if (!have_counts_) {
      last_candies_eaten_ = state.candies_eaten;
      last_people_haunted_ = state.people_haunted;
      have_counts_ = true;
    } else {
      for (uint32_t i = last_candies_eaten_; i < state.candies_eaten; ++i) {
        on_collected(false);
      }
      for (uint32_t i = last_people_haunted_; i < state.people_haunted; ++i) {
        on_collected(true);
      }
      last_candies_eaten_ = state.candies_eaten;
      last_people_haunted_ = state.people_haunted;
    }

    // Nothing of a kind left in the house: any track of it is a phantom
    if (state.candies_left == 0) {
      tracker_.remove_all(false);
    }
    if (state.people_left == 0) {
      tracker_.remove_all(true);
    }
  }

  state_ = state;
}

void GhostBrainNode::on_collected(bool is_person)
{
  const auto here = localizer_.pose().position;
  std::optional<int> removed;

  // Most likely the one we were flying to
  if (target_id_) {
    for (const auto & tracked : tracker_.targets()) {
      if (tracked.id == *target_id_ && tracked.is_person == is_person &&
        distance(here, tracked.position) < 1.5)
      {
        removed = tracked.id;
        break;
      }
    }
    if (removed) {
      tracker_.remove_id(*removed);
    }
  }
  if (!removed) {
    removed = tracker_.remove_nearest(is_person, here, 1.5);
  }

  if (removed) {
    retry_after_.erase(*removed);
    attempts_.erase(*removed);
    if (target_id_ && *target_id_ == *removed) {
      release_target();
    }
    RCLCPP_INFO(get_logger(), "Collected a %s (track %d)", is_person ? "person" : "candy", *removed);
  } else {
    RCLCPP_INFO(get_logger(), "Collected a %s, but no track was within 1.5 m", is_person ? "person" : "candy");
  }
}

void GhostBrainNode::on_detections(const vision_msgs::msg::Detection2DArray & detections)
{
  // The depth image and camera info come from the same frame as the detections, just before them
  if (!depth_ || !camera_info_ || depth_->header.stamp != detections.header.stamp) {
    return;
  }
  tracker_.on_detections(detections, *depth_, *camera_info_, localizer_.pose(), now());
}

// Points per meter of travel. People are worth more but move, so an old sighting counts as farther
double GhostBrainNode::target_score(const TrackedTarget & target, const std::vector<float> & travel) const
{
  const auto here = localizer_.pose().position;

  double cost = travel_to(*map_, travel, target.position);
  if (cost < 0.0) {
    // Over furniture, or somewhere the map cannot say: guess from the straight line
    cost = 1.5 * distance(here, target.position) + 1.0;
  }

  double staleness = 0.0;
  if (target.is_person) {
    staleness = 0.6 * std::max(0.0, (now() - target.seen_at).seconds());
  }

  const double value = target.is_person ? kPersonValue : kCandyValue;
  return value / (cost + 1.0 + staleness);
}

std::optional<TrackedTarget> GhostBrainNode::choose_target(
  const std::vector<float> & travel, double * score_out) const
{
  const auto current_time = now();
  std::optional<TrackedTarget> best;
  double best_score = -1.0;

  for (const auto & target : tracker_.targets()) {
    const auto retry = retry_after_.find(target.id);
    if (retry != retry_after_.end() && current_time < retry->second) {
      continue;
    }

    const double score = target_score(target, travel);
    if (score > best_score) {
      best = target;
      best_score = score;
    }
  }

  if (score_out) {
    *score_out = best_score;
  }
  return best;
}

void GhostBrainNode::commit_target(const TrackedTarget & target, const geometry_msgs::msg::Point & here)
{
  target_id_ = target.id;
  planned_goal_.reset();
  arrival_time_.reset();
  explore_goal_.reset();
  explore_cell_ = -1;
  path_.clear();
  progress_position_ = here;
  progress_time_ = now();
}

void GhostBrainNode::release_target()
{
  target_id_.reset();
  planned_goal_.reset();
  arrival_time_.reset();
  path_.clear();
}

void GhostBrainNode::init_coverage()
{
  const double width_m = map_->info.width * map_->info.resolution;
  const double height_m = map_->info.height * map_->info.resolution;
  seen_cols_ = std::max(1, static_cast<int>(std::ceil(width_m / seen_cell_)));
  seen_rows_ = std::max(1, static_cast<int>(std::ceil(height_m / seen_cell_)));
  seen_.assign(static_cast<size_t>(seen_cols_) * static_cast<size_t>(seen_rows_), 0);

  // Walls and furniture need no looking at
  const double ox = map_->info.origin.position.x;
  const double oy = map_->info.origin.position.y;
  for (int row = 0; row < seen_rows_; ++row) {
    for (int col = 0; col < seen_cols_; ++col) {
      if (is_occupied(*map_, ox + (col + 0.5) * seen_cell_, oy + (row + 0.5) * seen_cell_)) {
        seen_[static_cast<size_t>(row) * seen_cols_ + col] = 1;
      }
    }
  }
}

// Mark the cells the camera can currently see: in front, in range, and not behind a wall
void GhostBrainNode::update_coverage(const geometry_msgs::msg::Point & here)
{
  const auto current_time = now();
  if (!seen_.empty() && (current_time - last_coverage_time_).seconds() < 0.25) {
    return;
  }
  last_coverage_time_ = current_time;
  if (seen_.empty()) {
    init_coverage();
  }

  constexpr double view_range = 3.5;
  constexpr double half_fov = 1.0;  // a little under the camera's 60 degrees each side

  const double yaw = yaw_of(localizer_.pose().orientation);
  const double ox = map_->info.origin.position.x;
  const double oy = map_->info.origin.position.y;

  for (int row = 0; row < seen_rows_; ++row) {
    for (int col = 0; col < seen_cols_; ++col) {
      const size_t index = static_cast<size_t>(row) * seen_cols_ + col;
      if (seen_[index]) {
        continue;
      }

      const double cell_x = ox + (col + 0.5) * seen_cell_;
      const double cell_y = oy + (row + 0.5) * seen_cell_;
      const double dx = cell_x - here.x;
      const double dy = cell_y - here.y;
      const double d = std::hypot(dx, dy);
      if (d > view_range) {
        continue;
      }
      if (d > 0.3 && std::abs(wrap_angle(std::atan2(dy, dx) - yaw)) > half_fov) {
        continue;
      }

      bool visible = true;
      const int samples = static_cast<int>(std::ceil(d / 0.2));
      for (int s = 1; s < samples; ++s) {
        const double f = static_cast<double>(s) / samples;
        if (is_occupied(*map_, here.x + dx * f, here.y + dy * f)) {
          visible = false;
          break;
        }
      }
      if (visible) {
        seen_[index] = 1;
      }
    }
  }
}

// The nearest reachable cell that has not been looked at yet (but not one right next to us)
bool GhostBrainNode::pick_exploration_goal(const std::vector<float> & travel, bool * any_unseen)
{
  *any_unseen = false;
  const double ox = map_->info.origin.position.x;
  const double oy = map_->info.origin.position.y;

  int best = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Point best_point;

  for (int row = 0; row < seen_rows_; ++row) {
    for (int col = 0; col < seen_cols_; ++col) {
      const size_t index = static_cast<size_t>(row) * seen_cols_ + col;
      if (seen_[index]) {
        continue;
      }

      const auto p = make_point(
        ox + (col + 0.5) * seen_cell_, oy + (row + 0.5) * seen_cell_, kExploreHeightM);
      const double cost = travel_to(*map_, travel, p);
      if (cost < 0.0) {
        continue;
      }

      *any_unseen = true;
      if (cost < 1.0) {
        continue;  // close ones get seen as we turn and move
      }
      if (cost < best_cost) {
        best_cost = cost;
        best = static_cast<int>(index);
        best_point = p;
      }
    }
  }

  if (best < 0) {
    return false;
  }
  explore_cell_ = best;
  explore_goal_ = best_point;
  return true;
}

void GhostBrainNode::update()
{
  using haunted_house_msgs::msg::GameState;
  if (start_pose_) {
    publish_pose();
  }
  if (!start_pose_ || !map_ || !obstacles_ || !state_ || state_->phase != GameState::RUNNING) {
    publish_command(geometry_msgs::msg::Twist{});
    return;
  }

  const auto here = localizer_.pose().position;
  const auto current_time = now();

  tracker_.drop_stale_people(current_time, kPersonMaxAgeS);
  update_coverage(here);
  const std::vector<float> travel = travel_map(*map_, here);

  // The committed target, if its track still exists
  std::optional<TrackedTarget> target;
  if (target_id_) {
    for (const auto & tracked : tracker_.targets()) {
      if (tracked.id == *target_id_) {
        target = tracked;
        break;
      }
    }
    if (!target) {
      release_target();
    }
  }

  // Pick a target, or switch to one that is much better (a person has come into view)
  const bool period_elapsed = (current_time - last_plan_time_) >= replan_period_;
  if (!target || !planned_goal_ || period_elapsed) {
    double best_score = 0.0;
    const auto best = choose_target(travel, &best_score);
    if (best) {
      if (!target) {
        commit_target(*best, here);
        target = best;
      } else if (best->id != target->id && best_score > kSwitchRatio * target_score(*target, travel)) {
        RCLCPP_INFO(get_logger(), "Switching from id=%d to id=%d", target->id, best->id);
        commit_target(*best, here);
        target = best;
      }
    }
  }

  // Stuck: barely moved for a while
  if (target || explore_goal_) {
    if (distance(here, progress_position_) > kStuckDistanceM) {
      progress_position_ = here;
      progress_time_ = current_time;
    } else if ((current_time - progress_time_).seconds() > kStuckTimeS) {
      if (target) {
        RCLCPP_WARN(get_logger(), "Stuck on id=%d: trying something else for 10 s", target->id);
        retry_after_.insert_or_assign(target->id, current_time + rclcpp::Duration::from_seconds(10.0));
        release_target();
      } else {
        RCLCPP_WARN(get_logger(), "Stuck while exploring: skipping that spot");
        if (explore_cell_ >= 0) {
          seen_[static_cast<size_t>(explore_cell_)] = 1;
        }
        explore_goal_.reset();
        explore_cell_ = -1;
        planned_goal_.reset();
        path_.clear();
      }
      progress_position_ = here;
      progress_time_ = current_time;
      publish_command(geometry_msgs::msg::Twist{});
      return;
    }
  }

  // We think we are there, but nothing scored
  if (target) {
    if (distance(here, target->position) < kArrivalRadiusM) {
      if (!arrival_time_) {
        arrival_time_ = current_time;
      } else if ((current_time - *arrival_time_).seconds() > kDwellTimeS) {
        const int id = target->id;
        const int attempts = ++attempts_[id];
        RCLCPP_WARN(get_logger(), "At id=%d by our estimate but nothing scored (attempt %d)", id, attempts);
        if (attempts >= 2) {
          tracker_.remove_id(id);
          attempts_.erase(id);
          retry_after_.erase(id);
        } else {
          retry_after_.insert_or_assign(id, current_time + rclcpp::Duration::from_seconds(6.0));
        }
        release_target();
        publish_command(geometry_msgs::msg::Twist{});
        return;
      }
    } else {
      arrival_time_.reset();
    }
  }

  // Where to go: the target, or somewhere not looked at yet
  std::optional<geometry_msgs::msg::Point> goal;
  if (target) {
    goal = target->position;
  } else {
    if (explore_goal_ && distance_xy(here, *explore_goal_) < 0.6) {
      if (explore_cell_ >= 0) {
        seen_[static_cast<size_t>(explore_cell_)] = 1;
      }
      explore_goal_.reset();
      explore_cell_ = -1;
      planned_goal_.reset();
    }

    if (!explore_goal_) {
      bool any_unseen = false;
      if (pick_exploration_goal(travel, &any_unseen)) {
        planned_goal_.reset();
        path_.clear();
        progress_position_ = here;
        progress_time_ = current_time;
      } else if (!any_unseen) {
        seen_.clear();  // looked everywhere: start another sweep
      }
    }
    goal = explore_goal_;
  }

  if (!goal) {
    // Nothing to chase, and only nearby spots left to look at: turn on the spot
    geometry_msgs::msg::Twist search_command;
    search_command.angular.z = 0.5;
    publish_command(search_command);

    path_.clear();
    planned_goal_.reset();
    publish_path();
    publish_markers(std::nullopt);
    return;
  }

  // Plan, and plan again every so often, because people keep walking
  if (!planned_goal_ || (current_time - last_plan_time_) >= replan_period_) {
    const auto plan_start = std::chrono::steady_clock::now();
    path_ = planner_.plan_3d(here, *goal, *map_, *obstacles_);
    const double plan_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - plan_start).count();

    RCLCPP_INFO(
      get_logger(),
      "Plan: %s id=%d here=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f), points=%zu plan_ms=%.1f",
      target ? "target" : "explore", target ? target->id : -1,
      here.x, here.y, here.z, goal->x, goal->y, goal->z, path_.size(), plan_ms);

    last_plan_time_ = current_time;

    if (path_.empty()) {
      // No way there: try something else now, and come back to this one later
      if (target) {
        retry_after_.insert_or_assign(target->id, current_time + rclcpp::Duration::from_seconds(5.0));
        release_target();
      } else {
        if (explore_cell_ >= 0) {
          seen_[static_cast<size_t>(explore_cell_)] = 1;
        }
        explore_goal_.reset();
        explore_cell_ = -1;
      }
      planned_goal_.reset();
      publish_path();
      publish_command(geometry_msgs::msg::Twist{});
      publish_markers(std::nullopt);
      return;
    }

    planned_goal_ = *goal;
    publish_path();
  }

  // Close to the target but not scoring: our estimate may be a little off, so head right at it
  if (target && arrival_time_) {
    path_ = Path{here, target->position};
  }

  const auto command = controller_.compute_velocity(localizer_.pose(), path_);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "ctl cmd=(%.2f, %.2f, %.2f, wz=%.2f) goal_dist=%.2f tracks=%zu",
    command.linear.x, command.linear.y, command.linear.z, command.angular.z,
    distance(here, *goal), tracker_.targets().size());
  publish_command(command);
  publish_markers(target);
}

void GhostBrainNode::publish_command(const geometry_msgs::msg::Twist & twist)
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = base_frame_;
  msg.twist = twist;
  cmd_vel_pub_->publish(msg);
}

void GhostBrainNode::publish_pose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.pose = localizer_.pose();
  pose_pub_->publish(msg);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = msg.header;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = msg.pose.position.x;
  transform.transform.translation.y = msg.pose.position.y;
  transform.transform.translation.z = msg.pose.position.z;
  transform.transform.rotation = msg.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

void GhostBrainNode::publish_path()
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  for (const auto & point : path_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  path_pub_->publish(msg);
}

void GhostBrainNode::publish_markers(const std::optional<TrackedTarget> & target)
{
  using visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray msg;
  Marker clear;
  clear.header.stamp = now();
  clear.header.frame_id = map_frame_;
  clear.action = Marker::DELETEALL;
  msg.markers.push_back(clear);

  // Everything Tracker knows about: orange for people, green for candy. Add your own decorations here!
  for (const auto & tracked : tracker_.targets()) {
    Marker dot = clear;
    dot.ns = "tracked";
    dot.id = static_cast<int>(msg.markers.size());
    dot.type = Marker::SPHERE;
    dot.action = Marker::ADD;
    dot.pose.position = tracked.position;
    dot.pose.orientation.w = 1.0;
    dot.scale.x = dot.scale.y = dot.scale.z = 0.15;
    dot.color.r = tracked.is_person ? 1.0f : 0.3f;
    dot.color.g = tracked.is_person ? 0.5f : 1.0f;
    dot.color.b = 0.0f;
    dot.color.a = 0.9f;
    msg.markers.push_back(dot);
  }

  // A ring around whatever the ghost is after
  if (target) {
    Marker ring = clear;
    ring.ns = "target";
    ring.id = static_cast<int>(msg.markers.size());
    ring.type = Marker::CYLINDER;
    ring.action = Marker::ADD;
    ring.pose.position = target->position;
    ring.pose.orientation.w = 1.0;
    ring.scale.x = ring.scale.y = 1.0;
    ring.scale.z = 0.02;
    ring.color.r = 0.4f;
    ring.color.g = 1.0f;
    ring.color.b = 0.4f;
    ring.color.a = 0.6f;
    msg.markers.push_back(ring);
  }
  markers_pub_->publish(msg);
}

void GhostBrainNode::release_resources()
{
  timer_.reset();
  velocity_sub_.reset();
  imu_sub_.reset();
  sonar_sub_.reset();
  depth_sub_.reset();
  camera_info_sub_.reset();
  detections_sub_.reset();
  map_sub_.reset();
  obstacles_sub_.reset();
  start_sub_.reset();
  state_sub_.reset();
  cmd_vel_pub_.reset();
  pose_pub_.reset();
  path_pub_.reset();
  markers_pub_.reset();
  tf_broadcaster_.reset();
  start_pose_.reset();
  map_.reset();
  obstacles_.reset();
  state_.reset();
  depth_.reset();
  camera_info_.reset();
  path_.clear();
  planned_goal_.reset();
  target_id_.reset();
  retry_after_.clear();
  attempts_.clear();
  arrival_time_.reset();
  explore_goal_.reset();
  explore_cell_ = -1;
  seen_.clear();
  have_counts_ = false;
}

}  // namespace ghost_brain

RCLCPP_COMPONENTS_REGISTER_NODE(ghost_brain::GhostBrainNode)