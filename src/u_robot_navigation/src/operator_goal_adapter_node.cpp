#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace u_robot_navigation
{
namespace
{
diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = std::move(key);
  result.value = std::move(value);
  return result;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}
}  // namespace

class OperatorGoalAdapterNode final : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  OperatorGoalAdapterNode()
  : Node("operator_goal_adapter"),
    goal_topic_(declare_parameter<std::string>("goal_topic", "/operator/goal_pose")),
    standard_goal_topic_(
      declare_parameter<std::string>("standard_goal_topic", "/move_base_simple/goal")),
    cancel_topic_(
      declare_parameter<std::string>("cancel_topic", "/operator/cancel_navigation")),
    patrol_active_topic_(
      declare_parameter<std::string>("patrol_active_topic", "/waypoint_patrol/active")),
    map_topic_(declare_parameter<std::string>("map_topic", "/map")),
    pose_topic_(declare_parameter<std::string>("pose_topic", "/amcl_pose")),
    obstacle_topic_(
      declare_parameter<std::string>("obstacle_topic", "/navigation/obstacles")),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    obstacle_timeout_(positive_parameter("obstacle_timeout", 0.50)),
    transform_tolerance_(positive_parameter("transform_tolerance", 0.10)),
    position_stddev_limit_(positive_parameter("position_stddev_limit", 0.75)),
    yaw_stddev_limit_(positive_parameter("yaw_stddev_limit", 0.75)),
    occupied_threshold_(declare_parameter<std::int64_t>("occupied_threshold", 50)),
    goal_clearance_(positive_parameter("goal_clearance", 0.55)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    if (occupied_threshold_ < 1 || occupied_threshold_ > 100) {
      throw std::invalid_argument("occupied_threshold must be in [1, 100]");
    }
    if (goal_topic_.empty() || standard_goal_topic_.empty() ||
      goal_topic_ == standard_goal_topic_)
    {
      throw std::invalid_argument("goal topics must be non-empty and distinct");
    }

    navigate_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {map_ = std::move(message);});
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message) {
        pose_seen_ = true;
        last_pose_time_ = std::chrono::steady_clock::now();
        const auto & covariance = message->pose.covariance;
        position_stddev_ = std::sqrt(std::max(0.0, std::max(covariance[0], covariance[7])));
        yaw_stddev_ = std::sqrt(std::max(0.0, covariance[35]));
      });
    obstacle_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        obstacle_seen_ = true;
        last_obstacle_time_ = std::chrono::steady_clock::now();
        last_obstacle_points_ = static_cast<std::uint64_t>(message->width) * message->height;
        obstacle_cloud_valid_ = message->header.frame_id == "base_link";
      });
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {on_goal(*message);});
    standard_goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      standard_goal_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {on_goal(*message);});
    cancel_subscription_ = create_subscription<std_msgs::msg::Empty>(
      cancel_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const std_msgs::msg::Empty::SharedPtr) {
        cancel_request_count_++;
        navigate_client_->async_cancel_all_goals();
        RCLCPP_WARN(get_logger(), "Operator requested cancellation of all navigation goals");
      });
    patrol_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      patrol_active_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        patrol_active_ = message->data;
        if (patrol_active_ && goal_busy_ && active_goal_handle_) {
          navigate_client_->async_cancel_goal(active_goal_handle_);
          RCLCPP_WARN(
            get_logger(), "Patrol started; cancelling the active manual navigation goal");
        }
      });
    diagnostic_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(),
      "Operator goal gate ready on %s and %s; accepted goals are sent to NavigateToPose",
      goal_topic_.c_str(), standard_goal_topic_.c_str());
  }

private:
  double positive_parameter(const std::string & name, const double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  bool fresh(
    const bool seen, const std::chrono::steady_clock::time_point & timestamp,
    const double timeout) const
  {
    return seen && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - timestamp).count() <= timeout;
  }

  std::optional<std::string> readiness_failure()
  {
    if (!map_) {
      return "map has not been received";
    }
    if (map_->info.resolution <= 0.0F || map_->info.width == 0U || map_->info.height == 0U ||
      map_->data.size() != static_cast<std::size_t>(map_->info.width) * map_->info.height)
    {
      return "map metadata or data is invalid";
    }
    if (!pose_seen_) {
      return "AMCL pose is missing; set and verify initial pose";
    }
    if (!std::isfinite(position_stddev_) || !std::isfinite(yaw_stddev_) ||
      position_stddev_ > position_stddev_limit_ || yaw_stddev_ > yaw_stddev_limit_)
    {
      return "AMCL covariance is above the goal-acceptance limit";
    }
    if (!fresh(obstacle_seen_, last_obstacle_time_, obstacle_timeout_)) {
      return "filtered obstacle cloud is missing or stale";
    }
    if (!obstacle_cloud_valid_) {
      return "filtered obstacle cloud has an unexpected frame or layout";
    }
    if (!tf_buffer_.canTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance_)))
    {
      return "current map-to-base transform is unavailable";
    }
    if (!navigate_client_->action_server_is_ready()) {
      return "NavigateToPose action server is not ready";
    }
    if (patrol_active_) {
      return "waypoint patrol is active; stop it before publishing a manual goal";
    }
    if (goal_busy_) {
      return "a navigation goal is already active";
    }
    return std::nullopt;
  }

  std::optional<std::string> validate_goal(const geometry_msgs::msg::PoseStamped & goal) const
  {
    const std::string frame = !goal.header.frame_id.empty() && goal.header.frame_id.front() == '/' ?
      goal.header.frame_id.substr(1) : goal.header.frame_id;
    if (frame != map_frame_) {
      return "goal frame must be '" + map_frame_ + "'";
    }
    const auto & position = goal.pose.position;
    const auto & orientation = goal.pose.orientation;
    const double quaternion_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(quaternion_norm) ||
      quaternion_norm < 0.90 || quaternion_norm > 1.10)
    {
      return "goal pose is non-finite or its quaternion is not normalized";
    }
    if (!goal_area_is_free(position.x, position.y)) {
      return "goal is outside the map, unknown, occupied, or lacks clearance";
    }
    return std::nullopt;
  }

  bool goal_area_is_free(const double map_x, const double map_y) const
  {
    const auto & info = map_->info;
    const double origin_yaw = yaw_from_quaternion(info.origin.orientation);
    const double dx = map_x - info.origin.position.x;
    const double dy = map_y - info.origin.position.y;
    const double local_x = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
    const double local_y = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
    const auto center_x = static_cast<std::int64_t>(std::floor(local_x / info.resolution));
    const auto center_y = static_cast<std::int64_t>(std::floor(local_y / info.resolution));
    const auto radius_cells = static_cast<std::int64_t>(
      std::ceil(goal_clearance_ / static_cast<double>(info.resolution)));

    for (std::int64_t y = center_y - radius_cells; y <= center_y + radius_cells; ++y) {
      for (std::int64_t x = center_x - radius_cells; x <= center_x + radius_cells; ++x) {
        const double cell_dx = static_cast<double>(x - center_x) * info.resolution;
        const double cell_dy = static_cast<double>(y - center_y) * info.resolution;
        if (cell_dx * cell_dx + cell_dy * cell_dy > goal_clearance_ * goal_clearance_) {
          continue;
        }
        if (x < 0 || y < 0 || x >= static_cast<std::int64_t>(info.width) ||
          y >= static_cast<std::int64_t>(info.height))
        {
          return false;
        }
        const auto index = static_cast<std::size_t>(y) * info.width +
          static_cast<std::size_t>(x);
        const std::int8_t occupancy = map_->data[index];
        if (occupancy < 0 || occupancy >= occupied_threshold_) {
          return false;
        }
      }
    }
    return true;
  }

  void reject(const std::string & reason)
  {
    rejected_goal_count_++;
    last_rejection_ = reason;
    RCLCPP_WARN(get_logger(), "Rejected operator goal: %s", reason.c_str());
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & message)
  {
    if (const auto failure = readiness_failure()) {
      reject(*failure);
      return;
    }
    if (const auto failure = validate_goal(message)) {
      reject(*failure);
      return;
    }

    NavigateToPose::Goal action_goal;
    action_goal.pose = message;
    action_goal.pose.header.frame_id = map_frame_;
    action_goal.pose.header.stamp = now();
    goal_busy_ = true;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr handle) {
        if (!handle) {
          goal_busy_ = false;
          reject("NavigateToPose server rejected the goal");
          return;
        }
        active_goal_handle_ = handle;
        accepted_goal_count_++;
        last_rejection_.clear();
        RCLCPP_INFO(get_logger(), "Operator navigation goal accepted by Nav2");
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        goal_busy_ = false;
        active_goal_handle_.reset();
        last_result_code_ = static_cast<std::int8_t>(result.code);
        RCLCPP_INFO(
          get_logger(), "Operator navigation goal finished with result code %d",
          static_cast<int>(last_result_code_));
      };
    navigate_client_->async_send_goal(action_goal, options);
  }

  void publish_diagnostics()
  {
    const auto failure = readiness_failure();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "u_robot/operator_goal_gate";
    status.hardware_id = "unitree_a2";
    status.level = failure.has_value() && !goal_busy_ ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = goal_busy_ ? "navigation goal active" :
      (failure.has_value() ? *failure : "ready to accept an operator navigation goal");
    status.values.push_back(key_value("goal_topic", goal_topic_));
    status.values.push_back(key_value("standard_goal_topic", standard_goal_topic_));
    status.values.push_back(key_value("cancel_topic", cancel_topic_));
    status.values.push_back(key_value("patrol_active", patrol_active_ ? "true" : "false"));
    status.values.push_back(key_value("map_received", map_ ? "true" : "false"));
    status.values.push_back(key_value("pose_received", pose_seen_ ? "true" : "false"));
    status.values.push_back(key_value(
      "obstacles_fresh",
      fresh(obstacle_seen_, last_obstacle_time_, obstacle_timeout_) ? "true" : "false"));
    status.values.push_back(key_value("obstacle_points", std::to_string(last_obstacle_points_)));
    status.values.push_back(key_value("position_stddev", std::to_string(position_stddev_)));
    status.values.push_back(key_value("yaw_stddev", std::to_string(yaw_stddev_)));
    status.values.push_back(key_value("goal_active", goal_busy_ ? "true" : "false"));
    status.values.push_back(key_value("accepted_goals", std::to_string(accepted_goal_count_)));
    status.values.push_back(key_value("rejected_goals", std::to_string(rejected_goal_count_)));
    status.values.push_back(key_value("cancel_requests", std::to_string(cancel_request_count_)));
    status.values.push_back(key_value("last_rejection", last_rejection_));
    status.values.push_back(key_value("last_result_code", std::to_string(last_result_code_)));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  const std::string goal_topic_;
  const std::string standard_goal_topic_;
  const std::string cancel_topic_;
  const std::string patrol_active_topic_;
  const std::string map_topic_;
  const std::string pose_topic_;
  const std::string obstacle_topic_;
  const std::string map_frame_;
  const std::string base_frame_;
  const double obstacle_timeout_;
  const double transform_tolerance_;
  const double position_stddev_limit_;
  const double yaw_stddev_limit_;
  const std::int64_t occupied_threshold_;
  const double goal_clearance_;

  bool pose_seen_{false};
  bool obstacle_seen_{false};
  bool obstacle_cloud_valid_{false};
  bool goal_busy_{false};
  bool patrol_active_{false};
  double position_stddev_{std::numeric_limits<double>::infinity()};
  double yaw_stddev_{std::numeric_limits<double>::infinity()};
  std::chrono::steady_clock::time_point last_pose_time_{};
  std::chrono::steady_clock::time_point last_obstacle_time_{};
  std::uint64_t last_obstacle_points_{0};
  std::uint64_t accepted_goal_count_{0};
  std::uint64_t rejected_goal_count_{0};
  std::uint64_t cancel_request_count_{0};
  std::int8_t last_result_code_{-1};
  std::string last_rejection_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  GoalHandle::SharedPtr active_goal_handle_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    standard_goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr patrol_active_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};
}  // namespace u_robot_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<u_robot_navigation::OperatorGoalAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
