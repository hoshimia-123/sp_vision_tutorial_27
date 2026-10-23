#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>


namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

}

namespace pid_controller {

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");
  // TODO: 在此读取参数
  linear_kp_ = require_param<double>(
    node_, plugin_name_ + ".linear_kp");
  linear_ki_ = require_param<double>(
    node_, plugin_name_ + ".linear_ki");
  linear_kd_ = require_param<double>(
    node_, plugin_name_ + ".linear_kd");

  angular_kp_ = require_param<double>(
    node_, plugin_name_ + ".angular_kp");
  angular_ki_ = require_param<double>(
    node_, plugin_name_ + ".angular_ki");
  angular_kd_ = require_param<double>(
    node_, plugin_name_ + ".angular_kd");

  max_linear_speed_ = require_param<double>(
    node_, plugin_name_ + ".max_linear_speed");
  max_angular_speed_ = require_param<double>(
    node_, plugin_name_ + ".max_angular_speed");
  lookahead_distance_ = require_param<double>(
    node_, plugin_name_ + ".lookahead_distance");
  goal_tolerance_ = require_param<double>(
    node_, plugin_name_ + ".goal_tolerance");

  RCLCPP_INFO(
    node_->get_logger(),
    "PID controller configured: max_linear=%.2f, max_angular=%.2f",
    max_linear_speed_, max_angular_speed_);
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  linear_integral_ = 0.0;
  angular_integral_ = 0.0;
  previous_distance_error_ = 0.0;
  previous_heading_error_ = 0.0;
  has_previous_error_ = false;
  previous_time_ = std::chrono::steady_clock::now();
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &)
{
  // TODO：在此实现路径跟踪
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;


  const auto & points = global_plan_.poses;

  // 没有路径，就不发送运动指令
  if (points.empty()) {
    return cmd_vel;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;

  // 找到距离机器人最近的路径点
  size_t nearest = 0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < points.size(); ++i) {
    const double dx = points[i].pose.position.x - x;
    const double dy = points[i].pose.position.y - y;
    const double d = std::hypot(dx, dy);

    if (d < min_distance) {
      min_distance = d;
      nearest = i;
    }
  }

  // 到达终点后停车
  const auto & goal = points.back().pose.position;
  if (std::hypot(goal.x - x, goal.y - y) <= goal_tolerance_) {
    return cmd_vel;
  }

  // 选择前方的目标路径点
  size_t target_index = nearest;

  while (target_index + 1 < points.size()) {
    const auto & p = points[target_index].pose.position;

    if (std::hypot(p.x - x, p.y - y) >= lookahead_distance_) {
      break;
    }

    ++target_index;
  }

  const auto & target = points[target_index].pose.position;

  const double dx = target.x - x;
  const double dy = target.y - y;
  const double distance_error = std::hypot(dx, dy);

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double target_yaw = std::atan2(dy, dx);
  const double heading_error = std::atan2(
    std::sin(target_yaw - yaw),
    std::cos(target_yaw - yaw));

  // 计算控制周期
  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(
    now - previous_time_).count();

  if (dt <= 1e-4 || dt > 0.5) {
    dt = 0.02;
  }

  double distance_derivative = 0.0;
  double heading_derivative = 0.0;

  if (has_previous_error_) {
    distance_derivative =
      (distance_error - previous_distance_error_) / dt;

    heading_derivative =
      std::atan2(
        std::sin(heading_error - previous_heading_error_),
        std::cos(heading_error - previous_heading_error_)) / dt;
  }

  linear_integral_ = std::clamp(
    linear_integral_ + distance_error * dt, -2.0, 2.0);

  angular_integral_ = std::clamp(
    angular_integral_ + heading_error * dt, -1.0, 1.0);

  // PID 计算
  double linear_speed =
    linear_kp_ * distance_error +
    linear_ki_ * linear_integral_ +
    linear_kd_ * distance_derivative;

  double angular_speed =
    angular_kp_ * heading_error +
    angular_ki_ * angular_integral_ +
    angular_kd_ * heading_derivative;

  // 转向角度较大时，降低前进速度
  linear_speed *= std::max(0.0, std::cos(heading_error));

  cmd_vel.twist.linear.x =
    std::clamp(linear_speed, 0.0, max_linear_speed_);

  cmd_vel.twist.angular.z =
    std::clamp(
      angular_speed, -max_angular_speed_, max_angular_speed_);

  previous_distance_error_ = distance_error;
  previous_heading_error_ = heading_error;
  previous_time_ = now;
  has_previous_error_ = true;
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
