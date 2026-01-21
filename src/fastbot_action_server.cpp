#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "fastbot_waypoints/action/waypoint.hpp"

class FastbotActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  FastbotActionServer() : Node("fastbot_as") {
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&FastbotActionServer::odomCallback, this,
                  std::placeholders::_1));

    pub_cmd_vel_ =
        create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&FastbotActionServer::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&FastbotActionServer::handleCancel, this,
                  std::placeholders::_1),
        std::bind(&FastbotActionServer::handleAccepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "✅ Fastbot Action Server started");
  }

private:
  // ===== Robot state =====
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  // ===== PID (angular) =====
  double kp_ = 2.0;
  double ki_ = 0.0;
  double kd_ = 0.2;

  double err_i_ = 0.0;
  double prev_err_ = 0.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;

  // ===== Utils =====
  double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
  }

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const Waypoint::Goal> goal) {

    RCLCPP_INFO(get_logger(), "[GOAL] x=%.3f y=%.3f", goal->position.x,
                goal->position.y);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleWaypoint>) {
    stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    rclcpp::Rate rate(30);
    auto result = std::make_shared<Waypoint::Result>();
    const auto &goal = goal_handle->get_goal()->position;

    // ================= PHASE 1: ROTATE =================
    while (rclcpp::ok()) {
      double dx = goal.x - x_;
      double dy = goal.y - y_;

      double desired = std::atan2(dy, dx);
      double yaw_global = normalize(yaw_ - M_PI / 2.0);
      double err = normalize(desired - yaw_global);

      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = std::clamp(2.0 * err, -0.8, 0.8);
      pub_cmd_vel_->publish(cmd);

      RCLCPP_INFO(get_logger(), "[ROTATE] err=%.2f deg", err * 180.0 / M_PI);

      if (std::fabs(err) < 0.0200) {
        stop();
        break;
      }

      rate.sleep();
    }

    // PID reset
    err_i_ = 0.0;
    prev_err_ = 0.0;

    // ================= PHASE 2: MOVE + PID =================
    while (rclcpp::ok()) {
      double dx = goal.x - x_;
      double dy = goal.y - y_;
      double dist = std::sqrt(dx * dx + dy * dy);

      double desired = std::atan2(dy, dx);
      double yaw_global = normalize(yaw_ - M_PI / 2.0);
      double err = normalize(desired - yaw_global);

      // --- PID ---
      err_i_ += err;
      double err_d = err - prev_err_;
      prev_err_ = err;

      double w = kp_ * err + ki_ * err_i_ + kd_ * err_d;
      w = std::clamp(w, -0.6, 0.6);

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = std::clamp(0.8 * dist, 0.0, 0.35);
      cmd.angular.z = w;
      pub_cmd_vel_->publish(cmd);

      RCLCPP_INFO(get_logger(), "[MOVE] dist=%.3f err=%.2f deg", dist,
                  err * 180.0 / M_PI);

      if (dist < 0.05) {
        stop();
        break;
      }

      rate.sleep();
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "✅ Goal reached");
  }

  void stop() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub_cmd_vel_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastbotActionServer>());
  rclcpp::shutdown();
  return 0;
}
