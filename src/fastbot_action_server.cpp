#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "fastbot_waypoints/action/waypoint.hpp"

using namespace std::chrono_literals;

class FastbotActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  FastbotActionServer() : Node("fastbot_action_server") {
    cmd_vel_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "fastbot/cmd_vel");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/fastbot/odom");
    yaw_precision_ =
        declare_parameter<double>("yaw_precision", M_PI / 90.0);         // ~2°
    dist_precision_ = declare_parameter<double>("dist_precision", 0.10); // 5 cm

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 50,
        std::bind(&FastbotActionServer::odomCb, this, std::placeholders::_1));

    server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&FastbotActionServer::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&FastbotActionServer::handleCancel, this,
                  std::placeholders::_1),
        std::bind(&FastbotActionServer::handleAccepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "fastbot_action_server started.");
  }

private:
  // State
  geometry_msgs::msg::Point position_;
  double yaw_{0.0};
  std::string state_{"idle"};

  // ROS I/O
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_action::Server<Waypoint>::SharedPtr server_;

  // Params
  std::string cmd_vel_topic_, odom_topic_;
  double yaw_precision_, dist_precision_;

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    yaw_ = yaw;
  }

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const Waypoint::Goal> goal) {
    (void)goal;
    RCLCPP_INFO(get_logger(), "Goal received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Goal cancel requested");
    stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{
        std::bind(&FastbotActionServer::execute, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }

  void stop() {
    geometry_msgs::msg::Twist z;
    z.linear.x = 0.0;
    z.angular.z = 0.0;
    pub_cmd_->publish(z);
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "Executing to (%.3f, %.3f)", goal->position.x,
                goal->position.y);

    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    rclcpp::Rate rate(25.0);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(get_logger(), "Canceled");
        stop();
        goal_handle->canceled(result);
        return;
      }

      const double dx = goal->position.x - position_.x;
      const double dy = goal->position.y - position_.y;
      const double desired_yaw = std::atan2(dy, dx);
      const double err_pos = std::hypot(dx, dy);
      double err_yaw = desired_yaw - yaw_;
      err_yaw = std::atan2(std::sin(err_yaw),
                           std::cos(err_yaw)); // normalize [-pi,pi]

      geometry_msgs::msg::Twist cmd;

      if (err_pos <= dist_precision_) {
        state_ = "idle";
        stop();
        break;
      } else if (std::fabs(err_yaw) > yaw_precision_) {
        state_ = "fix yaw";
        cmd.angular.z = (err_yaw > 0.0) ? 0.65 : -0.65;
      } else {
        state_ = "go to point";
        cmd.linear.x = 0.6;
        cmd.angular.z = 0.0;
      }

      pub_cmd_->publish(cmd);

      feedback->current = position_;
      feedback->state = state_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stop();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastbotActionServer>());
  rclcpp::shutdown();
  return 0;
}
