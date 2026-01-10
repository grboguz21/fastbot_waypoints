#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "fastbot_waypoints/action/waypoint.hpp"

class FastbotActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  FastbotActionServer() : Node("fastbot_action_server") {

    pub_cmd_vel_ =
        create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 10);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&FastbotActionServer::odomCallback, this,
                  std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&FastbotActionServer::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&FastbotActionServer::handleCancel, this,
                  std::placeholders::_1),
        std::bind(&FastbotActionServer::handleAccepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "[ACTION SERVER] Started");
  }

private:
  // ---------------- ROBOT STATE ----------------
  geometry_msgs::msg::Point position_;
  double yaw_{0.0};

  std::string state_{"idle"};
  std::string last_state_{"none"};

  // ---------------- PRECISION ----------------
  const double yaw_precision_ = M_PI / 90.0; // ~2°
  const double dist_precision_ = 0.05;

  // ---------------- ROS INTERFACES ----------------
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;

  // ---------------- ODOM CALLBACK ----------------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
  }

  // ---------------- ACTION HANDLERS ----------------
  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const Waypoint::Goal> goal) {

    RCLCPP_INFO(get_logger(), "[ACTION] Goal RECEIVED -> x: %.3f y: %.3f",
                goal->position.x, goal->position.y);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleWaypoint>) {

    RCLCPP_WARN(get_logger(), "[ACTION] Goal CANCELED");
    stopRobot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    std::thread{std::bind(&FastbotActionServer::execute, this, goal_handle)}
        .detach();
  }

  // ---------------- EXECUTION ----------------
  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {

    RCLCPP_INFO(get_logger(), "[ACTION] Execution STARTED");

    rclcpp::Rate rate(25.0);

    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    while (rclcpp::ok()) {

      if (goal_handle->is_canceling()) {
        stopRobot();
        result->success = false;
        goal_handle->canceled(result);
        return;
      }

      double dx = goal_handle->get_goal()->position.x - position_.x;
      double dy = goal_handle->get_goal()->position.y - position_.y;

      double desired_yaw = std::atan2(dy, dx);
      double err_yaw = desired_yaw - yaw_;
      double err_pos = std::sqrt(
          std::pow(goal_handle->get_goal()->position.y - position_.y, 2) +
          std::pow(goal_handle->get_goal()->position.x - position_.x, 2));

      geometry_msgs::msg::Twist cmd;

      if (std::fabs(err_yaw) > yaw_precision_) {
        state_ = "fix yaw";
        cmd.angular.z = (err_yaw > 0) ? 0.65 : -0.65;

      } else if (err_pos > dist_precision_) {
        state_ = "go to point";
        cmd.linear.x = 0.6;

      } else {
        state_ = "goal reached";
        break;
      }

      // -------- STATE LOG (ONLY WHEN CHANGED) --------
      if (state_ != last_state_) {
        RCLCPP_INFO(get_logger(), "[STATE] %s | err_pos: %.3f err_yaw: %.3f",
                    state_.c_str(), err_pos, err_yaw);
        last_state_ = state_;
      }

      pub_cmd_vel_->publish(cmd);

      // -------- ACTION FEEDBACK --------
      feedback->current = position_;
      feedback->state = state_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stopRobot();

    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(get_logger(), "[ACTION] Goal SUCCEEDED -> x: %.3f y: %.3f",
                position_.x, position_.y);
  }

  // ---------------- STOP ROBOT ----------------
  void stopRobot() {
    geometry_msgs::msg::Twist stop;
    pub_cmd_vel_->publish(stop);
  }
};

// ---------------- MAIN ----------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastbotActionServer>());
  rclcpp::shutdown();
  return 0;
}
