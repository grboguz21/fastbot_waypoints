#include "fastbot_waypoints/action/waypoint.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>

using Waypoint = fastbot_waypoints::action::Waypoint;

class FastbotActionTest : public ::testing::Test {
protected:
  static inline bool goal_sent_ = false;

  // Goal & result
  static inline std::shared_ptr<Waypoint::Goal> goal_;
  static inline double goal_yaw_ = 0.0;
  static inline rclcpp_action::ClientGoalHandle<Waypoint>::WrappedResult
      wrapped_result_;

  // Robot state (from odom)
  static inline double last_x_ = 0.0;
  static inline double last_y_ = 0.0;
  static inline double last_yaw_ = 0.0;

  // ROS interfaces
  static inline rclcpp::Node::SharedPtr node_;
  static inline rclcpp_action::Client<Waypoint>::SharedPtr client_;
  static inline rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      sub_odom_;

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("fastbot_action_test");

    // ---------------- ODOM SUB ----------------
    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10, [](const nav_msgs::msg::Odometry::SharedPtr msg) {
          last_x_ = msg->pose.pose.position.x;
          last_y_ = msg->pose.pose.position.y;

          const auto &q = msg->pose.pose.orientation;
          double siny = 2.0 * (q.w * q.z + q.x * q.y);
          double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
          last_yaw_ = std::atan2(siny, cosy);
        });

    // ---------------- ACTION CLIENT ----------------
    client_ = rclcpp_action::create_client<Waypoint>(node_, "fastbot_as");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));

    // ---------------- SEND GOAL (ONCE) ----------------
    goal_ = std::make_shared<Waypoint::Goal>();

    // // failling
    // goal_->position.x = 1.8;
    // goal_->position.y = 1.2;

    // passing
    goal_->position.x = 1.4;
    goal_->position.y = 1.1;

    auto goal_future = client_->async_send_goal(*goal_);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, goal_future),
              rclcpp::FutureReturnCode::SUCCESS);

    auto goal_handle = goal_future.get();
    ASSERT_TRUE(goal_handle);

    auto result_future = client_->async_get_result(goal_handle);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node_, result_future),
              rclcpp::FutureReturnCode::SUCCESS);

    wrapped_result_ = result_future.get();
    ASSERT_EQ(wrapped_result_.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_TRUE(wrapped_result_.result->success);

    // ensure odom callback processed
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    rclcpp::spin_some(node_);

    goal_sent_ = true;
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static double normalizeYaw(double yaw) {
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    return yaw;
  }
};

// ============================
// POSITION TEST
// ============================
TEST_F(FastbotActionTest, RobotReachesCorrectPosition) {
  ASSERT_TRUE(goal_sent_);

  double dx = last_x_ - goal_->position.x;
  double dy = last_y_ - goal_->position.y;
  double error = std::sqrt(dx * dx + dy * dy);
  //   double error_ = 0.02;

  std::cout << "[TEST][POS] error = " << error << std::endl;
  EXPECT_LT(error, 0.048);
}

// ============================
// YAW TEST (SERVER LOGIC)
// ============================
TEST_F(FastbotActionTest, RobotReachesCorrectYaw) {
  ASSERT_TRUE(goal_sent_);
  ASSERT_TRUE(wrapped_result_.result->success);

  double dx = goal_->position.x - last_x_;
  double dy = goal_->position.y - last_y_;

  double desired_yaw = std::atan2(dy, dx);
  double yaw_error = normalizeYaw(desired_yaw - last_yaw_);
  //   double yaw_error_ = 0.03;

  std::cout << "[TEST][YAW] "
            << "desired=" << desired_yaw << " actual=" << last_yaw_
            << " error=" << yaw_error << std::endl;

  EXPECT_LT(std::fabs(yaw_error), 0.06);
}
