#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "gtest/gtest.h"

#include "fastbot_waypoints/action/waypoint.hpp"

using Waypoint = fastbot_waypoints::action::Waypoint;
using namespace std::chrono_literals;

class FastbotActionTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Waypoint>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double x_{0.0};
  double y_{0.0};
  bool odom_received_{false};

  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("fastbot_action_integration_test");

    // ODOM SUB
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          x_ = msg->pose.pose.position.x;
          y_ = msg->pose.pose.position.y;
          odom_received_ = true;
        });

    // ACTION CLIENT
    client_ = rclcpp_action::create_client<Waypoint>(node_, "/fastbot_as");

    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(FastbotActionTest, RobotReachesGoalPosition) {
  // ---------------- SEND GOAL ----------------
  Waypoint::Goal goal;
  goal.position.x = 2.0;
  goal.position.y = 1.0;

  auto goal_future = client_->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, goal_future),
            rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);

  // ---------------- TRACK ODOM ----------------
  auto start_time = std::chrono::steady_clock::now();
  bool reached = false;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);

    if (!odom_received_) {
      std::this_thread::sleep_for(50ms);
      continue;
    }

    double dx = goal.position.x - x_;
    double dy = goal.position.y - y_;
    double dist = std::sqrt(dx * dx + dy * dy);

    std::cout << "[TEST] dist = " << dist << std::endl;

    if (dist < 0.05) {
      reached = true;
      break;
    }

    if (std::chrono::steady_clock::now() - start_time > 10s) {
      break;
    }

    std::this_thread::sleep_for(50ms);
  }

  EXPECT_TRUE(reached) << "Robot hedefe 10 saniye içinde ulaşamadı";
}
