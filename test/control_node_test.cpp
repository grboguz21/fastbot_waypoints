#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "gtest/gtest.h"

#include "fastbot_waypoints/action/waypoint.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using Waypoint = fastbot_waypoints::action::Waypoint;
using namespace std::chrono_literals;

class FastbotActionTest : public ::testing::Test {
protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Waypoint>::SharedPtr client_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  std::atomic<double> x_{0.0};
  std::atomic<double> y_{0.0};
  std::atomic<double> yaw_{0.0};

  std::atomic<bool> odom_received_{false};
  std::atomic<bool> scan_received_{false};
  std::atomic<float> min_scan_{std::numeric_limits<float>::max()};

  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("fastbot_action_test");

    // ===== ODOM =====
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          x_ = msg->pose.pose.position.x;
          y_ = msg->pose.pose.position.y;

          tf2::Quaternion q(
              msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

          double roll, pitch, yaw_tmp;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_tmp);
          yaw_ = yaw_tmp;

          odom_received_ = true;
        });

    // ===== LASER =====
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot/scan", rclcpp::QoS(10).best_effort(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          float min_d = std::numeric_limits<float>::max();
          for (float r : msg->ranges) {
            if (r > 0.0f)
              min_d = std::min(min_d, r);
          }
          min_scan_ = min_d;
          scan_received_ = true;
        });

    // ===== ACTION CLIENT =====
    client_ = rclcpp_action::create_client<Waypoint>(node_, "/fastbot_as");
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override { rclcpp::shutdown(); }

  // ===== Utils =====
  double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  double yawError(double goal_x, double goal_y) {
    double dx = goal_x - x_;
    double dy = goal_y - y_;
    double desired = std::atan2(dy, dx);
    double yaw_global = normalize(yaw_ - M_PI / 2.0);
    return normalize(desired - yaw_global);
  }
};

// ==========================================================
// =============== DIST + YAW + SCAN TEST ===================
// ==========================================================
TEST_F(FastbotActionTest, ReachGoalSafelyWithCorrectYaw) {
  Waypoint::Goal goal;
  goal.position.x = 2.0;
  goal.position.y = 2.0;

  auto future = client_->async_send_goal(goal);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node_, future),
            rclcpp::FutureReturnCode::SUCCESS);

  ASSERT_TRUE(future.get());

  auto start = std::chrono::steady_clock::now();
  bool success = false;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);

    if (!odom_received_ || !scan_received_) {
      std::this_thread::sleep_for(50ms);
      continue;
    }

    double dx = goal.position.x - x_;
    double dy = goal.position.y - y_;
    double dist = std::sqrt(dx * dx + dy * dy);
    double yaw_err = yawError(goal.position.x, goal.position.y);
    float scan = min_scan_;

    std::cout << "[TEST] dist=" << dist << " yaw_err=" << yaw_err
              << " min_scan=" << scan << std::endl;

    // ❌ HERHANGİ BİR ANDA ÇARPIŞMA RİSKİ VARSA FAIL
    ASSERT_GE(scan, 0.2) << "Çarpışma riski! LaserScan < 0.2 m";

    // ✅ SADECE ÜÇÜ DE AYNI ANDA SAĞLANIRSA PASS
    if (dist < 0.07 && std::fabs(yaw_err) < 0.05) {
      success = true;
      break;
    }

    if (std::chrono::steady_clock::now() - start > 40s)
      break;

    std::this_thread::sleep_for(50ms);
  }

  EXPECT_TRUE(success) << "Robot hedefe güvenli ve doğru yaw ile ulaşamadı";
}
