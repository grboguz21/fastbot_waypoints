#!/usr/bin/env python3
import argparse
import sys
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from fastbot_waypoints.action import Waypoint

# -----------------------------
# Quaternion → Yaw dönüşümü
# -----------------------------
try:
    from tf_transformations import euler_from_quaternion
except Exception:
    def euler_from_quaternion(q):
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (0.0, 0.0, yaw)


def normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


# ====================================================
# 🚀 Waypoint Action Tester
# ====================================================
class WaypointActionTester(Node):
    def __init__(
        self,
        action_name: str,
        goal_x: float,
        goal_y: float,
        cmd_vel_topic: str,
        odom_topic: str,
        expect_failure: bool,
        timeout_sec: float,
        pos_margin: float,
        yaw_tol_deg: float,
        skip_pos_check: bool,
        skip_yaw_check: bool,
    ):
        super().__init__("waypoint_action_tester")
        self.action_name = action_name
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.expect_failure = expect_failure
        self.timeout_sec = timeout_sec
        self.pos_margin = pos_margin
        self.yaw_tol = math.radians(yaw_tol_deg)
        self.skip_pos_check = skip_pos_check
        self.skip_yaw_check = skip_yaw_check

        # --- motion and odom ---
        self.got_motion = False
        self.last_twist = Twist()
        self.have_odom = False
        self.last_pos = Point()
        self.last_yaw = 0.0

        # Subscribers
        self.create_subscription(Twist, cmd_vel_topic, self._twist_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)

        # Action client
        self.client = ActionClient(self, Waypoint, self.action_name)

    # ------------------ CALLBACKS ------------------
    def _twist_cb(self, msg: Twist):
        self.last_twist = msg
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.got_motion = True

    def _odom_cb(self, msg: Odometry):
        self.last_pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.last_yaw = normalize_angle(yaw)
        self.have_odom = True

    # ------------------ HELPERS ------------------
    def wait_action_server(self, timeout_sec=10.0) -> bool:
        self.get_logger().info(f"⏳ Waiting for action server [{self.action_name}] ...")
        return self.client.wait_for_server(timeout_sec=timeout_sec)

    def _check_end_position(self) -> bool:
        if self.skip_pos_check:
            self.get_logger().info("Position check skipped.")
            return True

        dx = self.goal_x - self.last_pos.x
        dy = self.goal_y - self.last_pos.y
        dist_err = math.hypot(dx, dy)

        ok = dist_err <= self.pos_margin
        if ok:
            self.get_logger().info(
                f"✅ Position OK: error={dist_err:.3f} m ≤ margin={self.pos_margin:.3f} m"
            )
        else:
            self.get_logger().error(
                f"❌ Position FAIL: error={dist_err:.3f} m > margin={self.pos_margin:.3f} m"
            )
        return ok

    def _check_end_yaw(self) -> bool:
        if self.skip_yaw_check:
            self.get_logger().info("Yaw check skipped.")
            return True

        desired_yaw = math.atan2(self.goal_y - self.last_pos.y, self.goal_x - self.last_pos.x)
        err = abs(normalize_angle(desired_yaw - self.last_yaw))

        ok = err <= self.yaw_tol
        if ok:
            self.get_logger().info(
                f"✅ Yaw OK: {math.degrees(err):.2f}° ≤ ±{math.degrees(self.yaw_tol):.2f}°"
            )
        else:
            self.get_logger().error(
                f"❌ Yaw FAIL: {math.degrees(err):.2f}° > ±{math.degrees(self.yaw_tol):.2f}°"
            )
        return ok

    # ------------------ MAIN TEST LOGIC ------------------
    def run(self) -> bool:
        # Wait for odom messages
        t0 = time.time()
        while rclpy.ok() and not self.have_odom and (time.time() - t0) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.have_odom:
            self.get_logger().error("⚠️ No /odom received before sending goal.")

        # Wait for server
        if not self.wait_action_server(10.0):
            self.get_logger().error("❌ Action server not available.")
            return not self.expect_failure

        # Send goal
        goal_msg = Waypoint.Goal()
        goal_msg.position = Point(x=self.goal_x, y=self.goal_y, z=0.0)
        self.get_logger().info(f"🎯 Sending goal: x={self.goal_x:.3f}, y={self.goal_y:.3f}")
        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)

        # Wait for acceptance
        end_time = time.time() + self.timeout_sec
        while rclpy.ok() and not send_future.done() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not send_future.done():
            self.get_logger().error("❌ Goal send timeout.")
            return not self.expect_failure

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by server.")
            return not self.expect_failure

        self.get_logger().info("✅ Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()

        # Wait for action result or timeout
        while rclpy.ok() and not result_future.done() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            self.get_logger().error("❌ Result timeout (goal not finished).")
            result_ok = False
        else:
            result = result_future.result()
            status_ok = int(result.status) == 4  # SUCCEEDED
            msg_ok = getattr(result.result, "success", False)
            result_ok = status_ok and msg_ok
            self.get_logger().info(f"📋 Action result: status_ok={status_ok}, success_field={msg_ok}")

        # Allow final odom updates
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # Position & yaw validation
        pos_ok = self._check_end_position()
        yaw_ok = self._check_end_yaw()
        overall_ok = result_ok and pos_ok and yaw_ok

        # Wait for robot to stop fully
        self.get_logger().info("⏸ Waiting for robot to fully stop (cmd_vel ≈ 0)...")
        still_time = time.time()
        while rclpy.ok() and (time.time() - still_time) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if abs(self.last_twist.linear.x) > 1e-3 or abs(self.last_twist.angular.z) > 1e-3:
                still_time = time.time()
        self.get_logger().info("🟢 Robot fully stopped. Test evaluation complete.")

        # Log test summary
        if overall_ok:
            self.get_logger().info("✅ TEST PASSED: Position and yaw within limits.")
        else:
            self.get_logger().error("❌ TEST FAILED: Goal not reached or timeout.")

        # Return pass/fail
        return (not overall_ok) if self.expect_failure else overall_ok

    def _feedback_cb(self, feedback_msg):
        # Optional feedback logging
        pass


# ====================================================
# 🧩 ARG PARSER & MAIN
# ====================================================
def parse_args(argv):
    p = argparse.ArgumentParser(description="Waypoint action tester with end pos/yaw checks")
    p.add_argument('--action-name', default='/fastbot_as')
    p.add_argument('--goal-x', type=float, default=1.8)
    p.add_argument('--goal-y', type=float, default=1.2)
    p.add_argument('--cmd-vel-topic', default='fastbot/cmd_vel')
    p.add_argument('--odom-topic', default='/fastbot/odom')
    p.add_argument('--pos-margin', type=float, default=0.35)
    p.add_argument('--yaw-tol-deg', type=float, default=10.0)
    p.add_argument('--skip-pos-check', action='store_true')
    p.add_argument('--skip-yaw-check', action='store_true')
    p.add_argument('-e', '--expect-failure', default='False')
    p.add_argument('--timeout', type=float, default=40.0)
    args = p.parse_args(argv)
    expect_fail = str(args.expect_failure).lower() == 'true'
    return args, expect_fail


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args, expect_fail = parse_args(argv)
    rclpy.init()
    node = WaypointActionTester(
        action_name=args.action_name,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        cmd_vel_topic=args.cmd_vel_topic,
        odom_topic=args.odom_topic,
        expect_failure=expect_fail,
        timeout_sec=args.timeout,
        pos_margin=args.pos_margin,
        yaw_tol_deg=args.yaw_tol_deg,
        skip_pos_check=args.skip_pos_check,
        skip_yaw_check=args.skip_yaw_check,
    )
    try:
        ok = node.run()
    finally:
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
