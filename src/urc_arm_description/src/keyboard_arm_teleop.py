#!/usr/bin/env python3

import sys
import termios
import tty
from typing import Dict, List

import rclpy
from builtin_interfaces.msg import Duration, Time
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from rclpy.node import Node


HELP_TEXT = """
5-DOF Arm Keyboard Teleop
-------------------------
Increase / decrease joints:
  1 / q : joint_1
  2 / w : joint_2
  3 / e : joint_3
  4 / r : joint_4
  5 / t : joint_5

Other:
  h     : show help
    x     : clear efforts (stop motion)
  CTRL-C: quit
"""


class KeyboardArmTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_arm_teleop")

        self.joint_names: List[str] = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
        ]
        self.key_to_joint_delta: Dict[str, tuple[int, float]] = {
            "1": (0, +1.0),
            "q": (0, -1.0),
            "2": (1, +1.0),
            "w": (1, -1.0),
            "3": (2, +1.0),
            "e": (2, -1.0),
            "4": (3, +1.0),
            "r": (3, -1.0),
            "5": (4, +1.0),
            "t": (4, -1.0),
        }

        self.effort_value = float(self.declare_parameter("effort", 0.6).value)
        self.effort_duration_sec = float(self.declare_parameter("duration_sec", 0.08).value)

        self.apply_effort_client = self.create_client(
            ApplyJointEffort,
            "/apply_joint_effort",
        )
        self.clear_effort_client = self.create_client(
            JointRequest,
            "/clear_joint_efforts",
        )

        if not self.apply_effort_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/apply_joint_effort service is not available")

        if not self.clear_effort_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/clear_joint_efforts service is not available")

        self.get_logger().info(HELP_TEXT)

    def apply_key(self, key: str) -> None:
        if key == "h":
            self.get_logger().info(HELP_TEXT)
            return

        if key == "x":
            self._clear_all_efforts()
            self.get_logger().info("Cleared joint efforts")
            return

        if key not in self.key_to_joint_delta:
            return

        joint_idx, sign = self.key_to_joint_delta[key]
        joint_name = self.joint_names[joint_idx]
        effort = sign * self.effort_value
        self._apply_joint_effort(joint_name, effort)

        self.get_logger().info(f"{joint_name}: effort {effort:.3f}")

    def _apply_joint_effort(self, joint_name: str, effort: float) -> None:
        req = ApplyJointEffort.Request()
        req.joint_name = joint_name
        req.effort = effort
        req.start_time = Time(sec=0, nanosec=0)

        duration = max(self.effort_duration_sec, 0.01)
        sec = int(duration)
        nanosec = int((duration - sec) * 1e9)
        req.duration = Duration(sec=sec, nanosec=nanosec)

        future = self.apply_effort_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() is None:
            self.get_logger().warn(f"Failed to apply effort on {joint_name}")

    def _clear_all_efforts(self) -> None:
        for joint_name in self.joint_names:
            req = JointRequest.Request()
            req.joint_name = joint_name

            future = self.clear_effort_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)


def _get_key() -> str:
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        return ""
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main() -> None:
    rclpy.init()
    node = KeyboardArmTeleop()

    try:
        while rclpy.ok():
            key = _get_key()
            if key == "":
                node.get_logger().warn("No interactive TTY available for keyboard input")
                rclpy.spin_once(node, timeout_sec=0.5)
                continue
            if key == "\x03":
                break
            node.apply_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
