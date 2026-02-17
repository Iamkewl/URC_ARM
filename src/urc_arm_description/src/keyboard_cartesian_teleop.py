#!/usr/bin/env python3

import sys
import termios
import tty
from typing import List

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration, Time
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from rclpy.node import Node
from sensor_msgs.msg import JointState


HELP_TEXT = """
Cartesian Keyboard Teleop (position-only)
-----------------------------------------
Move TCP target:
  i / k : +X / -X
  j / l : +Y / -Y
  u / o : +Z / -Z

Other:
  h     : show help
  x     : clear efforts (stop motion)
  CTRL-C: quit
"""


class KeyboardCartesianTeleop(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_cartesian_teleop")

        self.joint_names: List[str] = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        self.key_to_delta = {
            "i": np.array([+1.0, 0.0, 0.0]),
            "k": np.array([-1.0, 0.0, 0.0]),
            "j": np.array([0.0, +1.0, 0.0]),
            "l": np.array([0.0, -1.0, 0.0]),
            "u": np.array([0.0, 0.0, +1.0]),
            "o": np.array([0.0, 0.0, -1.0]),
        }

        self.cartesian_step = float(self.declare_parameter("cartesian_step_m", 0.01).value)
        self.jacobian_eps = float(self.declare_parameter("jacobian_eps", 1e-4).value)
        self.ik_gain = float(self.declare_parameter("ik_gain", 0.45).value)
        self.joint_kp = float(self.declare_parameter("joint_kp", 2.0).value)
        self.max_effort = float(self.declare_parameter("max_effort", 0.8).value)
        self.effort_duration_sec = float(self.declare_parameter("duration_sec", 0.08).value)

        # Approximate DH model for position IK (must be tuned to your final URDF)
        self.dh_params = [
            [0.0, 0.0, 0.085, 0.0],
            [0.0, 1.5708, 0.197, 0.0],
            [0.35, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.239, 0.0],
            [0.0, 0.0, -0.135, 0.0],
        ]

        self.joint_limits = np.array(
            [
                [-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14],
                [-3.14, 3.14],
            ],
            dtype=float,
        )

        self.current_joints = np.zeros(5, dtype=float)
        self.target_joints = np.zeros(5, dtype=float)
        self.target_xyz = None
        self.have_joint_state = False

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self.apply_effort_client = self.create_client(ApplyJointEffort, "/apply_joint_effort")
        self.clear_effort_client = self.create_client(JointRequest, "/clear_joint_efforts")

        if not self.apply_effort_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/apply_joint_effort service is not available")
        if not self.clear_effort_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/clear_joint_efforts service is not available")

        self.get_logger().info(HELP_TEXT)

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        updated = False
        for j, joint_name in enumerate(self.joint_names):
            idx = name_to_idx.get(joint_name)
            if idx is not None and idx < len(msg.position):
                self.current_joints[j] = float(msg.position[idx])
                updated = True

        if updated and not self.have_joint_state:
            self.have_joint_state = True
            self.target_joints = self.current_joints.copy()
            self.target_xyz = self.forward_kinematics(self.current_joints)
            self.get_logger().info(
                f"Initialized target at xyz = {self.target_xyz[0]:.3f}, {self.target_xyz[1]:.3f}, {self.target_xyz[2]:.3f}"
            )

    def forward_kinematics(self, joints: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        for i in range(5):
            a, alpha, d, off = self.dh_params[i]
            theta = joints[i] + off
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)

            Ti = np.array(
                [
                    [ct, -st * ca, st * sa, a * ct],
                    [st, ct * ca, -ct * sa, a * st],
                    [0.0, sa, ca, d],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
            T = T @ Ti

        return T[:3, 3]

    def numeric_jacobian(self, joints: np.ndarray) -> np.ndarray:
        J = np.zeros((3, 5), dtype=float)
        base = self.forward_kinematics(joints)
        for i in range(5):
            perturbed = joints.copy()
            perturbed[i] += self.jacobian_eps
            J[:, i] = (self.forward_kinematics(perturbed) - base) / self.jacobian_eps
        return J

    def _solve_ik_step(self, desired_xyz: np.ndarray) -> np.ndarray:
        q = self.target_joints.copy()
        x = self.forward_kinematics(q)
        err = desired_xyz - x

        J = self.numeric_jacobian(q)
        dq = self.ik_gain * np.linalg.pinv(J) @ err

        q_new = q + dq
        q_new = np.clip(q_new, self.joint_limits[:, 0], self.joint_limits[:, 1])
        return q_new

    def apply_key(self, key: str) -> None:
        if key == "h":
            self.get_logger().info(HELP_TEXT)
            return

        if key == "x":
            self._clear_all_efforts()
            self.get_logger().info("Cleared joint efforts")
            return

        if not self.have_joint_state:
            self.get_logger().warn("Waiting for /joint_states...")
            return

        if key not in self.key_to_delta:
            return

        self.target_xyz = self.target_xyz + self.key_to_delta[key] * self.cartesian_step
        self.target_joints = self._solve_ik_step(self.target_xyz)

        joint_error = self.target_joints - self.current_joints
        efforts = np.clip(self.joint_kp * joint_error, -self.max_effort, self.max_effort)

        for joint_name, effort in zip(self.joint_names, efforts):
            self._apply_joint_effort(joint_name, float(effort))

        self.get_logger().info(
            f"target xyz={self.target_xyz[0]:.3f}, {self.target_xyz[1]:.3f}, {self.target_xyz[2]:.3f}"
        )

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
    node = KeyboardCartesianTeleop()

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
