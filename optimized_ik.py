#!/usr/bin/env python3
"""Persistent 5-DOF IK solver for the URC arm.

Copyright (c) 2026 Iamkewl
Licensed under the MIT License.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from scipy.optimize import minimize
from sensor_msgs.msg import JointState


class ArmIKSolver(Node):
    def __init__(self):
        super().__init__('arm_ik_solver')
        self.dh_params = [
            [0.0, 0.0, 0.1, 0.0],
            [0.0, 1.57, 0.0, 0.0],
            [0.2, 0.0, 0.0, 0.0],
            [0.15, 0.0, 0.0, 0.0],
            [0.05, 0.0, 0.0, 0.0],
        ]
        self.joint_limits = [(-3.14, 3.14), (-1.57, 1.57), (-2.0, 2.0), (-2.0, 2.0), (-3.14, 3.14)]
        self.current_joints = [0.0] * 5

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Pose, '/target_pose', self.callback, 10)
        self.timer = self.create_timer(0.1, self.publish_current_state)
        self.get_logger().info("Persistent 5-DOF Solver Ready and Broadcasting.")

    def forward_kinematics(self, joints):
        T = np.eye(4)
        for i in range(5):
            a, alpha, d, off = self.dh_params[i]
            theta = joints[i] + off
            ct = math.cos(theta)
            st = math.sin(theta)
            ca = math.cos(alpha)
            sa = math.sin(alpha)
            Ti = np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ])
            T = np.dot(T, Ti)
        return T[:3, 3]

    def publish_current_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        js.position = [float(j) for j in self.current_joints]
        self.pub.publish(js)

    def callback(self, msg):
        target = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.get_logger().info(f"Solving for Target: {target}")
        res = minimize(
            lambda j: np.linalg.norm(self.forward_kinematics(j) - target),
            self.current_joints,
            bounds=self.joint_limits,
            method='L-BFGS-B',
        )
        if res.success:
            self.current_joints = res.x.tolist()
            self.get_logger().info("IK Success! Position updated.")
        else:
            self.get_logger().warn("IK failed to find a valid solution.")


def main():
    rclpy.init()
    rclpy.spin(ArmIKSolver())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
