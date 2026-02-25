#!/usr/bin/env python3
"""Simple joint-space motion demo for the 5-DOF arm mapping in Gazebo Fortress."""

import math
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ArmWaveMotion(Node):
    def __init__(self) -> None:
        super().__init__("arm_wave_motion")

        # 5-DOF mapping requested by user (mapped onto available joints)
        # J1 -> revolute_13, J2 -> revolute_5, J3 -> revolute_6, J5 -> revolute_8, J6 -> revolute_12
        self.declare_parameter(
            "joint_names",
            ["revolute_13", "revolute_5", "revolute_6", "revolute_8", "revolute_12"],
        )
        self.declare_parameter(
            "joint_topics",
            ["/revolute_13_cmd", "/revolute_5_cmd", "/revolute_6_cmd", "/revolute_8_cmd", "/revolute_12_cmd"],
        )

        # Requested limitations in degrees:
        # J1=360, J2=120, J3=80, J5=360, J6=120
        self.declare_parameter("joint_amplitudes_deg", [150.0, 50.0, 35.0, 150.0, 50.0])
        self.declare_parameter("joint_offsets_deg", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("joint_phase_deg", [0.0, 60.0, 120.0, 180.0, 240.0])

        self.declare_parameter("frequency_hz", 0.08)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("start_delay_sec", 3.0)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.joint_topics: List[str] = list(self.get_parameter("joint_topics").value)
        self.amplitudes_deg: List[float] = [float(v) for v in self.get_parameter("joint_amplitudes_deg").value]
        self.offsets_deg: List[float] = [float(v) for v in self.get_parameter("joint_offsets_deg").value]
        self.phase_deg: List[float] = [float(v) for v in self.get_parameter("joint_phase_deg").value]

        self.frequency_hz = float(self.get_parameter("frequency_hz").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)

        n = len(self.joint_names)
        if not (len(self.joint_topics) == len(self.amplitudes_deg) == len(self.offsets_deg) == len(self.phase_deg) == n):
            raise ValueError("joint_names, joint_topics, amplitudes, offsets, and phases must have equal lengths")

        self.joint_publishers = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in zip(self.joint_names, self.joint_topics)
        }

        self.start_time = self.get_clock().now()
        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._publish_motion)

        self.get_logger().info("arm_wave_motion started (5-DOF mapped motion demo)")

    def _publish_motion(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.start_delay_sec:
            return

        t = elapsed - self.start_delay_sec
        omega = 2.0 * math.pi * self.frequency_hz

        for idx, joint_name in enumerate(self.joint_names):
            amp = math.radians(self.amplitudes_deg[idx])
            off = math.radians(self.offsets_deg[idx])
            phase = math.radians(self.phase_deg[idx])
            position = off + amp * math.sin(omega * t + phase)

            msg = Float64()
            msg.data = position
            self.joint_publishers[joint_name].publish(msg)


def main() -> None:
    rclpy.init()
    node = ArmWaveMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("arm_wave_motion interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
