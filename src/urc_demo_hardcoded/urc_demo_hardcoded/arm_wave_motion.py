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

        # Motion mapping on current model
        # J1 -> revolute_13, J2 -> revolute_5, J3 -> revolute_6,
        # J4 -> revolute_7, J5 -> revolute_8, J6 -> revolute_12
        self.declare_parameter(
            "joint_names",
            ["revolute_13", "revolute_5", "revolute_6", "revolute_7", "revolute_8", "revolute_12"],
        )
        self.declare_parameter(
            "joint_topics",
            ["/revolute_13_cmd", "/revolute_5_cmd", "/revolute_6_cmd", "/revolute_7_cmd", "/revolute_8_cmd", "/revolute_12_cmd"],
        )

        # Requested limitations in degrees:
        # J1=360, J2=120, J3=80, J5=360, J6=120
        self.declare_parameter("joint_amplitudes_deg", [60.0, 18.0, 22.0, 18.0, 80.0, 45.0])
        self.declare_parameter("joint_offsets_deg", [0.0, 35.0, -45.0, 25.0, 0.0, 0.0])
        self.declare_parameter("joint_phase_deg", [0.0, 90.0, 90.0, 180.0, 0.0, 0.0])

        self.declare_parameter("motion_style", "showcase")
        self.declare_parameter("frequency_hz", 0.12)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("start_delay_sec", 1.0)

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.joint_topics: List[str] = list(self.get_parameter("joint_topics").value)
        self.amplitudes_deg: List[float] = [float(v) for v in self.get_parameter("joint_amplitudes_deg").value]
        self.offsets_deg: List[float] = [float(v) for v in self.get_parameter("joint_offsets_deg").value]
        self.phase_deg: List[float] = [float(v) for v in self.get_parameter("joint_phase_deg").value]

        self.motion_style = str(self.get_parameter("motion_style").value)
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

        self.get_logger().info(f"arm_wave_motion started (style={self.motion_style})")

    def _showcase_position(self, joint_name: str, omega: float, t: float, fallback: float) -> float:
        if joint_name == "revolute_13":  # Strong base yaw sweep (constant-speed triangle wave)
            tri = (2.0 / math.pi) * math.asin(math.sin(0.45 * omega * t))
            return math.radians(150.0) * tri
        if joint_name == "revolute_5":  # Shoulder stays lifted
            return math.radians(60.0) + math.radians(12.0) * math.sin(omega * t + math.pi / 2.0)
        if joint_name == "revolute_6":  # Elbow support
            return math.radians(-70.0) + math.radians(15.0) * math.sin(omega * t + math.pi / 2.0)
        if joint_name == "revolute_7":  # Forearm follow-through
            return math.radians(35.0) + math.radians(12.0) * math.sin(omega * t + math.pi)
        if joint_name == "revolute_8":  # Wrist flourish
            return math.radians(55.0) * math.sin(1.3 * omega * t)
        if joint_name == "revolute_12":  # Tool wag
            return math.radians(40.0) * math.sin(1.7 * omega * t)
        return fallback

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
            sine_position = off + amp * math.sin(omega * t + phase)
            if self.motion_style == "showcase":
                position = self._showcase_position(joint_name, omega, t, sine_position)
            else:
                position = sine_position

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
