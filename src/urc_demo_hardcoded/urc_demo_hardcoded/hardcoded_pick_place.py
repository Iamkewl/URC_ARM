#!/usr/bin/env python3
"""Hard-coded pick-and-place sequencer for quick Gazebo demos."""

from typing import Dict, List

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint


class HardcodedPickPlace(Node):
    def __init__(self) -> None:
        super().__init__("hardcoded_pick_place")

        self.declare_parameter("controller_action", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("controller_mode", "ros_gz_topic")
        self.declare_parameter(
            "joint_names",
            [
                "revolute_13",
                "revolute_5",
                "revolute_6",
                "revolute_7",
                "revolute_8",
                "revolute_12",
            ],
        )
        self.declare_parameter(
            "waypoint_order",
            [
                "home",
                "joint1_pos",
                "joint1_neg",
                "home",
                "joint2_pos",
                "joint2_neg",
                "home",
                "joint3_pos",
                "joint3_neg",
                "home",
                "joint4_pos",
                "joint4_neg",
                "home",
                "joint5_pos",
                "joint5_neg",
                "home",
                "joint6_pos",
                "joint6_neg",
                "home",
                "pregrasp",
                "grasp",
                "lift",
                "preplace",
                "place",
                "retreat",
                "home",
            ],
        )

        self.declare_parameter("waypoints.home", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint1_pos", [0.70, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint1_neg", [-0.70, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint2_pos", [0.0, 0.55, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint2_neg", [0.0, -0.55, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint3_pos", [0.0, 0.0, 0.80, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint3_neg", [0.0, 0.0, -0.80, 0.0, 0.0, 0.0])
        self.declare_parameter("waypoints.joint4_pos", [0.0, 0.0, 0.0, 0.85, 0.0, 0.0])
        self.declare_parameter("waypoints.joint4_neg", [0.0, 0.0, 0.0, -0.85, 0.0, 0.0])
        self.declare_parameter("waypoints.joint5_pos", [0.0, 0.0, 0.0, 0.0, 0.95, 0.0])
        self.declare_parameter("waypoints.joint5_neg", [0.0, 0.0, 0.0, 0.0, -0.95, 0.0])
        self.declare_parameter("waypoints.joint6_pos", [0.0, 0.0, 0.0, 0.0, 0.0, 0.95])
        self.declare_parameter("waypoints.joint6_neg", [0.0, 0.0, 0.0, 0.0, 0.0, -0.95])
        self.declare_parameter("waypoints.pregrasp", [0.0, 0.15, -0.35, 0.50, 0.0, 0.0])
        self.declare_parameter("waypoints.grasp", [0.0, 0.20, -0.55, 0.65, 0.0, 0.0])
        self.declare_parameter("waypoints.lift", [0.0, 0.12, -0.25, 0.45, 0.0, 0.0])
        self.declare_parameter("waypoints.preplace", [0.6, 0.12, -0.25, 0.45, 0.0, 0.0])
        self.declare_parameter("waypoints.place", [0.6, 0.18, -0.45, 0.60, 0.0, 0.0])
        self.declare_parameter("waypoints.retreat", [0.6, 0.10, -0.20, 0.35, 0.0, 0.0])

        self.declare_parameter("move_duration_sec", 2.0)
        self.declare_parameter("pause_after_grasp_sec", 0.5)
        self.declare_parameter("pause_after_place_sec", 0.5)
        self.declare_parameter("loop_forever", True)

        self.declare_parameter("gripper_joint_name", "revolute_12")
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_closed_position", 0.8)
        self.declare_parameter("gripper_move_duration_sec", 1.0)
        self.declare_parameter(
            "joint_command_topics",
            [
                "/revolute_13_cmd",
                "/revolute_5_cmd",
                "/revolute_6_cmd",
                "/revolute_7_cmd",
                "/revolute_8_cmd",
                "/revolute_12_cmd",
            ],
        )

        self.declare_parameter("publish_planning_scene", True)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("end_effector_link", "link_6_1")
        self.declare_parameter("object_id", "demo_object")
        self.declare_parameter("object_size", [0.04, 0.04, 0.12])
        self.declare_parameter("object_pose", [0.32, 0.0, 0.06, 0.0, 0.0, 0.0, 1.0])

        self.controller_action = self.get_parameter("controller_action").value
        self.controller_mode = str(self.get_parameter("controller_mode").value)
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.waypoint_order = list(self.get_parameter("waypoint_order").value)
        self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)
        self.pause_after_grasp_sec = float(self.get_parameter("pause_after_grasp_sec").value)
        self.pause_after_place_sec = float(self.get_parameter("pause_after_place_sec").value)
        self.loop_forever = bool(self.get_parameter("loop_forever").value)
        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self.gripper_closed_position = float(self.get_parameter("gripper_closed_position").value)
        self.gripper_move_duration_sec = float(self.get_parameter("gripper_move_duration_sec").value)
        self.joint_command_topics = list(self.get_parameter("joint_command_topics").value)

        self.publish_planning_scene = bool(self.get_parameter("publish_planning_scene").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.end_effector_link = str(self.get_parameter("end_effector_link").value)
        self.object_id = str(self.get_parameter("object_id").value)

        self.waypoints = self._read_waypoints()
        self.current_positions = list(self.waypoints.get("home", [0.0] * len(self.joint_names)))

        if len(self.joint_command_topics) != len(self.joint_names):
            raise ValueError(
                "joint_command_topics length must match joint_names length"
            )

        self.topic_publishers = {
            joint_name: self.create_publisher(Float64, topic, 10)
            for joint_name, topic in zip(self.joint_names, self.joint_command_topics)
        }

        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.trajectory_client = None
        if self.controller_mode == "follow_joint_trajectory":
            self.trajectory_client = ActionClient(
                self, FollowJointTrajectory, self.controller_action
            )

    def _read_waypoints(self) -> Dict[str, List[float]]:
        waypoints: Dict[str, List[float]] = {}
        for name in self.waypoint_order:
            key = f"waypoints.{name}"
            if not self.has_parameter(key):
                self.declare_parameter(key, [0.0] * len(self.joint_names))
            values = list(self.get_parameter(key).value)
            if len(values) != len(self.joint_names):
                raise ValueError(
                    f"{key} has {len(values)} entries but joint_names has {len(self.joint_names)}"
                )
            waypoints[name] = [float(v) for v in values]
        return waypoints

    def run(self) -> None:
        if self.controller_mode == "follow_joint_trajectory":
            self.get_logger().info("Waiting for trajectory action server...")
            if self.trajectory_client is None or not self.trajectory_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(
                    f"Action server not available on {self.controller_action}. "
                    "Start ros2_control + joint trajectory controller first."
                )
                return
        elif self.controller_mode == "ros_gz_topic":
            self.get_logger().info("Using ros_gz topic mode for joint commands")
        else:
            raise ValueError(
                "controller_mode must be 'follow_joint_trajectory' or 'ros_gz_topic'"
            )

        if self.publish_planning_scene:
            self.publish_world_object()

        while rclpy.ok():
            if not self.execute_sequence_once():
                break
            if not self.loop_forever:
                break
        self.get_logger().info("Hard-coded sequence finished.")

    def execute_sequence_once(self) -> bool:
        for name in self.waypoint_order:
            ok = self.send_joint_goal(name, self.waypoints[name])
            if not ok:
                return False

            if name == "grasp":
                self.command_gripper(close=True)
                if self.publish_planning_scene:
                    self.attach_object_to_tool()
                self._sleep(self.pause_after_grasp_sec)
            elif name == "place":
                self.command_gripper(close=False)
                if self.publish_planning_scene:
                    self.detach_object_to_world()
                self._sleep(self.pause_after_place_sec)
        return True

    def send_joint_goal(self, waypoint_name: str, positions: List[float]) -> bool:
        if self.controller_mode == "ros_gz_topic":
            return self._send_joint_goal_ros_gz(waypoint_name, positions)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(self.move_duration_sec), nanosec=int((self.move_duration_sec % 1) * 1e9))
        goal.trajectory.points = [point]

        self.get_logger().info(f"Moving to waypoint: {waypoint_name}")
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Waypoint {waypoint_name} rejected by controller")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None or result.status != 4:  # 4 = STATUS_SUCCEEDED
            status = "none" if result is None else str(result.status)
            self.get_logger().error(f"Waypoint {waypoint_name} failed with status {status}")
            return False

        self.current_positions = list(positions)
        return True

    def _send_joint_goal_ros_gz(self, waypoint_name: str, positions: List[float]) -> bool:
        self.get_logger().info(f"Moving to waypoint: {waypoint_name}")
        deadline = self.get_clock().now().nanoseconds + int(self.move_duration_sec * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            for joint_name, position in zip(self.joint_names, positions):
                msg = Float64()
                msg.data = float(position)
                self.topic_publishers[joint_name].publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.current_positions = list(positions)
        return True

    def command_gripper(self, close: bool) -> bool:
        if self.gripper_joint_name not in self.joint_names:
            self.get_logger().warn(
                f"gripper_joint_name '{self.gripper_joint_name}' not in joint_names; skipping gripper command"
            )
            return False

        gripper_index = self.joint_names.index(self.gripper_joint_name)
        target = list(self.current_positions)
        target[gripper_index] = (
            self.gripper_closed_position if close else self.gripper_open_position
        )

        original_move_duration = self.move_duration_sec
        self.move_duration_sec = self.gripper_move_duration_sec
        try:
            action = "close" if close else "open"
            return self.send_joint_goal(f"gripper_{action}", target)
        finally:
            self.move_duration_sec = original_move_duration

    def publish_world_object(self) -> None:
        obj = CollisionObject()
        obj.id = self.object_id
        obj.header.frame_id = self.world_frame
        obj.operation = CollisionObject.ADD

        dimensions = list(self.get_parameter("object_size").value)
        if len(dimensions) != 3:
            raise ValueError("object_size must contain exactly 3 values")

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(dimensions[0]), float(dimensions[1]), float(dimensions[2])]

        object_pose_values = list(self.get_parameter("object_pose").value)
        if len(object_pose_values) != 7:
            raise ValueError("object_pose must have 7 entries [x,y,z,qx,qy,qz,qw]")

        pose = Pose()
        pose.position.x = float(object_pose_values[0])
        pose.position.y = float(object_pose_values[1])
        pose.position.z = float(object_pose_values[2])
        pose.orientation.x = float(object_pose_values[3])
        pose.orientation.y = float(object_pose_values[4])
        pose.orientation.z = float(object_pose_values[5])
        pose.orientation.w = float(object_pose_values[6])

        obj.primitives = [primitive]
        obj.primitive_poses = [pose]

        msg = PlanningScene()
        msg.is_diff = True
        msg.world.collision_objects = [obj]

        self.scene_pub.publish(msg)
        self.get_logger().info("Published world collision object")

    def attach_object_to_tool(self) -> None:
        attach = AttachedCollisionObject()
        attach.link_name = self.end_effector_link
        attach.object.id = self.object_id
        attach.object.header.frame_id = self.end_effector_link
        attach.object.operation = CollisionObject.ADD

        msg = PlanningScene()
        msg.is_diff = True
        msg.robot_state.is_diff = True
        msg.robot_state.attached_collision_objects = [attach]

        self.scene_pub.publish(msg)
        self.get_logger().info("Attached object in planning scene")

    def detach_object_to_world(self) -> None:
        attach = AttachedCollisionObject()
        attach.link_name = self.end_effector_link
        attach.object.id = self.object_id
        attach.object.operation = CollisionObject.REMOVE

        msg = PlanningScene()
        msg.is_diff = True
        msg.robot_state.is_diff = True
        msg.robot_state.attached_collision_objects = [attach]

        self.scene_pub.publish(msg)
        self.get_logger().info("Detached object in planning scene")

    def _sleep(self, seconds: float) -> None:
        if seconds <= 0.0:
            return
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> None:
    rclpy.init()
    node = HardcodedPickPlace()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, stopping hardcoded pick/place node")
    except Exception as exc:  # pylint: disable=broad-except
        node.get_logger().error(f"Hardcoded pick/place node crashed: {exc}")
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
