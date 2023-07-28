#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import baxter

import numpy as np
import time

from geometry_msgs.msg import Pose

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
from tf2_geometry_msgs import do_transform_pose


class PoseFollower(Node):
    def __init__(self):
        super().__init__("servo_pose_follower")

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=baxter.joint_names(),
            base_link_name=baxter.base_link_name(),
            end_effector_name=baxter.end_effector_name(),
            group_name=baxter.MOVE_GROUP_ARM,
            execute_via_moveit=True,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.frame = "base"
        self.pose_frame = "world_absolute"  # TODO: Change to param

        self.goal_position = np.array([0.8, -0.2, 0.25])
        self.current_pos = self.goal_position
        self.goal_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        self.moveit2.move_to_pose(
            position=self.goal_position,
            quat_xyzw=self.goal_quaternion,
            cartesian=False,
        )

        self.subscription = self.create_subscription(
            Pose, "target_pose", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        delta_pos = self.goal_position - self.current_pos
        distance = np.linalg.norm(delta_pos)

        if distance < 0.01:
            return
        mov = self.current_pos + ((delta_pos / distance) * 0.1)

        self.moveit2.move_to_pose(
            position=mov,
            quat_xyzw=self.goal_quaternion,
            cartesian=True,
        )

        self.current_pos = mov

    def listener_callback(self, msg):
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.frame,
                self.pose_frame,
                rclpy.time.Time(),
            )

            pose_base = do_transform_pose(msg, base_tf)

            self.goal_position = np.array(
                [
                    pose_base.position.x,
                    pose_base.position.y,
                    pose_base.position.z,
                ]
            )
            self.goal_quaternion = np.array(
                [
                    pose_base.orientation.x,
                    pose_base.orientation.y,
                    pose_base.orientation.z,
                    pose_base.orientation.w,
                ]
            )

        except Exception as e:
            self.get_logger().info(
                f"Could not transform {self.frame} to {self.pose_frame}: {e}"
            )


def main(args=None):
    rclpy.init(args=args)

    pose_follower = PoseFollower()

    # Declare parameters for position and orientation
    pose_follower.declare_parameter("position", [0.8, -0.2, 0.25])
    pose_follower.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
    pose_follower.declare_parameter("cartesian", False)

    rclpy.spin(pose_follower)

    pose_follower.destroy_node()
    rclpy.shutdown()


# def main():
#     rclpy.init()

#     # Create node for this example
#     node = Node("ex_pose_goal")

#     # Declare parameters for position and orientation
#     node.declare_parameter("position", [0.8, -0.2, 0.25])
#     node.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
#     node.declare_parameter("cartesian", False)

#     # Create callback group that allows execution of callbacks in parallel without restrictions
#     callback_group = ReentrantCallbackGroup()

#     # Create MoveIt 2 interface
# moveit2 = MoveIt2(
#         node=node,
#         joint_names=baxter.joint_names(),
#         base_link_name=baxter.base_link_name(),
#         end_effector_name=baxter.end_effector_name(),
#         group_name=baxter.MOVE_GROUP_ARM,
#         callback_group=callback_group,
#         execute_via_moveit=True,
#     )

#     # Spin the node in background thread(s)
#     executor = rclpy.executors.MultiThreadedExecutor(2)
#     executor.add_node(node)
#     executor_thread = Thread(target=executor.spin, daemon=True, args=())
#     executor_thread.start()

#     # Get parameters
#     position = node.get_parameter("position").get_parameter_value().double_array_value
#     quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
#     cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

#     # Move to pose
#     node.get_logger().info(
#         f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
#     )

#     t_list = [0, 10]
#     z_list = [0.15, 0.45]

#     while True:
#         for t in range(t_list[1] - t_list[0]):
#             moveit2.move_to_pose(
#                 position=[0.6, -0.2, np.interp(t, t_list, z_list)],
#                 quat_xyzw=quat_xyzw,
#                 cartesian=cartesian,
#             )
#             moveit2.wait_until_executed()
#             time.sleep(0.01)

#     rclpy.shutdown()
#     exit(0)


if __name__ == "__main__":
    main()
