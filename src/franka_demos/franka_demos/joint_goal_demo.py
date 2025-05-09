#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from rclpy.executors import SingleThreadedExecutor
import numpy as np

class MoveItJointGoal(Node):
    def __init__(self):
        super().__init__("moveit_joint_goal")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "fr3_joint1", "fr3_joint2", "fr3_joint3",
                "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
            ],
            base_link_name="fr3_link0",
            end_effector_name="fr3_hand_tcp",
            group_name="fr3_arm",
            execute_via_moveit=True,
        )
        self.timer = self.create_timer(2.0, self.send_goal)

    def send_goal(self):
        joint_goal = [0.2, -0.6, 0.0, -1.0, 0.0, 1.0, 0.5]
        self.get_logger().info(f"Sending joint goal: {joint_goal}")
        self.moveit2.move_to_configuration(joint_goal)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Motion executed.")
        self.destroy_timer(self.timer)

def main():
    rclpy.init()
    node = MoveItJointGoal()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
