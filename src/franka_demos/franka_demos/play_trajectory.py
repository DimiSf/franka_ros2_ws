#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from rclpy.executors import SingleThreadedExecutor
import numpy as np
import time

class MoveItTrajectoryPlayer(Node):
    def __init__(self):
        super().__init__("moveit_trajectory_player")
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
        self.timer = self.create_timer(2.0, self.play_trajectory)

    def play_trajectory(self):
        # Define start and end poses
        start = np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854])
        end   = np.array([0.0, -1.0,    0.2, -1.5,    0.0, 1.2,    1.0   ])

        steps = 100
        duration = 5.0
        dt = duration / steps

        self.get_logger().info("Starting trajectory playback via MoveIt...")

        for i in range(steps):
            t = i / (steps - 1)
            q = (1 - t) * start + t * end
            self.moveit2.move_to_configuration(q.tolist())
            self.moveit2.wait_until_executed()
            time.sleep(dt)

        self.get_logger().info("✅ Trajectory completed.")
        self.destroy_timer(self.timer)

def main():
    rclpy.init()
    node = MoveItTrajectoryPlayer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
