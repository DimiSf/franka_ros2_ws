#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_sender')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/fr3_arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.send_trajectory)
        self.sent = False

    def send_trajectory(self):
        if self.sent:
            return

        now = self.get_clock().now().to_msg()

        traj = JointTrajectory()
        traj.header.stamp = now
        traj.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]

        point = JointTrajectoryPoint()
        point.positions = [1.0, -1.5, 1.0, -2.5, 0.5, 1.2, 1.0]

        point.velocities = [0.0] * 7
        point.time_from_start.sec = 3

        traj.points.append(point)

        self.publisher.publish(traj)
        self.get_logger().info("Sent trajectory to fr3_arm_controller.")
        self.sent = True


def main():
    rclpy.init()
    node = TrajectorySender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
