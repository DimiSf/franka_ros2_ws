#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Grasp, Move
import time


class FrankaPickPlace(Node):
    def __init__(self):
        super().__init__('franka_pick_place')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
        self.grasp_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')
        self.open_client = ActionClient(self, Move, '/fr3_gripper/move')

    def move_arm(self, joint_positions, duration=4.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        traj.points.append(point)
        goal.trajectory = traj

        self.arm_client.wait_for_server()
        self.get_logger().info(f'[ARM] Moving to: {joint_positions}')
        send_goal_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm motion goal rejected!')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('[ARM] Motion complete.')

    def grasp(self, width=0.03, speed=0.02, force=20.0):
        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = 0.005
        goal_msg.epsilon.outer = 0.005

        self.grasp_client.wait_for_server()
        self.get_logger().info('[GRIPPER] Grasping...')
        future = self.grasp_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('[GRIPPER] Grasp complete.')

    def open(self, width=0.08, speed=0.02):
        goal_msg = Move.Goal()
        goal_msg.width = width
        goal_msg.speed = speed

        self.open_client.wait_for_server()
        self.get_logger().info('[GRIPPER] Opening...')
        future = self.open_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('[GRIPPER] Open complete.')


def main(args=None):
    rclpy.init(args=args)
    node = FrankaPickPlace()

    try:
        # HOME position
        HOME = [0.0, 0.0, 0.0, -2.2, 0.0, 2.2, 0.7]

        # Pregrasp position (from your libfranka)
        PREGRASP = [-0.0175, 0.3665, 0.0175, -2.3213, 0.0, 2.6878, 0.7679]

        # Lift and Drop: change only joint 2 (Z)
        Z_LIFT_DELTA = -0.15
        Z_UP = PREGRASP.copy()
        Z_UP[1] += Z_LIFT_DELTA

        DROP = Z_UP.copy()
        DROP[0] = -0.2269     # Rotate base
        DROP[2] = -0.2069     # Slight adjust in joint 3
        DROP[4] = 0.1768      # Adjust wrist angle

        Z_DOWN = DROP.copy()
        Z_DOWN[1] -= Z_LIFT_DELTA  # same amount down

        # Sequence
        node.move_arm(HOME)
        time.sleep(2)

        node.move_arm(PREGRASP)
        time.sleep(2)

        node.grasp()
        time.sleep(1)

        node.move_arm(Z_UP)
        time.sleep(2)

        node.move_arm(DROP)
        time.sleep(2)

        node.move_arm(Z_DOWN)
        time.sleep(2)

        node.open()
        time.sleep(1)

        node.move_arm(HOME)
        time.sleep(2)

        node.get_logger().info('[DONE] Pick and place routine complete.')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
