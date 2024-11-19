#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2 as cv
import time


class ControlRobot(Node):
    def __init__(self):
        super().__init__('control_robot')
        self.get_logger().info("Starting the node...")

        self.topic_name = '/cmdvel_robot'

        self.get_logger().info(
            f'publishing to {self.topic_name}'
        )

        self.cmdvel_robot = self.create_publisher(
            Twist, self.topic_name, 10)
        time.sleep(1)

   
    def compute_next_command(self):
        # TODO: go on from here!
        pass

    def robot_stop(self):
        twist = Twist()

        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.cmdvel_robot.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    ControlRobotNode = ControlRobot()

    try:
        while rclpy.ok():
            ControlRobotNode.compute_next_command()
    except KeyboardInterrupt:
        ControlRobotNode.robot_stop()
        time.sleep(0.5)

    # Destroy the node
    ControlRobotNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
