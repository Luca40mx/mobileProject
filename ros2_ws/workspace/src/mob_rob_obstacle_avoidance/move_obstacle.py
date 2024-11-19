#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math 


class SimpleTrajectory(Node):
    def __init__(self):
        super().__init__('move_obstacle')
        self.get_logger().info("Starting the node...")

        self.declare_parameter('obstacle_index', 1)
        self.declare_parameter('trajectory', 'line')

        self.obstacle_index = self.get_parameter(
            'obstacle_index').get_parameter_value().integer_value
        self.trajectory_type = self.get_parameter(
            'trajectory').get_parameter_value().string_value

        self.topic_name = f'/cmdvel_obstacle{int(self.obstacle_index)}'

        self.get_logger().info(
            f'publishing to {self.topic_name} | trajectory type: {self.trajectory_type}')

        self.cmdvel_obstacle = self.create_publisher(
            Twist, self.topic_name, 10)
        time.sleep(1)

        self.linear_speed = 0.25
        self.angular_speed = math.pi/6
        
        self.line_traj_lenght = 1

        self.rectange_traj_lengths = [2.0, 2.0]  # Length of the sides of the rectangle

    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = self.linear_speed
        time_duration = distance / self.linear_speed
        self.publish_for_duration(twist, time_duration)

    def turn_90_degrees(self):
        "Rotate og 90 degrees to the right."
        twist = Twist()
        twist.angular.z = -self.angular_speed
        time_duration = 1.57 / self.angular_speed  # 90 degrees in rad = π/2 rad = 1.57
        self.publish_for_duration(twist, time_duration)

    def publish_for_duration(self, twist, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            # print(twist)
            self.cmdvel_obstacle.publish(twist)
            time.sleep(0.1)
        # Stop the robot
        self.cmdvel_obstacle.publish(Twist())
        time.sleep(1)

    def run_trajectory(self):

        if self.trajectory_type == 'line':
            self.move_forward(self.line_traj_lenght)
            self.turn_90_degrees()
            self.turn_90_degrees()
            self.move_forward(self.line_traj_lenght)
            self.turn_90_degrees()
            self.turn_90_degrees() 
                
        elif self.trajectory_type == 'rectangle':
            for _ in range(2):
                self.move_forward(self.rectange_traj_lengths[0])
                self.turn_90_degrees()
                self.move_forward(self.rectange_traj_lengths[1])
                self.turn_90_degrees()

        # TODO: add other trajectories?

    def robot_stop(self):
        twist = Twist()

        twist.angular.z = 0.0
        twist.linear.x = 0.0
        self.cmdvel_obstacle.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    simpleTrajectoryNode = SimpleTrajectory()

    try:
        while rclpy.ok():
            simpleTrajectoryNode.run_trajectory()
    except KeyboardInterrupt:
        simpleTrajectoryNode.robot_stop()
        time.sleep(0.5)

    # Destroy the node
    simpleTrajectoryNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
