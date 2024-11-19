#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
from threading import Thread

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        self.get_logger().info("CircleMover Node started!")
        self.publisher = self.create_publisher(Twist, '/cmdvvel_obstacle1', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.move_in_circle)
        self.radius = 0.5  # Radius
        self.linear_speed = 0.6
        self.angular_speed = self.linear_speed / self.radius

    def move_in_circle(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)


class RectangularPath(Node):
    def __init__(self):
        super().__init__('rectangular_path')
        self.get_logger().info("RectangularPath Node started!")
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmdvvel_obstacle2', 10)
        time.sleep(1)

        # rectangular movement
        self.linear_speed = 0.5
        self.angular_speed = 0.8
        self.side_lengths = [2.0, 2.0]

    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = self.linear_speed
        time_duration = distance / self.linear_speed
        self.publish_for_duration(twist, time_duration)

    def turn_90_degrees(self):
        twist = Twist()
        twist.angular.z = -self.angular_speed
        time_duration = 1.57 / self.angular_speed
        self.publish_for_duration(twist, time_duration)

    def publish_for_duration(self, twist, duration):
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        self.cmd_vel_publisher.publish(Twist())
        time.sleep(1)

    def run_path(self):
        for _ in range(2):
            self.move_forward(self.side_lengths[0])
            self.turn_90_degrees()
            self.move_forward(self.side_lengths[1])
            self.turn_90_degrees()

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    circle_mover = CircleMover()
    rectangular_path = RectangularPath()

    circle_thread = Thread(target=rclpy.spin, args=(circle_mover,))
    rect_thread = Thread(target=rectangular_path.run_path)

    try:
        circle_thread.start()
        rect_thread.start()
        rect_thread.join() 
    except KeyboardInterrupt:
        pass
    finally:
        # Stop
        circle_mover.stop()
        rectangular_path.stop()
        
        # Close nodes
        circle_mover.destroy_node()
        rectangular_path.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
