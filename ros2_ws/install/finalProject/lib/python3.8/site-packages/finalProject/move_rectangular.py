#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RectangularPath(Node):
    def __init__(self):
        super().__init__('rectangular_path')
        self.get_logger().info("Node started!")
        # Publish on topic cmd_vel_2
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmdvvel_obstacle2', 10)
        time.sleep(1)
        
        # Parameters
        self.linear_speed = 0.5 
        self.angular_speed = 0.8  
        self.side_lengths = [2.0, 2.0]  # Length of the sides of the rectangle

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
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1) 
        # Stop the robot
        self.cmd_vel_publisher.publish(Twist()) 
        time.sleep(1) 

    def run_path(self):
        "do the rectangular trajectory."
        for _ in range(2): 
            self.move_forward(self.side_lengths[0])
            self.turn_90_degrees()
            self.move_forward(self.side_lengths[1])  
            self.turn_90_degrees()

    def robot_stop(self):
        twist=Twist()

        twist.angular.z=0.0
        twist.linear.x=0.0
        self.cmd_vel_publisher.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)
    rectangular_path_node = RectangularPath()
    
    try:
        while rclpy.ok():
            rectangular_path_node.run_path()
    except KeyboardInterrupt:
        rectangular_path_node.robot_stop()
        time.sleep(0.5)

    # Destroy the node
    rectangular_path_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
