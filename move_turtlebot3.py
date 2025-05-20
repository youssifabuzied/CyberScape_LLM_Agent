#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

class MoveDistance(Node):
    def __init__(self):
        super().__init__('move')
        # Declare the target_distance parameter with a default value
        self.declare_parameter('target_distance', 1.0)
        self.target_distance = self.get_parameter('target_distance').value

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.initial_x = None
        self.initial_y = None
        self.moving = True

        # Timer for publishing movement commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        # Set a constant forward speed (adjust as needed)
        self.twist.linear.x = 0.2

    def odom_callback(self, msg):
        # Set initial position if not already set
        if self.initial_x is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.get_logger().info(f"Initial position set to: ({self.initial_x:.2f}, {self.initial_y:.2f})")
            return

        # Calculate Euclidean distance from the initial position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        distance_moved = math.sqrt((current_x - self.initial_x) ** 2 + (current_y - self.initial_y) ** 2)
        self.get_logger().info(f"Distance moved: {distance_moved:.2f} m")

        # Check if the robot has moved the desired distance
        if distance_moved >= self.target_distance and self.moving:
            self.get_logger().info("Target distance reached. Stopping the robot.")
            self.stop_robot()

    def timer_callback(self):
        # Only publish movement commands if the robot hasn't reached its target
        if self.moving:
            self.publisher_.publish(self.twist)

    def stop_robot(self):
        # Stop the robot by publishing zero velocities
        self.moving = False
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.publisher_.publish(stop_twist)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

