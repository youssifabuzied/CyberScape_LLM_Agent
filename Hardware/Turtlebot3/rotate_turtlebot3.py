#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class RotateTurtlebot(Node):
    def __init__(self):
        super().__init__('rotate')
        # Declare parameters with default values.
        self.declare_parameter('target_angle_deg', 90.0)  # Default: 90 degrees
        self.declare_parameter('angular_speed', 0.2)        # Default speed (rad/s)

        # Read parameters
        target_angle_deg = self.get_parameter('target_angle_deg').value
        # Convert target angle to radians.
        self.target_angle = math.radians(target_angle_deg)
        angular_speed = self.get_parameter('angular_speed').value

        # Determine rotation direction based on target_angle sign.
        if self.target_angle < 0:
            self.twist_speed = -abs(angular_speed)
        else:
            self.twist_speed = abs(angular_speed)

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Subscriber to odometry data
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # For cumulative rotation tracking
        self.last_yaw = None
        self.cumulative_angle = 0.0
        self.moving = True

        # Timer to publish velocity commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist = Twist()
        self.twist.angular.z = self.twist_speed

    def odom_callback(self, msg):
        # Convert current orientation to yaw (using quaternion)
        # Since we're only using yaw, we can compute it directly.
        q = msg.pose.pose.orientation
        # Formula for yaw from quaternion:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_yaw is None:
            self.last_yaw = current_yaw
            self.get_logger().info(f"Initial yaw set to: {self.last_yaw:.2f} rad")
            return

        # Compute the change since last message (normalized to account for wrapping)
        delta = normalize_angle(current_yaw - self.last_yaw)
        self.cumulative_angle += delta
        self.last_yaw = current_yaw

        self.get_logger().info(f"Cumulative rotation: {abs(self.cumulative_angle):.2f} rad (target: {abs(self.target_angle):.2f} rad)")

        # If the accumulated rotation (in absolute value) exceeds the target, stop.
        if abs(self.cumulative_angle) >= abs(self.target_angle) and self.moving:
            self.get_logger().info("Target rotation reached. Stopping the robot.")
            self.stop_robot()

    def timer_callback(self):
        if self.moving:
            self.publisher_.publish(self.twist)

    def stop_robot(self):
        self.moving = False
        stop_twist = Twist()
        stop_twist.angular.z = 0.0
        self.publisher_.publish(stop_twist)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = RotateTurtlebot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
