#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint
)
import numpy as np
import sys
import math


class DroneRotateController(Node):
    def __init__(self, target_angle=90.0, angular_speed=30.0):
        super().__init__('drone_rotate_controller')
        
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        # Variables
        self.vehicle_local_position = None
        self.vehicle_status = VehicleStatus()
        self.current_position = None
        self.initial_heading = None
        self.current_heading = 0.0
        self.target_angle = math.radians(target_angle)  # Convert to radians
        self.angular_speed = math.radians(angular_speed)  # Convert to radians/s
        self.offboard_setpoint_counter = 0
        self.state = "INIT"
        self.rotation_complete = False
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Rotate controller initialized: {target_angle}° at {angular_speed}°/s')
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
        # Extract heading from quaternion
        q = msg.q
        # Convert quaternion to euler angles (yaw)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)
        
    def publish_trajectory_setpoint(self, x, y, z, yaw):
        """Publish trajectory setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        msg.yawspeed = float('nan')
        
        self.trajectory_setpoint_publisher.publish(msg)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
        
    def timer_callback(self):
        """Main control loop"""
        self.offboard_setpoint_counter += 1
        
        # Publish offboard control heartbeat
        self.publish_offboard_control_mode()
        
        if self.state == "INIT":
            if self.current_position is not None and self.current_heading is not None:
                self.initial_heading = self.current_heading
                self.state = "ROTATING"
                self.get_logger().info(f"Starting rotation from heading: {math.degrees(self.initial_heading):.1f}°")
                
        elif self.state == "ROTATING":
            if self.current_position is not None:
                # Calculate target yaw
                target_heading = self.initial_heading + self.target_angle
                target_heading = self.normalize_angle(target_heading)
                
                # Calculate how much we've rotated
                angle_rotated = self.normalize_angle(self.current_heading - self.initial_heading)
                
                if abs(self.normalize_angle(self.target_angle - angle_rotated)) > 0.05:  # 3 degrees tolerance
                    # Continue rotating
                    self.publish_trajectory_setpoint(
                        self.current_position[0],
                        self.current_position[1],
                        self.current_position[2],
                        target_heading
                    )
                    
                    # Progress update
                    if self.offboard_setpoint_counter % 10 == 0:
                        self.get_logger().info(
                            f'Rotation progress: {math.degrees(angle_rotated):.1f}° of {math.degrees(self.target_angle):.1f}°'
                        )
                else:
                    self.get_logger().info("Rotation complete!")
                    self.rotation_complete = True
                    self.state = "COMPLETE"
                    
        elif self.state == "COMPLETE" and self.rotation_complete:
            # Script can exit when rotation is complete
            self.get_logger().info("Rotate script completed successfully")
            self.destroy_timer(self.timer)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    angle = 90.0  # degrees
    speed = 30.0  # degrees per second
    
    if len(sys.argv) > 1:
        angle = float(sys.argv[1])
    if len(sys.argv) > 2:
        speed = float(sys.argv[2])
    
    node = DroneRotateController(target_angle=angle, angular_speed=speed)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
