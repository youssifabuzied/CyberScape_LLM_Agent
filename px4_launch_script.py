#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock
from px4_msgs.msg import (
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint
)
import numpy as np
import sys


class DroneLaunchController(Node):
    def __init__(self, hover_altitude=2.0):
        super().__init__('drone_launch_controller')
        
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )
        
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
        self.initial_position = None
        self.target_altitude = hover_altitude
        self.offboard_setpoint_counter = 0
        self.state = "INIT"
        self.launched = False
        
        # Timer for publishing control commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Drone launch controller initialized. Target altitude: {hover_altitude}m')
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def arm(self):
        """Arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")
        
    def engage_offboard_mode(self):
        """Switch to offboard mode"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode command sent")
        
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
        
    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
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
        
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish a vehicle command"""
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)
        
    def timer_callback(self):
        """Main control loop"""
        self.offboard_setpoint_counter += 1
        
        # Publish offboard control heartbeat
        self.publish_offboard_control_mode()
        
        if self.state == "INIT":
            if self.offboard_setpoint_counter > 10:
                self.arm()
                self.state = "ARMING"
                self.get_logger().info("Arming...")
                
        elif self.state == "ARMING":
            if self.vehicle_status.arming_state == 2:  # ARMED
                self.get_logger().info("Armed! Starting takeoff...")
                self.initial_position = self.current_position.copy()
                self.state = "ENGAGE_OFFBOARD"
                self.engage_offboard_mode()
                
        elif self.state == "ENGAGE_OFFBOARD":
            if self.offboard_setpoint_counter % 10 == 0:
                self.engage_offboard_mode()
                
            if self.vehicle_status.nav_state == 14:  # OFFBOARD
                self.get_logger().info("Offboard mode engaged! Taking off...")
                self.state = "TAKEOFF"
                
        elif self.state == "TAKEOFF":
            if self.current_position is not None and self.initial_position is not None:
                # Maintain current x,y position, climb to target altitude
                self.publish_trajectory_setpoint(
                    self.initial_position[0],
                    self.initial_position[1],
                    -self.target_altitude  # NED frame
                )
                
                # Check if reached altitude
                if abs(self.current_position[2] + self.target_altitude) < 0.2:
                    self.get_logger().info(f"Launch complete! Hovering at {self.target_altitude}m")
                    self.launched = True
                    self.state = "COMPLETE"
                    
        elif self.state == "COMPLETE" and self.launched:
            # Script can exit when launched
            self.get_logger().info("Launch script completed successfully")
            self.destroy_timer(self.timer)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Default altitude or get from command line
    altitude = 2.0
    if len(sys.argv) > 1:
        altitude = float(sys.argv[1])
    
    node = DroneLaunchController(hover_altitude=altitude)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
