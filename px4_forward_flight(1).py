#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from px4_msgs.msg import (
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint
)
import numpy as np
import time


class DroneForwardController(Node):
    def __init__(self, target_distance=2.0):
        super().__init__('drone_forward_controller')
        
        # Configure QoS profile for publishing to PX4
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
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.current_position = None
        self.initial_position = None
        self.target_distance = target_distance
        self.offboard_setpoint_counter = 0
        self.target_reached = False
        self.mission_state = "INIT"  # INIT, TAKEOFF, FORWARD, HOVER, LAND
        self.hover_altitude = 2.0  # meters
        self.forward_speed = 0.5  # m/s
        self.mission_start_time = None
        
        # Timer for publishing control commands
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        self.get_logger().info(f'Drone forward controller initialized with target distance: {target_distance}m')
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def arm(self):
        """Arm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")
        
    def disarm(self):
        """Disarm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")
        
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
        msg.position = [float(x), float(y), float(z)]  # NED frame
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)  # Radians
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
        
        # State machine for controlled mission execution
        if self.mission_state == "INIT":
            # Phase 1: Initialize and arm
            if self.offboard_setpoint_counter > 10:
                self.arm()
                self.mission_state = "ARMING"
                self.get_logger().info("Arming...")
                
        elif self.mission_state == "ARMING":
            # Wait for arming and establish initial position
            if self.vehicle_status.arming_state == 2:  # ARMED
                self.get_logger().info("Armed! Starting takeoff...")
                self.initial_position = self.current_position.copy()
                self.mission_state = "ENGAGE_OFFBOARD"
                self.engage_offboard_mode()
                
        elif self.mission_state == "ENGAGE_OFFBOARD":
            # Switch to offboard mode
            if self.offboard_setpoint_counter % 10 == 0:
                self.engage_offboard_mode()
                
            if self.vehicle_status.nav_state == 14:  # OFFBOARD
                self.get_logger().info("Offboard mode engaged! Taking off...")
                self.mission_state = "TAKEOFF"
                self.mission_start_time = self.get_clock().now()
                
        elif self.mission_state == "TAKEOFF":
            # Take off to hover altitude
            if self.current_position is not None and self.initial_position is not None:
                # Maintain current x,y position, climb to hover altitude
                self.publish_trajectory_setpoint(
                    self.initial_position[0],
                    self.initial_position[1],
                    -self.hover_altitude  # NED: down is positive
                )
                
                # Check if reached altitude
                if abs(self.current_position[2] + self.hover_altitude) < 0.2:
                    self.get_logger().info("Takeoff complete. Starting forward flight...")
                    self.mission_state = "FORWARD"
                    self.mission_start_time = self.get_clock().now()
                    
        elif self.mission_state == "FORWARD":
            # Move forward maintaining altitude
            if self.current_position is not None and self.initial_position is not None:
                # Calculate distance moved
                distance_moved = (self.current_position[0] - self.initial_position[0])
                
                # Calculate target position for smooth movement
                if distance_moved < self.target_distance:
                    # Move forward gradually
                    current_target = min(distance_moved + self.forward_speed * 0.1, self.target_distance)
                    
                    self.publish_trajectory_setpoint(
                        self.initial_position[0] + current_target,
                        self.initial_position[1],  # Stay at same y position
                        -self.hover_altitude  # Maintain altitude
                    )
                    
                    # Progress update
                    if self.offboard_setpoint_counter % 20 == 0:
                        self.get_logger().info(
                            f'Progress: {distance_moved:.2f}m of {self.target_distance}m'
                        )
                else:
                    self.get_logger().info("Target reached! Starting hover...")
                    self.mission_state = "HOVER"
                    self.mission_start_time = self.get_clock().now()
                    
        elif self.mission_state == "HOVER":
            # Hover for 3 seconds
            if self.current_position is not None and self.initial_position is not None:
                # Maintain final position
                self.publish_trajectory_setpoint(
                    self.initial_position[0] + self.target_distance,
                    self.initial_position[1],
                    -self.hover_altitude
                )
                
                # Check hover duration
                if (self.get_clock().now() - self.mission_start_time).nanoseconds / 1e9 > 3.0:
                    self.get_logger().info("Mission complete. Landing...")
                    self.mission_state = "LAND"
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main(args=None):
    rclpy.init(args=args)
    
    # Create node with target distance
    distance = float(input("Enter target distance (meters): "))
    node = DroneForwardController(target_distance=distance)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
