#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock
from px4_msgs.msg import (
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    VehicleLandDetected,
    OffboardControlMode,
    TrajectorySetpoint
)
import numpy as np
import sys
import time


class DroneLandController(Node):
    def __init__(self, landing_speed=0.3):
        super().__init__('drone_land_controller')
        
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
        
        self.vehicle_land_detected_subscriber = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.vehicle_land_detected_callback,
            qos_profile
        )
        
        # Variables
        self.vehicle_local_position = None
        self.vehicle_status = VehicleStatus()
        self.vehicle_land_detected = VehicleLandDetected()
        self.current_position = None
        self.landing_speed = landing_speed
        self.offboard_setpoint_counter = 0
        self.state = "INIT"
        self.landing_complete = False
        self.landing_start_time = None
        self.disarm_attempts = 0
        self.altitude_threshold = 0.05  # 5cm threshold for ground detection
        
        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz for smoother control
        
        self.get_logger().info(f'Land controller initialized. Landing speed: {landing_speed}m/s')
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def vehicle_land_detected_callback(self, msg):
        self.vehicle_land_detected = msg
        
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
        
    def disarm(self):
        """Disarm the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.disarm_attempts += 1
        self.get_logger().info(f"Disarm command sent (attempt {self.disarm_attempts})")
        
    def timer_callback(self):
        """Main control loop"""
        self.offboard_setpoint_counter += 1
        
        # Always publish offboard control heartbeat to prevent mode switching
        self.publish_offboard_control_mode()
        
        if self.state == "INIT":
            if self.current_position is not None:
                # Wait for stable position reading
                if self.offboard_setpoint_counter > 5:
                    self.landing_start_time = self.get_clock().now()
                    self.state = "CONTROLLED_DESCENT"
                    current_altitude = -self.current_position[2]
                    self.get_logger().info(f"Starting controlled descent from altitude: {current_altitude:.2f}m")
            else:
                # Stay at current position for initial readings
                if self.offboard_setpoint_counter > 0:
                    self.publish_trajectory_setpoint(0, 0, 0)  # Maintain
                
        elif self.state == "CONTROLLED_DESCENT":
            if self.current_position is not None:
                current_altitude = -self.current_position[2]
                
                # Check if we should stop descending
                if current_altitude <= self.altitude_threshold or self.vehicle_land_detected.landed:
                    self.get_logger().info(f"Ground detected at altitude: {current_altitude:.3f}m")
                    self.state = "GROUND_CONTACT"
                else:
                    # Continue controlled descent
                    descent_rate = self.landing_speed * 0.05  # Per timer interval (20Hz)
                    target_z = self.current_position[2] + descent_rate  # Descend in NED frame
                    
                    self.publish_trajectory_setpoint(
                        self.current_position[0],
                        self.current_position[1],
                        target_z,
                        0.0  # Maintain heading
                    )
                    
                    # Progress update every second
                    if self.offboard_setpoint_counter % 20 == 0:
                        self.get_logger().info(f'Descending - Altitude: {current_altitude:.2f}m')
                
        elif self.state == "GROUND_CONTACT":
            # Hold position on ground for a moment before disarming
            self.publish_trajectory_setpoint(
                self.current_position[0],
                self.current_position[1],
                self.current_position[2],
                0.0
            )
            
            # Wait a bit to ensure stable ground contact
            if (self.get_clock().now() - self.landing_start_time).nanoseconds / 1e9 > 0.5:
                self.get_logger().info("Ground contact stable, initiating disarm...")
                self.state = "DISARMING"
                self.disarm()
                
        elif self.state == "DISARMING":
            # Keep sending disarm command until successful
            if self.vehicle_status.arming_state == 1:  # DISARMED
                self.get_logger().info("Disarm successful! Landing complete")
                self.landing_complete = True
                self.state = "COMPLETE"
            elif self.disarm_attempts < 10:
                # Continue trying to disarm
                if self.offboard_setpoint_counter % 5 == 0:  # Every 0.25 seconds
                    self.disarm()
            else:
                self.get_logger().error("Failed to disarm after 10 attempts")
                self.state = "COMPLETE"
                
        elif self.state == "COMPLETE":
            # Script can exit
            self.get_logger().info("Landing script completed")
            # Give a short delay before shutdown
            if self.offboard_setpoint_counter - self.landing_complete > 10:
                self.destroy_timer(self.timer)
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    speed = 0.3  # Default to slower speed for safer landing
    
    if len(sys.argv) > 1:
        speed = float(sys.argv[1])
    
    node = DroneLandController(landing_speed=speed)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
