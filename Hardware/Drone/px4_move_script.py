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


class DroneMoveController(Node):
    def __init__(self, target_distance=2.0, direction='forward', speed=0.5):
        super().__init__('drone_move_controller')
        
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
        self.initial_position = None
        self.target_distance = target_distance
        self.direction = direction
        self.speed = speed
        self.offboard_setpoint_counter = 0
        self.state = "INIT"
        self.movement_complete = False
        
        # Direction mapping (NED frame)
        self.direction_map = {
            'forward': (1, 0),   # x+
            'backward': (-1, 0), # x-
            'left': (0, -1),     # y-
            'right': (0, 1),     # y+
        }
        
        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Move controller initialized: {direction} {target_distance}m at {speed}m/s')
        
    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.current_position = np.array([msg.x, msg.y, msg.z])
        
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
        
    def timer_callback(self):
        """Main control loop"""
        self.offboard_setpoint_counter += 1
        
        # Publish offboard control heartbeat
        self.publish_offboard_control_mode()
        
        if self.state == "INIT":
            if self.current_position is not None:
                self.initial_position = self.current_position.copy()
                self.state = "MOVING"
                self.get_logger().info(f"Starting movement from position: {self.initial_position}")
                
        elif self.state == "MOVING":
            if self.current_position is not None and self.initial_position is not None:
                # Calculate movement vector
                dx, dy = self.direction_map[self.direction]
                
                # Calculate distance moved
                distance_vector = self.current_position[:2] - self.initial_position[:2]
                distance_moved = np.dot(distance_vector, [dx, dy])
                
                if distance_moved < self.target_distance:
                    # Calculate target position for smooth movement
                    current_progress = min(distance_moved + self.speed * 0.1, self.target_distance)
                    
                    target_x = self.initial_position[0] + dx * current_progress
                    target_y = self.initial_position[1] + dy * current_progress
                    target_z = self.current_position[2]  # Maintain altitude
                    
                    self.publish_trajectory_setpoint(target_x, target_y, target_z)
                    
                    # Progress update
                    if self.offboard_setpoint_counter % 20 == 0:
                        self.get_logger().info(
                            f'Progress: {distance_moved:.2f}m of {self.target_distance}m'
                        )
                else:
                    self.get_logger().info("Movement complete!")
                    self.movement_complete = True
                    self.state = "COMPLETE"
                    
        elif self.state == "COMPLETE" and self.movement_complete:
            # Script can exit when movement is complete
            self.get_logger().info("Move script completed successfully")
            self.destroy_timer(self.timer)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    distance = 2.0
    direction = 'forward'
    speed = 0.5
    
    if len(sys.argv) > 1:
        distance = float(sys.argv[1])
    if len(sys.argv) > 2:
        direction = sys.argv[2]
    if len(sys.argv) > 3:
        speed = float(sys.argv[3])
    
    # Validate direction
    valid_directions = ['forward', 'backward', 'left', 'right']
    if direction not in valid_directions:
        print(f"Invalid direction. Use one of: {valid_directions}")
        sys.exit(1)
    
    node = DroneMoveController(target_distance=distance, direction=direction, speed=speed)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
