#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from threading import Thread, Event, Lock
import time
import requests
import json
import subprocess
import sys
import os
import logging
import signal
import numpy as np
from geometry_msgs.msg import Twist
import math

# Configure logging
logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('mission_controller')

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
       
        # Parameters (could be loaded from ROS params)
        self.server_url = "http://localhost:8000/mission_status"  # URL to send status updates
        self.mission_endpoint = "http://localhost:8000/get_mission"  # URL to receive missions
        self.poll_interval = 5.0  # How often to check for new missions (seconds)
        self.execution_timeout = 120.0  # Default timeout for each action (seconds)
        self.position_tolerance = 0.05  # Tolerance for position checking (meters)
        self.rotation_tolerance = 0.05  # Tolerance for rotation checking (radians)
        self.obstacle_distance_threshold = 0.5  # Minimum distance to consider an obstacle (meters)
        self.stuck_threshold = 8.0  # Time to consider robot stuck if not moving (seconds)
        self.movement_check_window = 3.0  # Window of time to check for significant movement (seconds)
        self.min_movement_for_progress = 0.05  # Minimum movement to consider progress (meters)
       
        # Current mission state
        self.current_mission = None
        self.mission_id = None
        self.executing = False
        self.stop_requested = Event()
       
        # Robot state
        self.current_pose = None
        self.last_pose = None
        self.last_pose_update = None
        self.position_history = []
        self.scan_data = None
        self.obstacle_detected = False
        self.obstacle_direction = None  # Direction of detected obstacle (front, left, right)
        self.current_process = None
        self.initial_position = None      # Will store the starting position for verification
        self.process_lock = Lock()
        self.goal_distance = None  # Current goal distance during move action
        self.goal_rotation = None  # Current goal rotation during rotate action
        self.action_completed = False
        # Publishers for emergency stop
        self.stop_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
       
        # Subscribe to odometry to monitor robot movement
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
           
        # Subscribe to laser scan for obstacle detection
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
       
        # Path to script files
        self.script_paths = {
            "move": "/root/turtlebot3_ws/src/turtlebot3_mover/turtlebot3_mover/move_turtlebot3.py",
            "rotate": "/root/turtlebot3_ws/src/turtlebot3_mover/turtlebot3_mover/rotate_turtlebot.py",
            "detect": "/root/turtlebot3_ws/src/turtlebot3_yolov8_detector/turtlebot3_yolov8_detector/DetectOnj.py"
        }
       
        # Verify scripts exist and are executable
        self.verify_scripts()
       
        # Create a background thread for mission polling
        self.mission_thread = Thread(target=self.mission_polling_loop)
        self.mission_thread.daemon = True
        self.mission_thread.start()
       
        logger.info("Mission Controller initialized successfully")
   
    def verify_scripts(self):
        """Verify that all script files exist and are executable"""
        for action, path in self.script_paths.items():
            if not os.path.exists(path):
                logger.error(f"Script file not found: {path}")
            elif not os.access(path, os.X_OK):
                logger.warning(f"Script file not executable: {path}")
                try:
                    os.chmod(path, 0o755)  # Make executable
                    logger.info(f"Made script executable: {path}")
                except Exception as e:
                    logger.error(f"Failed to make script executable: {str(e)}")
            else:
                logger.info(f"Script verified: {path}")
   
    def get_yaw_from_pose(self, pose):
        """Extract yaw angle from a pose quaternion"""
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        """Callback for odometry messages to track robot position and movement"""
        self.last_pose = self.current_pose
        self.current_pose = msg.pose.pose
        
        # Update timestamp for movement monitoring
        current_time = time.time()
        self.last_pose_update = current_time
        
        # Add to position history for movement analysis
        if self.current_pose:
            # Extract yaw from quaternion
            yaw = self.get_yaw_from_pose(self.current_pose)
            
            pos = {
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'yaw': yaw,
                'time': current_time
            }
            self.position_history.append(pos)
            
            # Keep only recent positions (last 10 seconds)
            cutoff_time = current_time - 10.0
            self.position_history = [p for p in self.position_history if p['time'] > cutoff_time]
   
    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        self.scan_data = msg
        
        # Reset obstacle flags
        was_obstacle_detected = self.obstacle_detected
        self.obstacle_detected = False
        self.obstacle_direction = None
        
        # Check for obstacles if we have valid scan data
        if msg.ranges:
            # Define scan sections (front, left, right)
            num_ranges = len(msg.ranges)
            front_indices = list(range(int(num_ranges * 0.875), num_ranges)) + list(range(0, int(num_ranges * 0.125)))
            left_indices = list(range(int(num_ranges * 0.125), int(num_ranges * 0.375)))
            right_indices = list(range(int(num_ranges * 0.625), int(num_ranges * 0.875)))
            
            # Get valid ranges for each section
            front_ranges = [r for i, r in enumerate(msg.ranges) if i in front_indices and msg.range_min <= r <= msg.range_max]
            left_ranges = [r for i, r in enumerate(msg.ranges) if i in left_indices and msg.range_min <= r <= msg.range_max]
            right_ranges = [r for i, r in enumerate(msg.ranges) if i in right_indices and msg.range_min <= r <= msg.range_max]
            
            # MODIFIED: Only check for obstacles in front direction for stopping the robot
            if front_ranges and min(front_ranges) < self.obstacle_distance_threshold:
                self.obstacle_detected = True
                self.obstacle_direction = "front"
                min_distance = min(front_ranges)
                if not was_obstacle_detected:
                    logger.warning(f"Obstacle detected in FRONT at {min_distance:.2f}m")
                    
            # Still detect side obstacles for logging but don't set obstacle_detected flag
            elif left_ranges and min(left_ranges) < self.obstacle_distance_threshold * 0.7:
                # Note: Not setting self.obstacle_detected = True
                self.obstacle_direction = "left"  # Just for logging
                min_distance = min(left_ranges)
                if not was_obstacle_detected:
                    logger.info(f"Obstacle detected on LEFT at {min_distance:.2f}m (continuing movement)")
                    
            elif right_ranges and min(right_ranges) < self.obstacle_distance_threshold * 0.7:
                # Note: Not setting self.obstacle_detected = True
                self.obstacle_direction = "right"  # Just for logging
                min_distance = min(right_ranges)
                if not was_obstacle_detected:
                    logger.info(f"Obstacle detected on RIGHT at {min_distance:.2f}m (continuing movement)")
            
            # Log if obstacle cleared
            if was_obstacle_detected and not self.obstacle_detected:
                logger.info("Path is now clear of obstacles")
   
    def calculate_movement_rate(self, window_seconds=3.0):
        """
        Calculate how much the robot has moved in the last few seconds
        Returns movement rate in meters per second
        """
        if len(self.position_history) < 2:
            return 0.0
            
        current_time = time.time()
        cutoff_time = current_time - window_seconds
        
        # Get positions within our time window
        recent_positions = [p for p in self.position_history if p['time'] > cutoff_time]
        
        if len(recent_positions) < 2:
            return 0.0
            
        # Get the oldest and newest positions in our window
        oldest = min(recent_positions, key=lambda p: p['time'])
        newest = max(recent_positions, key=lambda p: p['time'])
        
        # Calculate distance moved
        dx = newest['x'] - oldest['x']
        dy = newest['y'] - oldest['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate time elapsed
        time_elapsed = newest['time'] - oldest['time']
        
        # Avoid division by zero
        if time_elapsed < 0.001:
            return 0.0
            
        # Return movement rate in meters per second
        return distance / time_elapsed

    def is_robot_making_progress(self):
        """
        Improved function to check if the robot is making significant progress
        Returns True if making progress, False if possibly stuck
        """
        # First check if we have enough position history data
        if len(self.position_history) < 3:  # Need at least 3 points for meaningful analysis
            return True  # Assume progress with insufficient data
            
        # Set appropriate window based on action type
        if self.goal_rotation is not None:
            # For rotation, use a shorter window
            window = min(2.0, self.movement_check_window)
        else:
            window = self.movement_check_window
        
        current_time = time.time()
        cutoff_time = current_time - window
        
        # Get positions within our time window
        recent_positions = [p for p in self.position_history if p['time'] > cutoff_time]
        
        if len(recent_positions) < 2:
            return True  # Not enough data to say it's stuck
            
        # Sort positions by time
        recent_positions.sort(key=lambda p: p['time'])
        
        # For rotation actions, check angular movement
        if self.goal_rotation is not None:
            # Calculate total angular change in the window
            total_angle_change = 0
            for i in range(1, len(recent_positions)):
                prev = recent_positions[i-1]
                curr = recent_positions[i]
                angle_diff = abs(self.normalize_angle(curr['yaw'] - prev['yaw']))
                total_angle_change += angle_diff
            
            # Convert to degrees per second
            time_span = recent_positions[-1]['time'] - recent_positions[0]['time']
            if time_span < 0.001:  # Avoid division by zero
                return True
                
            angular_speed = math.degrees(total_angle_change) / time_span
            
            # Different thresholds based on rotation amount
            min_angular_speed = 3.0  # degrees per second
            if abs(self.goal_rotation) < 30:
                min_angular_speed = 2.0  # Lower threshold for small rotations
                
            # Check for very recent stop
            oldest_time = recent_positions[0]['time']
            newest_time = recent_positions[-1]['time']
            if newest_time - oldest_time < 1.0:  # Very recent data
                # If we just started, give more time
                return True
                
            return angular_speed >= min_angular_speed
        
        # For move actions, analyze linear movement
        else:
            # Calculate total distance moved
            total_distance = 0
            for i in range(1, len(recent_positions)):
                prev = recent_positions[i-1]
                curr = recent_positions[i]
                dx = curr['x'] - prev['x']
                dy = curr['y'] - prev['y']
                segment_distance = math.sqrt(dx**2 + dy**2)
                total_distance += segment_distance
            
            # Calculate linear speed
            time_span = recent_positions[-1]['time'] - recent_positions[0]['time']
            if time_span < 0.001:  # Avoid division by zero
                return True
                
            linear_speed = total_distance / time_span
            
            # Use a lower threshold for movement detection - 1cm/second
            min_speed = 0.01  # meters per second (lowered from the original threshold)
            
            # Check for normal stop after process completion
            with self.process_lock:
                if self.current_process is None or self.current_process.poll() is not None:
                    # Process has completed - robot should be stopping normally
                    return True
            
            return linear_speed >= min_speed
   
    def check_path_clear(self, action, params):
        """Check if the path is clear before executing a move action"""
        if action != "move" or not self.scan_data:
            return True
            
        # MODIFIED: Only check for obstacles in front for move actions
        if self.obstacle_detected and self.obstacle_direction == "front":
            logger.warning(f"Cannot execute move command: obstacle detected in front at {self.get_front_obstacle_distance():.2f}m")
            return False
            
        return True
   
    def get_front_obstacle_distance(self):
        """Get the distance to the nearest obstacle in front"""
        if not self.scan_data or not self.scan_data.ranges:
            return float('inf')
           
        # Look at forward-facing scan points (front ~60 degree arc)
        num_ranges = len(self.scan_data.ranges)
        front_indices = list(range(int(num_ranges * 0.875), num_ranges)) + list(range(0, int(num_ranges * 0.125)))
        front_ranges = [r for i, r in enumerate(self.scan_data.ranges)
                       if i in front_indices and self.scan_data.range_min <= r <= self.scan_data.range_max]
       
        if not front_ranges:
            return float('inf')
           
        return min(front_ranges)
   
    def emergency_stop(self):
        """Stop the robot immediately by publishing zero velocity"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
       
        # Publish multiple times to ensure the message gets through
        for _ in range(5):
            self.stop_publisher.publish(stop_cmd)
            time.sleep(0.1)
           
        logger.warning("Emergency stop commanded")
   
    def mission_polling_loop(self):
        """Background thread that polls for new missions"""
        while rclpy.ok():
            if not self.executing:
                try:
                    # Poll for a new mission
                    response = requests.get(self.mission_endpoint)
                    if response.status_code == 200:
                        mission_data = response.json()
                       
                        # Check if we have a valid mission
                        if 'mission_id' in mission_data and 'instructions' in mission_data:
                            self.current_mission = mission_data['instructions']
                            self.mission_id = mission_data['mission_id']
                           
                            # Start mission execution in a separate thread
                            execution_thread = Thread(target=self.execute_mission)
                            execution_thread.daemon = True
                            execution_thread.start()
                           
                            logger.info(f"Starting mission {self.mission_id}")
                        else:
                            logger.warning("Received invalid mission format")
                except Exception as e:
                    logger.error(f"Error polling for mission: {str(e)}")
           
            # Wait before polling again
            time.sleep(5)
   
    def execute_mission(self):
        """Execute the current mission by parsing and running each instruction"""
        if not self.current_mission:
            return
            
        self.executing = True
        self.stop_requested.clear()
        
        mission_report = {
            "mission_id": self.mission_id,
            "status": "completed",
            "details": [],
            "timestamp": time.time(),
            "obstacle_detected": False,
            "obstacle_direction": None
        }
        
        try:
            # Parse the mission into individual actions
            lines = self.current_mission.strip().split('\n')
            
            # Print mission plan at the start
            logger.info("=" * 50)
            logger.info(f"MISSION EXECUTION PLAN ({self.mission_id}):")
            for i, line in enumerate(lines, 1):
                if line.strip():
                    logger.info(f"  [{i}] {line}")
            logger.info("=" * 50)
            
            # Track current step
            self.total_steps = sum(1 for line in lines if line.strip())
            self.current_step = 0
            
            for line_num, line in enumerate(lines, 1):
                if self.stop_requested.is_set():
                    mission_report["status"] = "aborted"
                    break
                    
                # Skip empty lines
                if not line.strip():
                    continue
                
                # Update and display progress
                self.current_step += 1
                logger.info("-" * 50)
                logger.info(f"EXECUTING STEP {self.current_step}/{self.total_steps}: {line}")
                logger.info("-" * 50)
                
                # Parse the action and parameters
                try:
                    # Parse the basic instruction format
                    parts = line.split('(', 1)
                    if len(parts) != 2 or not parts[1].endswith(')'):
                        raise ValueError(f"Invalid instruction format: {line}")
                        
                    action_name = parts[0].strip()
                    params_str = parts[1][:-1].strip()
                    
                    # Parse parameters
                    params = []
                    if params_str:
                        for param in params_str.split(','):
                            param = param.strip()
                            try:
                                if '.' in param:
                                    param = float(param)
                                else:
                                    param = int(param)
                            except ValueError:
                                if param.startswith('"') and param.endswith('"'):
                                    param = param[1:-1]
                                elif param.startswith("'") and param.endswith("'"):
                                    param = param[1:-1]
                            params.append(param)
                    
                    # Check if path is clear for move commands
                    if action_name == "move" and not self.check_path_clear(action_name, params):
                        # Report obstacle detection to server
                        obstacle_report = {
                            "line": line_num,
                            "instruction": line,
                            "status": "failed",
                            "error": f"Path blocked: obstacle detected in {self.obstacle_direction} direction",
                            "obstacle_detected": True,
                            "obstacle_direction": self.obstacle_direction,
                            "obstacle_distance": self.get_front_obstacle_distance()
                        }
                        mission_report["details"].append(obstacle_report)
                        mission_report["status"] = "failed"  # FIX: correctly mark as failed
                        mission_report["obstacle_detected"] = True
                        mission_report["obstacle_direction"] = self.obstacle_direction
                        
                        logger.error(f"STEP {self.current_step}/{self.total_steps} FAILED: Path blocked by obstacle")
                        break
                    
                    # Execute the action
                    action_report = self.execute_action(line, line_num)
                    mission_report["details"].append(action_report)
                    
                    # Report step status
                    if action_report["status"] == "completed":
                        logger.info(f"STEP {self.current_step}/{self.total_steps} COMPLETED SUCCESSFULLY")
                    else:
                        logger.error(f"STEP {self.current_step}/{self.total_steps} FAILED: {action_report.get('error', 'Unknown error')}")
                    
                    # If an action failed, mark the whole mission as failed
                    if action_report["status"] == "failed":
                        mission_report["status"] = "failed"
                        
                        # Include obstacle information if that was the cause
                        if "obstacle_detected" in action_report and action_report["obstacle_detected"]:
                            mission_report["obstacle_detected"] = True
                            mission_report["obstacle_direction"] = action_report.get("obstacle_direction", "unknown")
                        
                        break
                        
                except Exception as e:
                    error_msg = f"Error executing line {line_num}: {str(e)}"
                    logger.error(error_msg)
                    logger.error(f"STEP {self.current_step}/{self.total_steps} FAILED: {error_msg}")
                    mission_report["details"].append({
                        "line": line_num,
                        "instruction": line,
                        "status": "failed",
                        "error": error_msg
                    })
                    mission_report["status"] = "failed"
                    break
            
            # Report mission completion
            if mission_report["status"] == "completed":
                logger.info("=" * 50)
                logger.info(f"MISSION {self.mission_id} COMPLETED SUCCESSFULLY")
                logger.info("=" * 50)
            else:
                logger.error("=" * 50)
                logger.error(f"MISSION {self.mission_id} FAILED")
                logger.error(f"Completed {self.current_step}/{self.total_steps} steps")
                logger.error("=" * 50)
        
        except Exception as e:
            logger.error(f"Mission execution error: {str(e)}")
            mission_report["status"] = "failed"
            mission_report["error"] = str(e)
        
        finally:
            # Send the mission report back to the server
            self.send_mission_report(mission_report)
            
            # Reset mission state
            self.executing = False
            self.current_mission = None
            self.goal_distance = None
            self.goal_rotation = None
   
    def execute_action(self, instruction, line_num):
        """Execute a single action with time-based waiting and position verification"""
        action_report = {
            "line": line_num,
            "instruction": instruction,
            "status": "completed",  # Default to completed
            "start_time": time.time()
        }
    
        try:
            # Basic parsing - extract function name and parameters
            parts = instruction.split('(', 1)
            if len(parts) != 2 or not parts[1].endswith(')'):
                raise ValueError(f"Invalid instruction format: {instruction}")
            
            action_name = parts[0].strip()
            params_str = parts[1][:-1].strip()  # Remove trailing )
        
            # Parse parameters
            params = []
            if params_str:
                for param in params_str.split(','):
                    param = param.strip()
                    try:
                        if '.' in param:
                            param = float(param)
                        else:
                            param = int(param)
                    except ValueError:
                        if param.startswith('"') and param.endswith('"'):
                            param = param[1:-1]
                        elif param.startswith("'") and param.endswith("'"):
                            param = param[1:-1]
                    params.append(param)
        
            # Check if we have the script for this action
            if action_name not in self.script_paths:
                raise ValueError(f"Unknown action: {action_name}")
        
            script_path = self.script_paths[action_name]
            
            # Store initial position for verification later
            self.initial_position = None
            if self.current_pose and action_name in ["move", "rotate"]:
                self.initial_position = {
                    'x': self.current_pose.position.x,
                    'y': self.current_pose.position.y,
                    'yaw': self.get_yaw_from_pose(self.current_pose)
                }
                logger.info(f"Initial position: x={self.initial_position['x']:.2f}, y={self.initial_position['y']:.2f}, yaw={math.degrees(self.initial_position['yaw']):.2f}°")
        
            # Create monitoring thread for obstacle detection
            monitor_event = Event()
            monitor_thread = Thread(
                target=self.monitor_action,
                args=(action_name, params, monitor_event, action_report)
            )
            monitor_thread.daemon = True
            monitor_thread.start()
        
            # Execute the script with appropriate parameters
            if action_name == "move":
                # Move script takes a distance parameter
                distance = params[0] if params else 1.0
                distance = float(abs(distance))
                # Store goal distance for verification
                self.goal_distance = distance
                self.goal_rotation = None
                # Explicitly convert to float to ensure proper type
                cmd = [sys.executable, script_path, '--ros-args', '-p', f'target_distance:={distance}']
                
                # Calculate expected execution time based on distance
                # Assuming average speed of 0.2 m/s (adjust based on your robot's actual speed)
                move_speed = 0.2  # meters per second
                expected_time = distance / move_speed
                buffer_time = 5.0  # additional buffer in seconds
                total_wait_time = expected_time + buffer_time
                
                logger.info(f"Expected move time: {expected_time:.2f}s + {buffer_time:.2f}s buffer = {total_wait_time:.2f}s")
                
            elif action_name == "rotate":
                # Rotate script takes an angle parameter (in degrees)
                angle = params[0] if params else 90.0
                angle = float(angle)
                # Store goal rotation for verification
                self.goal_distance = None
                self.goal_rotation = angle
                # Explicitly convert to float to ensure proper type
                cmd = [sys.executable, script_path, '--ros-args', '-p', f'target_angle_deg:={angle}']
                
                # Calculate expected execution time based on angle
                # Assuming average rotation speed of 0.3 rad/s (17.2 deg/s) - adjust as needed
                # Using absolute angle since direction doesn't affect time
                rotate_speed = 17.2  # degrees per second
                expected_time = abs(angle) / rotate_speed
                buffer_time = 3.0  # additional buffer in seconds
                total_wait_time = expected_time + buffer_time
                
                logger.info(f"Expected rotation time: {expected_time:.2f}s + {buffer_time:.2f}s buffer = {total_wait_time:.2f}s")
                
            elif action_name == "detect":
                # Detect script (parameters depend on your implementation)
                object_type = params[0] if params else "ball"
                cmd = [sys.executable, script_path, '--ros-args', '-p', f'object_type:={object_type}']
                # Use a fixed timeout for detection actions
                total_wait_time = 15.0
                
            else:
                cmd = [sys.executable, script_path]
                # Default timeout for unknown actions
                total_wait_time = 30.0
        
            # Log the command
            logger.info(f"Executing: {' '.join(cmd)}")
        
            # Start the process
            with self.process_lock:
                start_time = time.time()
                self.current_process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
            
            # Wait for the calculated time
            try:
                logger.info(f"Waiting {total_wait_time:.2f}s for action to complete...")
                
                # Wait for the calculated time
                wait_end_time = time.time() + total_wait_time
                
                # Check periodically if the process has terminated or been stopped
                while time.time() < wait_end_time and not self.stop_requested.is_set():
                    # Check if process has already exited naturally
                    if self.current_process.poll() is not None:
                        logger.info(f"Process completed naturally with exit code {self.current_process.returncode}")
                        break
                    
                    # Sleep a short time before checking again
                    time.sleep(0.5)
                    
                # After the expected time, check if the goal was achieved
                success = False
                
                if self.stop_requested.is_set():
                    # Action was terminated due to an obstacle
                    logger.warning("Action terminated due to an obstacle")
                    success = False
                elif action_name in ["move", "rotate"] and self.initial_position and self.current_pose:
                    # Check if the move/rotate goal was achieved
                    current_pos = {
                        'x': self.current_pose.position.x,
                        'y': self.current_pose.position.y,
                        'yaw': self.get_yaw_from_pose(self.current_pose)
                    }
                    
                    logger.info(f"Current position: x={current_pos['x']:.2f}, y={current_pos['y']:.2f}, yaw={math.degrees(current_pos['yaw']):.2f}°")
                    
                    if action_name == "move" and self.goal_distance is not None:
                        dx = current_pos['x'] - self.initial_position['x']
                        dy = current_pos['y'] - self.initial_position['y']
                        distance_moved = math.sqrt(dx**2 + dy**2)
                        percentage = (distance_moved / self.goal_distance) * 100
                        
                        logger.info(f"Distance moved: {distance_moved:.2f}m of {self.goal_distance:.2f}m target ({percentage:.0f}%)")
                        
                        # Success if moved at least 70% of target
                        if distance_moved >= (self.goal_distance * 0.7):
                            logger.info(f"Move goal successfully achieved ({percentage:.0f}%)")
                            success = True
                        else:
                            logger.warning(f"Move goal not achieved, only moved {percentage:.0f}% of target")
                            success = False
                            
                    elif action_name == "rotate" and self.goal_rotation is not None:
                        angle_diff = self.normalize_angle(current_pos['yaw'] - self.initial_position['yaw'])
                        rotation_degrees = math.degrees(abs(angle_diff))
                        
                        # Check rotation direction
                        target_direction = 1 if self.goal_rotation > 0 else -1
                        actual_direction = 1 if angle_diff > 0 else -1
                        
                        # Calculate percentage, handling direction
                        percentage = (rotation_degrees / abs(self.goal_rotation)) * 100
                        
                        logger.info(f"Rotation: {rotation_degrees:.2f}° of {abs(self.goal_rotation):.2f}° target ({percentage:.0f}%)")
                        
                        # Success if rotated at least 70% of target in the right direction
                        direction_correct = (target_direction == actual_direction) or (abs(self.goal_rotation) < 10)
                        if rotation_degrees >= (abs(self.goal_rotation) * 0.7) and direction_correct:
                            logger.info(f"Rotation goal successfully achieved ({percentage:.0f}%)")
                            success = True
                        else:
                            if not direction_correct:
                                logger.warning(f"Rotation in wrong direction: expected {target_direction}, got {actual_direction}")
                            logger.warning(f"Rotation goal not achieved, only rotated {percentage:.0f}% of target")
                            success = False
                else:
                    # For other actions or if we don't have position data, assume success
                    logger.info(f"Position verification not applicable for {action_name}, assuming success")
                    success = True
                
                # Terminate the process if it's still running
                if self.current_process.poll() is None:
                    logger.info(f"Terminating process after position verification...")
                    self.current_process.terminate()
                    
                    # Give it a moment to terminate gracefully
                    try:
                        self.current_process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        # If it still doesn't terminate, kill it
                        logger.warning("Process didn't terminate gracefully, killing...")
                        self.current_process.kill()
                        self.current_process.wait(timeout=1.0)
                
                # Get process output
                stdout, stderr = self.current_process.communicate()
                
                # Update action status based on position verification
                if not success and not self.stop_requested.is_set():
                    action_report["status"] = "failed"
                    action_report["error"] = f"{action_name} goal not achieved after expected time"
                    
            except Exception as e:
                logger.error(f"Error during process wait: {str(e)}")
                if self.current_process.poll() is None:
                    self.current_process.terminate()
                    try:
                        self.current_process.wait(timeout=1.0)
                    except:
                        self.current_process.kill()
                    stdout, stderr = self.current_process.communicate()
                
                action_report["status"] = "failed"
                action_report["error"] = str(e)
                
            # Calculate elapsed time
            elapsed_time = time.time() - start_time
        
            # Signal monitoring thread to stop
            monitor_event.set()
            
            # Handle result based on whether we were interrupted
            if self.stop_requested.is_set():
                # Process was terminated due to obstacle detection
                logger.info(f"Action {action_name} was terminated due to obstacle detection")
                action_report["status"] = "failed"
                if "error" not in action_report:
                    action_report["error"] = "Action terminated due to obstacle detection"
            elif action_report["status"] == "completed":
                # Normal completion with successful position verification
                logger.info(f"Completed {action_name} successfully in {elapsed_time:.2f} seconds")
            
            # Include stdout/stderr for logging/debugging
            action_report["stdout"] = stdout if stdout else "No output"
            action_report["stderr"] = stderr if stderr else "No error output"
        
            # Update report with additional info
            action_report["elapsed_time"] = elapsed_time
            action_report["return_code"] = self.current_process.returncode
        
            # CRITICAL: Double-check that process is truly gone before proceeding
            if self.current_process and self.current_process.poll() is None:
                try:
                    logger.warning(f"Process still running after termination attempt, forcefully killing...")
                    self.current_process.kill()
                    self.current_process.wait(timeout=1.0)
                except Exception as e:
                    logger.error(f"Error while killing process: {str(e)}")
            
            with self.process_lock:
                self.current_process = None
        
            return action_report
            
        except Exception as e:
            elapsed_time = time.time() - action_report["start_time"]
            error_msg = f"Action execution error: {str(e)}"
            logger.error(error_msg)
        
            action_report["status"] = "failed"
            action_report["elapsed_time"] = elapsed_time
            action_report["error"] = error_msg
            
            # Ensure process is terminated in case of exception
            if hasattr(self, 'current_process') and self.current_process and self.current_process.poll() is None:
                try:
                    self.current_process.terminate()
                    time.sleep(0.5)
                    if self.current_process.poll() is None:
                        self.current_process.kill()
                except:
                    pass
                    
            return action_report
   
    def monitor_action(self, action_name, params, stop_event, action_report):
        """
        Monitoring function that only checks for obstacles
        """
        # Monitor only until the action completes or stop_event is set
        while not stop_event.is_set():
            # MODIFIED: Only stop for obstacles in front
            if action_name == "move" and self.obstacle_detected and self.obstacle_direction == "front":
                error_msg = f"Obstacle detected in front of robot"
                logger.warning(error_msg)
                
                action_report["status"] = "failed"
                action_report["error"] = error_msg
                action_report["obstacle_detected"] = True
                action_report["obstacle_direction"] = self.obstacle_direction
                
                # Terminate the action and emergency stop
                self._terminate_action_and_stop("obstacle")
                break
            
            # Sleep briefly to avoid consuming too much CPU
            time.sleep(0.1)


            
    def _terminate_action_and_stop(self, reason):
        """Helper to terminate current action and stop the robot"""
        # Terminate the current process if it exists
        with self.process_lock:
            if self.current_process and self.current_process.poll() is None:
                try:
                    self.current_process.terminate()
                    logger.info(f"Process terminated due to {reason}")
                    # Give it a moment to terminate
                    time.sleep(0.5)
                    # If still running, force kill
                    if self.current_process.poll() is None:
                        logger.warning(f"Process still running after terminate, killing...")
                        self.current_process.kill()
                        self.current_process.wait(timeout=1.0)
                except Exception as e:
                    logger.error(f"Error terminating process: {str(e)}")
        
        # Request mission abort
        self.stop_requested.set()
        
        # Emergency stop the robot
        self.emergency_stop()

    def check_goal_reached(self):
        """
        Check if the current movement goal has been reached with improved tolerance
        Returns True if goal has been reached, False otherwise
        """
        if not self.position_history or len(self.position_history) < 2:
            return False
            
        # Get start and current positions
        start_pos = self.position_history[0]
        current_pos = self.position_history[-1]

        # First, handle move actions (distance-based goals)
        if self.goal_distance is not None:
            # Calculate distance moved
            dx = current_pos['x'] - start_pos['x']
            dy = current_pos['y'] - start_pos['y']
            distance_moved = math.sqrt(dx**2 + dy**2)
            
            # Use more lenient threshold (75% of target)
            target_percentage = 0.75
            if distance_moved >= self.goal_distance * target_percentage:
                if not hasattr(self, '_distance_goal_logged') or not self._distance_goal_logged:
                    logger.info(f"Goal distance reached: moved {distance_moved:.2f}m of {self.goal_distance:.2f}m target ({distance_moved/self.goal_distance*100:.0f}%)")
                    logger.info("Goal has been reached, completing action")
                    self._distance_goal_logged = True
                return True
            else:
                self._distance_goal_logged = False
                
        # Then, handle rotate actions (rotation-based goals)
        elif self.goal_rotation is not None:
            # Calculate rotation difference (in radians)
            angle_diff = self.normalize_angle(current_pos['yaw'] - start_pos['yaw'])
            rotation_degrees = math.degrees(abs(angle_diff))
            
            # Determine rotation direction
            target_direction = 1 if self.goal_rotation > 0 else -1
            actual_direction = 1 if angle_diff > 0 else -1
            
            # Use more lenient threshold (75% of target)
            # For small rotations (<15 degrees), be even more lenient
            target_percentage = 0.75
            if abs(self.goal_rotation) < 15:
                target_percentage = 0.6
            
            # Ensure the rotation direction matches or if the rotation is very small (<10 degrees)
            if ((rotation_degrees >= abs(self.goal_rotation) * target_percentage) and 
                (target_direction == actual_direction or abs(self.goal_rotation) < 10)):
                if not hasattr(self, '_rotation_goal_logged') or not self._rotation_goal_logged:
                    logger.info(f"Goal rotation reached: rotated {rotation_degrees:.2f}° of {abs(self.goal_rotation):.2f}° target ({rotation_degrees/abs(self.goal_rotation)*100:.0f}%)")
                    logger.info("Goal has been reached, completing action")
                    self._rotation_goal_logged = True
                return True
            else:
                self._rotation_goal_logged = False
                
        return False


    def _handle_stuck_robot(self, action_name, start_time, elapsed_time, action_report):
        """Handle case where robot appears to be stuck"""
        # Before declaring stuck, do one final goal check with very lenient thresholds
        if action_name == "move" and self.goal_distance is not None:
            if len(self.position_history) >= 2:
                start_pos = self.position_history[0]
                current_pos = self.position_history[-1]
                dx = current_pos['x'] - start_pos['x']
                dy = current_pos['y'] - start_pos['y']
                distance_moved = math.sqrt(dx**2 + dy**2)
                
                # Super lenient check - if we've moved at least 65% of target, consider it done
                if distance_moved > (self.goal_distance * 0.65):
                    logger.info(f"Movement nearly complete ({distance_moved:.2f}m of {self.goal_distance:.2f}m - {distance_moved/self.goal_distance*100:.0f}%), accepting as successful")
                    return
        
        elif action_name == "rotate" and self.goal_rotation is not None:
            if len(self.position_history) >= 2:
                start_pos = self.position_history[0]
                current_pos = self.position_history[-1]
                angle_diff = abs(self.normalize_angle(current_pos['yaw'] - start_pos['yaw']))
                rotation_degrees = math.degrees(angle_diff)
                
                # Super lenient check - if we've rotated at least 65% of target, consider it done
                if rotation_degrees > (abs(self.goal_rotation) * 0.65):
                    logger.info(f"Rotation nearly complete ({rotation_degrees:.2f}° of {abs(self.goal_rotation):.2f}° - {rotation_degrees/abs(self.goal_rotation)*100:.0f}%), accepting as successful")
                    return
        
        # If we get here, robot is actually stuck
        movement_rate = self.calculate_movement_rate(self.movement_check_window)
        error_msg = f"Robot appears to be stuck (movement rate: {movement_rate:.3f} m/s)"
        logger.warning(error_msg)
        
        # Add position history to the report for debugging
        pos_history = [{
            'x': p['x'], 
            'y': p['y'], 
            'yaw': p['yaw'],
            'time_offset': p['time'] - start_time
        } for p in self.position_history[-10:]]  # Last 10 positions
        
        if pos_history:
            action_report["position_history"] = pos_history
        
        action_report["status"] = "failed"
        action_report["error"] = error_msg
        action_report["elapsed_time"] = elapsed_time
        
        # Terminate action and emergency stop
        self._terminate_action_and_stop("stuck")
    def send_mission_report(self, report):
        """Send the mission execution report back to the server"""
        try:
            # Add timestamp if not already present
            if "timestamp" not in report:
                report["timestamp"] = time.time()
                
            # Send the report
            logger.info(f"Sending mission report for mission {report['mission_id']}")
            response = requests.post(
                self.server_url,
                json=report,
                headers={"Content-Type": "application/json"}
            )
            
            if response.status_code == 200:
                logger.info("Mission report sent successfully")
            else:
                logger.error(f"Failed to send mission report: {response.status_code}")
                
        except Exception as e:
            logger.error(f"Error sending mission report: {str(e)}")
def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create and spin the node
    mission_controller = MissionController()
    
    try:
        rclpy.spin(mission_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        mission_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
