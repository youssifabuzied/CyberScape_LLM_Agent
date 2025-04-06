import json
import math
import random


class MultiRobotTestingFramework:
    def __init__(self, mission_plan_path, drone_low_level_plan_path, dog_low_level_plan_path):
        # Load mission plans
        with open(mission_plan_path, 'r') as file:
            self.mission_plan = json.load(file)

        with open(drone_low_level_plan_path, 'r') as file:
            self.drone_low_level_plan = json.load(file)

        with open(dog_low_level_plan_path, 'r') as file:
            self.dog_low_level_plan = json.load(file)

        # Initialize robot positions
        self.drone_position = (5.0, 5.0, 10.0)  # (x, y, z)
        self.dog_position = (25.0, 25.0)  # (x, y)

        # Initialize field parameters
        self.field_size = 50  # 50x50 meter field
        self.dog_grid_size = 2  # 2x2 meter squares
        self.drone_grid_size = 10 # 10x10 meter squares

        # Initialize metrics
        self.drone_actions_total = 0
        self.drone_actions_executable = 0
        self.dog_actions_total = 0
        self.dog_actions_executable = 0

        self.tasks_total = 0
        self.tasks_completed = 0

        self.mission_success = False

        self.drone_area_covered = set()  # For grid cells covered by the drone
        self.dog_area_covered = set()  # For grid cells covered by the dog
        self.ball_found = False
        self.ball_retrieved = False

        # Shared data between robots
        self.shared_data = {
            "ball_detected": False,
            "LightSwitch": "ON",
            "area_coordinates": None
        }

        # Randomly place the ball in the field
        self.ball_position = (
            random.randint(0, self.field_size),
            random.randint(0, self.field_size)
        )
        print(f"Ball placed at: {self.ball_position}")

        # Load available actions
        self.drone_actions = self._load_drone_actions()
        self.dog_actions = self._load_dog_actions()

        # Phase tracking
        self.drone_current_phase = 0
        self.dog_current_phase = 0
        self.drone_waiting = False
        self.dog_waiting = False

        # Set robot detection ranges
        self.drone_range = 10
        self.dog_range = 3

        # Simulation control
        self.max_steps = 10  # Prevent infinite loops
        self.step_count = 0

    def _load_drone_actions(self):
        # Parse the drone_specifications.txt file
        return [
            "move_to_point", "rotate", "detect_with_camera",
            "wait_for_signal", "communicate_with_apm", "fly", "return_to_base"
        ]

    def _load_dog_actions(self):
        # Parse the dog_specifications.txt file
        return [
            "move_to_point", "rotate", "detect_with_camera",
            "wait_for_signal", "communicate_with_apm", "jump", "return_to_base", "retrieve_the_object"
        ]

    def _parse_action(self, action_str):
        """Parse an action string to get the action name and parameters"""
        if not action_str or '.' not in action_str:
            return None, []

        action_without_prefix = action_str.split('.', 1)[1]

        if '(' not in action_without_prefix:
            return action_without_prefix, []

        action_name = action_without_prefix.split('(')[0]
        params_str = action_without_prefix.split('(', 1)[1].rsplit(')', 1)[0]

        # Handle different parameter types
        if params_str.strip() == '':
            return action_name, []

        # Handle tuple parameters like (x, y, z)
        if params_str.startswith('(') and params_str.endswith(')'):
            inner_params = params_str[1:-1].split(',')
            return action_name, [p.strip() for p in inner_params]

        # Handle string parameters
        if params_str.startswith("'") or params_str.startswith('"'):
            return action_name, [params_str.strip("'\"")]

        # Handle numeric or other parameters
        return action_name, [params_str]

    def _is_action_executable(self, action_name, robot_type):
        """Check if an action is executable by the specified robot"""
        if robot_type == "drone":
            return action_name in self.drone_actions
        else:  # robot_type == "dog"
            return action_name in self.dog_actions

    def _update_position(self, robot_type, action_name, params):
        """Update robot position based on the action"""
        if action_name == "move_to_point":
            if robot_type == "drone":
                try:
                    # Assume params is already a tuple or list like (x, y, z)
                    if isinstance(params, (list, tuple)) and len(params) >= 3:
                        x, y, z = map(float, params)
                        self.drone_position = (x, y, z)
                    else:
                        # Fallback: if area coordinates are known, use them
                        if self.shared_data["area_coordinates"]:
                            x, y = self.shared_data["area_coordinates"]
                            self.drone_position = (x, y, self.drone_position[2])
                except Exception as e:
                    print(f"Error updating drone position: {e}")
            else:  # robot_type == "dog"
                try:
                    # Assume params is already a tuple or list like (x, y)
                    if isinstance(params, (list, tuple)) and len(params) >= 2 and isinstance(params[0], (int, float)):
                        x, y = map(float, params)
                        self.dog_position = (x, y)
                    else:
                        # Fallback: use area coordinates or ball location
                        if self.shared_data["area_coordinates"]:
                            x, y = self.shared_data["area_coordinates"]
                            self.dog_position = (x, y)
                        elif self.shared_data["ball_location"]:
                            x, y = self.shared_data["ball_location"]
                            self.dog_position = (x, y)

                    print(f"Dog position: {self.dog_position}")
                except Exception as e:
                    print(f"Error updating dog position: {e}")

        elif action_name == "fly" and len(params) == 1 and robot_type == "drone":
            try:
                height = float(params[0])
                # Simplified: just move in x direction
                x, y, z = self.drone_position
                self.drone_position = (x, y, height)
            except Exception as e:
                print(f"Error in fly: {e}")

    def _calculate_distance(self, position1, position2):
        """Calculate Euclidean distance between two points in 2D space"""
        try:
            # Extract x, y coordinates from positions
            x1 = position1[0]
            y1 = position1[1]
            x2 = position2[0]
            y2 = position2[1]

            # Calculate distance
            return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        except Exception as e:
            print(f"Error calculating distance: {e}")
            return float('inf')  # Return infinity if there's an error

    def _detect_ball(self, robot_type, action_name, params):
        """Simulate ball detection"""
        if action_name == "detect_with_camera" and "ball" in str(params).lower():
            # Calculate distance to the ball
            if robot_type == "drone":
                distance = self._calculate_distance(self.drone_position, self.ball_position)
                # Drone can detect from higher altitude, so detection range is larger
                detection_range = self.drone_range
                grid_size = self.drone_grid_size
                print(f"Drone at {self.drone_position} checking for ball at {self.ball_position}. Distance: {distance}")
            else:  # robot_type == "dog"
                distance = self._calculate_distance(self.dog_position, self.ball_position)
                detection_range = self.dog_range
                grid_size = self.dog_grid_size
                print(f"Dog at {self.dog_position} checking for ball at {self.ball_position}. Distance: {distance}")

            if distance <= detection_range:
                print(f"Ball detected by {robot_type} at distance {distance}!")
                self.ball_found = True
                self.shared_data["ball_detected"] = True
                self.shared_data["ball_location"] = self.ball_position


                # If drone detects the ball, calculate the area coordinates
                grid_x = int(self.ball_position[0] // grid_size)
                grid_y = int(self.ball_position[1] // grid_size)
                area_center_x = (grid_x * grid_size) + (grid_size / 2)
                area_center_y = (grid_y * grid_size) + (grid_size / 2)
                self.shared_data["area_coordinates"] = (area_center_x, area_center_y)

                # Signal to waiting robots
                self.drone_waiting = False
                self.dog_waiting = False

                return True
        return False

    def _handle_communication(self, robot_type, action_name, params):
        """Handle communication between robots"""
        if action_name == "communicate_with_apm":
            # If the drone is communicating area coordinates
            if robot_type == "drone" and self.shared_data["area_coordinates"]:
                # Signal to waiting robots
                self.drone_waiting = False
                self.dog_waiting = False

    def _update_area_covered(self, position, robot_type):
        """Update the set of grid cells covered for drone and dog with different grid sizes."""
        if robot_type == "drone":
            grid_size = self.drone_grid_size
            # For a drone, position is expected to be (x, y, z)
            x, y, *_ = position
            grid_x = int(x // grid_size)
            grid_y = int(y // grid_size)
            self.drone_area_covered.add((grid_x, grid_y))
        elif robot_type == "dog":
            grid_size = self.drone_grid_size
            # For a dog, position is expected to be (x, y)
            x, y = position[:2]
            grid_x = int(x // grid_size)
            grid_y = int(y // grid_size)
            self.dog_area_covered.add((grid_x, grid_y))

    def _execute_single_action(self, action_str, robot_type):
        """Execute a single action and return if waiting"""
        if not action_str.strip():
            return False

        action_name, params = self._parse_action(action_str)
        if not action_name:
            return False

        # Count total actions
        if robot_type == "drone":
            self.drone_actions_total += 1
        else:  # robot_type == "dog"
            self.dog_actions_total += 1

        # Check if action is executable
        if self._is_action_executable(action_name, robot_type):
            if robot_type == "drone":
                self.drone_actions_executable += 1
            else:  # robot_type == "dog"
                self.dog_actions_executable += 1

            # Check for wait signal
            if action_name == "wait_for_signal":
                if robot_type == "drone":
                    self.drone_waiting = True
                    print("Drone is waiting for this phase.")
                else:  # robot_type == "dog"
                    self.dog_waiting = True
                    print("Dog is waiting for this phase.")
                return True

            print(f"Executing action: {action_name} with params: {params}")

            # Update robot position
            self._update_position(robot_type, action_name, params)

            # Update area covered
            if robot_type == "drone":
                self._update_area_covered(self.drone_position, "drone")
            else:  # robot_type == "dog"
                self._update_area_covered(self.dog_position, "dog")

            # Check for ball detection
            self._detect_ball(robot_type, action_name, params)

            # Retrieve an object
            if robot_type == "dog" and action_name == "retrieve_the_object" and self.shared_data["ball_location"]:
                # If the dog is at the same cell
                self.ball_retrieved = True

            # Handle communication
            self._handle_communication(robot_type, action_name, params)

        return False

    def _execute_phase(self, phase_index, robot_type):
        """Execute a single phase of the mission"""
        if robot_type == "drone":
            phases = self.drone_low_level_plan.get('phases', [])
            if phase_index >= len(phases):
                return True  # Phase complete
            phase = phases[phase_index]
        else:  # robot_type == "dog"
            phases = self.dog_low_level_plan.get('phases', [])
            if phase_index >= len(phases):
                return True  # Phase complete
            phase = phases[phase_index]

        print(f"Executing {robot_type} phase {phase_index + 1}")

        if 'low_level_plan' not in phase:
            return True  # Phase complete

        plan = phase['low_level_plan']
        actions = plan.split('\n')

        for action_str in actions:
            waiting = self._execute_single_action(action_str, robot_type)
            if self.shared_data["ball_detected"]:
                # If ball is already detected, skip the phase
                return True
            if waiting:
                # If waiting for a signal, stay in the same phase
                return False

        return True  # Phase complete

    def run_simulation(self):
        """Run the full mission simulation with synchronized phases"""
        # Set task total (in this case, finding and retrieving the ball)
        self.tasks_total = 2  # 1) Find the ball, 2) Retrieve the ball

        # Get total phases
        drone_phases = len(self.drone_low_level_plan.get('phases', []))
        dog_phases = len(self.dog_low_level_plan.get('phases', []))

        # Simulation loop - continue until both robots complete all phases or max steps reached
        while (
                self.drone_current_phase < drone_phases or self.dog_current_phase < dog_phases) and self.step_count < self.max_steps:
            self.step_count += 1
            print(f"\n--- Step {self.step_count} ---")
            print(f"Drone phase: {self.drone_current_phase + 1}/{drone_phases}, waiting: {self.drone_waiting}")
            print(f"Dog phase: {self.dog_current_phase + 1}/{dog_phases}, waiting: {self.dog_waiting}")

            # Execute drone phase if not waiting and not completed
            if self.drone_current_phase < drone_phases and not self.drone_waiting:
                phase_complete = self._execute_phase(self.drone_current_phase, "drone")
                if phase_complete:
                    print(f"Drone completed phase {self.drone_current_phase + 1}")
                    self.drone_current_phase += 1

                    # Automatically break waiting state for robots when moving to next phase
                    self.drone_waiting = False
                    self.dog_waiting = False

            # Execute dog phase if not waiting and not completed
            if self.dog_current_phase < dog_phases and not self.dog_waiting:
                phase_complete = self._execute_phase(self.dog_current_phase, "dog")
                if phase_complete:
                    print(f"Dog completed phase {self.dog_current_phase + 1}")
                    self.dog_current_phase += 1

                    # Automatically break waiting state for robots when moving to next phase
                    self.drone_waiting = False
                    self.dog_waiting = False

            # If both robots are waiting, force detection if close enough
            if self.drone_waiting and self.dog_waiting and not self.ball_found:
                print("Both robots are waiting. Checking for forced detection...")
                # Calculate distance between drone and ball
                distance = self._calculate_distance(self.drone_position, self.ball_position)

                if distance <= self.drone_range:  # Drone detection range
                    print(f"Forced ball detection! Distance: {distance}")
                    self.ball_found = True
                    self.shared_data["ball_detected"] = True
                    self.shared_data["ball_location"] = self.ball_position

                    # Calculate area coordinates
                    grid_x = int(self.ball_position[0] // self.drone_grid_size)
                    grid_y = int(self.ball_position[1] // self.drone_grid_size)
                    area_center_x = (grid_x * self.drone_grid_size) + (self.drone_grid_size / 2)
                    area_center_y = (grid_y * self.drone_grid_size) + (self.drone_grid_size / 2)
                    self.shared_data["area_coordinates"] = (area_center_x, area_center_y)

                    # Signal to waiting robots
                    self.drone_waiting = False
                    self.dog_waiting = False

            # Check for deadlock conditions - if both waiting and ball is found
            if self.drone_waiting and self.dog_waiting and self.ball_found:
                print("Detected deadlock. Both robots waiting but ball already found. Breaking wait state.")
                self.drone_waiting = False
                self.dog_waiting = False

            # Additional deadlock prevention - random phase advancement
            if self.step_count > 0 and self.step_count % 50 == 0:
                if self.drone_waiting and self.dog_waiting:
                    print("Simulation stuck in waiting state for 50 steps. Breaking deadlock.")
                    self.drone_waiting = False
                    self.dog_waiting = False

        if self.step_count >= self.max_steps:
            print("Simulation reached maximum steps limit. Terminating.")

        # Count completed tasks
        if self.ball_found:
            self.tasks_completed += 1
        if self.ball_retrieved:
            self.tasks_completed += 1

        # Determine mission success
        self.mission_success = (self.tasks_completed == self.tasks_total)

        return self.calculate_metrics()

    def calculate_metrics(self):
        """Calculate and return all metrics, including overall area coverage without double-counting."""
        # 1. Executability
        drone_executability = self.drone_actions_executable / max(1, self.drone_actions_total)
        dog_executability = self.dog_actions_executable / max(1, self.dog_actions_total)
        overall_executability = (self.drone_actions_executable + self.dog_actions_executable) / \
                                max(1, (self.drone_actions_total + self.dog_actions_total))

        # 2. Task Completion Rate (TCR)
        tcr = self.tasks_completed / max(1, self.tasks_total)

        # 3. Success Rate (SR)
        sr = 1.0 if self.mission_success else 0.0

        # 4. Area Coverage (using dog grid of 2x2 m cells for finer resolution)
        total_dog_cells = (self.field_size // self.dog_grid_size) ** 2  # For 50x50 field: (50/2)^2 = 25^2 = 625 cells

        # Calculate drone coverage in dog grid coordinates
        drone_to_dog_cells = self.drone_grid_size // self.dog_grid_size
        drone_converted = set()
        for (i, j) in self.drone_area_covered:
            # Each drone cell (10x10 m) covers 5x5 dog grid cells
            for x in range(i * drone_to_dog_cells, (i + 1) * drone_to_dog_cells):
                for y in range(j * drone_to_dog_cells, (j + 1) * drone_to_dog_cells):
                    drone_converted.add((x, y))

        # Combine with dog's directly covered cells
        overall_covered = self.dog_area_covered.union(drone_converted)
        overall_coverage = len(overall_covered)

        # Optional: Also compute individual coverage metrics
        total_drone_cells = (self.field_size // self.drone_grid_size) ** 2  # Drone grid (10x10 m): e.g., 5x5 = 25 cells

        print("\n=== SIMULATION SUMMARY ===")
        print(f"Total steps: {self.step_count}")
        print(f"Ball found: {self.ball_found}")
        print(f"Ball retrieved: {self.ball_retrieved}")
        print(f"Tasks completed: {self.tasks_completed}/{self.tasks_total}")
        print(f"Mission success: {self.mission_success}")

        return {
            "executability": {
                "drone": drone_executability,
                "dog": dog_executability,
                "overall": overall_executability
            },
            "tcr": tcr,
            "sr": sr,
            "area_coverage": {
                "overall": overall_coverage,
            },
            "ball_position": self.ball_position,
            "final_drone_position": self.drone_position,
            "final_dog_position": self.dog_position,
            "covered_cells_drone": len(self.drone_area_covered),
            "total_cells_drone": total_drone_cells,
            "covered_cells_dog": len(self.dog_area_covered),
            "total_cells_dog": total_dog_cells,
            "steps_taken": self.step_count
        }

