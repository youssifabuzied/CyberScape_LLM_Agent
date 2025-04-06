from robot import Robot


class ExecutionModule:
    def __init__(self, robot: Robot):
        """
        Initialize the execution module with a robot.

        Args:
            robot: The robot to execute actions with
        """
        self.robot = robot

    def execute_action(self, action_name: str, *params) -> bool:
        """
        Execute an action using the robot's skills.

        Args:
            action_name: The name of the action to execute
            *params: Parameters for the action

        Returns:
            bool: True if the action was executed, False otherwise
        """
        print(f"Attempting to execute action: {action_name} with params: {params}")
        return self.robot.execute_skill(action_name, *params)

    class ExecutionModule:
        def __init__(self, robot: Robot):
            """
            Initialize the execution module with a robot.

            Args:
                robot: The robot to execute actions with
            """
            self.robot = robot

        def execute_action(self, action_name: str, object: str, *params) -> bool:
            """
            Execute an action using the robot's skills.

            Args:
                action_name: The name of the action to execute
                object: The target object of the action
                *params: Additional parameters for the action

            Returns:
                bool: True if the action was executed, False otherwise
            """
            print(f"Attempting to execute action: {action_name} on object: {object} with params: {params}")

            match action_name:
                case "GoToObject":
                    # Navigate to the object
                    pass
                case "OpenObject":
                    # Open the object
                    pass
                case "CloseObject":
                    # Close the object
                    pass
                case "BreakObject":
                    # Break the object
                    pass
                case "SliceObject":
                    # Slice the object
                    pass
                case "SwitchOn":
                    # Switch on the object
                    pass
                case "SwitchOff":
                    # Switch off the object
                    pass
                case "CleanObject":
                    # Clean the object
                    pass
                case "PickupObject":
                    # Pick up the object
                    pass
                case "PutObject":
                    # Put the object at specified location
                    pass
                case "DropHandObject":
                    # Drop the object from hand
                    pass
                case "ThrowObject":
                    # Throw the object
                    pass
                case "PushObject":
                    # Push the object
                    pass
                case "PullObject":
                    # Pull the object
                    pass
                case _:
                    print(f"Action {action_name} is not recognized.")
                    return False

            return self.robot.execute_skill(action_name, object, *params)


        def execute_phase(self, phase_plan: str) -> bool:
            """
            Execute a single phase of the mission.

            Args:
                phase_plan: String containing newline-separated actions to execute

            Returns:
                bool: True if phase is complete, False if there is an error
            """
            if not phase_plan:
                return True  # Empty phase is considered complete

            print(f"Executing phase for {self.robot.name}")

            # Split the plan into individual actions
            actions = phase_plan.strip().split('\n')

            for action_str in actions:
                if not action_str.strip():
                    continue

                # Parse the action string
                action_name, params = self._parse_action(action_str)
                if not action_name:
                    continue

                if not self.execute_action(action_name, *params):
                    return False

            return True  # Phase complete after executing all actions

        def _parse_action(self, action_str: str) -> tuple:
            """
            Parse an action string to get the action name and parameters.

            Args:
                action_str: String representation of the action

            Returns:
                tuple: (action_name, params)
            """
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
