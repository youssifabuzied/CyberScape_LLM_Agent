import json
from typing import Dict, List, Any

from execution_module import ExecutionModule
from robot import Robot


class SimulationFramework:
    def __init__(self, mission_config_path: str, robot_configs: List[str]):
        """
        Initialize the simulation framework with mission and robot configurations.

        Args:
            mission_config_path: Path to the mission configuration file
            robot_configs: List of paths to robot configuration files
        """
        with open(mission_config_path, 'r') as file:
            self.mission_config = json.load(file)

        self.robots: List[Robot] = [Robot(config_path) for config_path in robot_configs]
        self.steps: int = 0
        self.max_steps: int = self.mission_config.get('max_steps', 10)

        # Initialize state tracking
        self.state: Dict[str, Any] = {
            "robot_positions": [robot.position for robot in self.robots],
            "area_coverage": set()
        }

        # Add any additional states from the mission config
        for state in self.mission_config.get('monitored_states', []):
            if state not in self.state:
                self.state[state] = None

    def update_state(self) -> None:
        """Update the simulation state."""
        # Update robot positions
        self.state["robot_positions"] = [robot.position for robot in self.robots]

        # Additional state updates would go here

    def check_final_states(self) -> bool:
        """
        Check if the final states have been reached.

        Returns:
            bool: True if all final states have been reached, False otherwise
        """
        final_states = self.mission_config.get('final_states', {})
        for state_name, state_value in final_states.items():
            if self.state.get(state_name) != state_value:
                return False
        return True

    def run_simulation(self) -> Dict[str, Any]:
        """
        Run the simulation.

        Returns:
            Dict[str, Any]: Results of the simulation
        """
        print(f"Starting simulation: {self.mission_config['mission_name']}")
        mission_complete = False

        while self.steps < self.max_steps and not mission_complete:
            print(f"Step {self.steps + 1}")
            for robot in self.robots:
                execution_module = ExecutionModule(robot)
                for action in self.mission_config.get('actions', []):
                    execution_module.execute_action(action['name'], *action.get('params', []))

            # Update simulation state
            self.update_state()

            # Check if mission is complete
            mission_complete = self.check_final_states()

            self.steps += 1

        result = {
            "mission_name": self.mission_config['mission_name'],
            "steps_taken": self.steps,
            "mission_complete": mission_complete,
            "final_state": self.state
        }

        print(f"Simulation complete. Mission success: {mission_complete}")
        return result


def run_simulation_multiple_configs(configs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Run the simulation multiple times with different configurations.

    Args:
        configs: List of configuration dictionaries

    Returns:
        List[Dict[str, Any]]: Results of all simulations
    """
    results = []

    for i, config in enumerate(configs):
        print(f"\n=== Running Simulation {i + 1} ===")
        # Save robot configuration to file
        robot_config_paths = []
        for j, robot_config in enumerate(config['robot_configs']):
            config_path = f'robot_config_{i}_{j}.json'
            with open(config_path, 'w') as file:
                json.dump(robot_config, file, indent=4)
            robot_config_paths.append(config_path)

        # Save mission configuration to file
        mission_config_path = f'mission_config_{i}.json'
        with open(mission_config_path, 'w') as file:
            json.dump(config['mission_config'], file, indent=4)

        # Run the simulation
        simulation = SimulationFramework(mission_config_path, robot_config_paths)
        result = simulation.run_simulation()
        results.append(result)

    return results
