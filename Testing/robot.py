import json
from typing import Dict, List, Any


class Robot:
    def __init__(self, config_path: str):
        """Initialize a robot from a configuration file."""
        with open(config_path, 'r') as file:
            config = json.load(file)

        self.name: str = config.get('name', 'Unnamed Robot')
        self.skills: List[str] = config.get('skills', [])
        self.additional_info: Dict[str, Any] = config.get('additional_info', {})

    def execute_skill(self, skill_name: str, *args) -> bool:
        """
        Execute a skill if available.

        Args:
            skill_name: The name of the skill to execute
            *args: Arguments for the skill

        Returns:
            bool: True if the skill was executed, False otherwise
        """
        if skill_name in self.skills:
            print(f"Executing {skill_name} with args {args} for {self.name}")
            return True
        else:
            print(f"Skill {skill_name} not available for {self.name}")
            return False
