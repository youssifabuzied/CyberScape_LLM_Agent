import json
import os
import sys
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage

# Load config.json
CONFIG_FILE = "config.json"
with open(CONFIG_FILE, "r") as f:
    config = json.load(f)

def load_file(file_path):
    """Helper function to read a file's content."""
    if os.path.exists(file_path):
        with open(file_path, "r") as f:
            return f.read().strip()
    return ""

def analyze_error(robot, phase, error_description):
    """Analyze the error and provide a textual suggestion for a fix."""
    spec_file = config[f"{robot.lower()}_spec_file"]
    mission_text_file = config["mission_text_file"]
    low_level_plan_file = config[f"{robot.lower()}_output_file"]

    # Load necessary files
    mission_text = load_file(mission_text_file)
    robot_specifications = load_file(spec_file)
    low_level_plan = json.loads(load_file(low_level_plan_file))
    failed_phase = next((p for p in low_level_plan["phases"] if p["phase_number"] == phase), None)
    
    if not failed_phase:
        return "Error: Phase not found in the low-level plan."
    
    prompt = f"""
    You are an expert in robotics and mission planning. A robot ({robot}) has encountered an error in Phase {phase} of its mission. 
    
    **Mission Scenario:**
    {mission_text}
    
    **Robot Specifications:**
    {robot_specifications}
    
    **Failed Phase Details:**
    - State: {failed_phase["state"]}
    - Goal: {failed_phase["phase_target"]}
    - Instructions:
    {failed_phase["low_level_plan"]}
    
    **Error Description:**
    {error_description}
    
    Provide a brief suggestion on how to modify the low-level plan to fix the issue.
    """
    
    llm = ChatOpenAI(model_name="gpt-4o", temperature=0)
    response = llm([SystemMessage(content="You are an expert in robotic planning."), HumanMessage(content=prompt)])
    
    return response.content.strip()

def fix_low_level_plan(robot, phase, fix_suggestion):
    """Modify the low-level plan based on OpenAI's suggested fix."""
    low_level_plan_file = config[f"{robot.lower()}_output_file"]
    low_level_plan = json.loads(load_file(low_level_plan_file))
    failed_phase = next((p for p in low_level_plan["phases"] if p["phase_number"] == phase), None)
    
    if not failed_phase:
        return "Error: Phase not found in the low-level plan."
    
    prompt = f"""
    You are a programming assistant. Update the low-level plan instructions for Phase {phase} of the {robot} mission.
    
    **Current Instructions:**
    {failed_phase["low_level_plan"]}
    
    **Fix Suggestion:**
    {fix_suggestion}
    
    Provide only the updated list of instructions, with no explanations or comments.
    """
    
    llm = ChatOpenAI(model_name="gpt-4o", temperature=0)
    response = llm([SystemMessage(content="You are an AI that edits robotic mission plans."), HumanMessage(content=prompt)])
    
    # Update the plan with the fixed instructions
    failed_phase["low_level_plan"] = response.content.strip()
    
    # Save back the modified plan
    with open(low_level_plan_file, "w") as f:
        json.dump(low_level_plan, f, indent=4)
    
    return "Low-level plan updated successfully."

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python APM.py <robot_name> <failed_phase_number> <error_description_file>")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    failed_phase_number = int(sys.argv[2])
    error_file = sys.argv[3]
    
    if not os.path.exists(error_file):
        print("Error: The specified error description file does not exist.")
        sys.exit(1)
    
    with open(error_file, "r") as f:
        error_data = json.load(f)
    
    error_description = error_data.get("description", "No description provided.")
    
    # Analyze error
    fix_suggestion = analyze_error(robot_name, failed_phase_number, error_description)
    print("Suggested Fix:", fix_suggestion)
    
    # Apply fix
    result = fix_low_level_plan(robot_name, failed_phase_number, fix_suggestion)
    print(result)
