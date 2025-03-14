import json
import re
import argparse
import os
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from Manager import read_file

def get_example_output(target):
    """Return the target-specific example output."""
    if target.upper() == "DRONE":
        return (
            "Drone.move_to_point((100.0,200.0,50.0))\n"
            "Drone.scan_area()\n"
            "if Drone.scan_successful():\n"
            "    Drone.send_data_to_apm()"
        )
    elif target.upper() == "ROBOT_DOG":
        return (
            "RobotDog.move_to(100.0,200.0)\n"
            "RobotDog.scan_area()\n"
            "RobotDog.send_data_to_apm()"
        )
    else:
        return ""

def build_phases_text(phases):
    """
    Builds a text representation of all phases.
    Each phase is listed with its number, state, phase target, inputs, and outputs.
    """
    phases_text = ""
    for phase in phases:
        phases_text += f"Phase {phase['phase_number']}:\n"
        phases_text += f"  State: {phase['state']}\n"
        phases_text += f"  Phase Target: {phase['phase_target']}\n"
        phases_text += f"  Inputs: {phase['inputs']}\n"
        phases_text += f"  Outputs: {phase['outputs']}\n\n"
    return phases_text

def get_target_rules(target):
    """Return the rules for the target as a string."""
    if target.upper() == "DRONE":
        return (
            """
            - ONLY use the functions provided. Do not invent or assume additional functions.
            - NO loops (for, while) and NO recursive calls.
            - Each phase must be independent, executing only after the previous phase completes.
            - If the drone needs to wait, explicitly include an idle state.
            """
        )
    elif target.upper() == "ROBOT_DOG":
        return (
            """
            - ONLY use the functions provided. Do not invent or assume additional functions.
            - NO loops (for, while) and NO recursive calls.
            - Each phase must be independent, executing only after the previous phase completes.
            - If the robot dog needs to wait, explicitly include an idle state.
            - Every movement must be verified using `if RobotDog.has_reached(X, Y):` before proceeding.
            """
        )
    else:
        return ""

def generate_low_level_for_plan(llm, mission_text, phases, robot_spec, target):
    """
    Makes a single LLM call for all phases for the given target.
    The prompt includes the original mission text, a structured list of all high-level phases,
    the robot specifications, target-specific rules, and an example output.
    The LLM is instructed to output a JSON object mapping phase numbers to low-level instructions.
    """
    example_output = get_example_output(target)
    phases_text = build_phases_text(phases)
    rules = get_target_rules(target)
    prompt = f"""
Original Mission:
{mission_text}

Robot Specification for {target}:
{robot_spec}

Below are the high-level phases for the {target} plan:
{phases_text}

Rules for {target}:
{rules}

Example Output for {target} (for one phase):
{example_output}

Output a JSON object where each key is the phase number (as a string) and each value is the low-level instruction block for that phase.
    """
    #print the prompt to the console for debugging
    print(prompt)

    response = llm.invoke([
        SystemMessage(content="You are a helpful assistant."),
        HumanMessage(content=prompt)
    ])
    
    # Extract JSON code block from the response using regex
    # This regex looks for text between ```json and ```
    json_match = re.search(r"```json(.*?)```", response.content, re.DOTALL)
    if json_match:
        json_str = json_match.group(1).strip()
    else:
        json_str = response.content.strip()
    
    try:
        instructions_json = json.loads(json_str)
    except Exception as e:
        raise ValueError(f"Error parsing JSON output from LLM: {e}\nRaw response: {json_str}")
    
    return instructions_json

def update_plan_with_low_level(mission_plan, instructions_json, target):
    """
    For the given target, update each phase in the mission plan with the corresponding low-level instructions.
    
    How It Works:
    - It first determines the key for the target's plan (either "drone_plan" or "robot_dog_plan").
    - Then it iterates through each phase in that plan.
    - For each phase, it looks up the phase number (converted to a string) in the instructions_json dictionary.
    - If the phase number is present, it sets the "low_level_plan" attribute of that phase to the corresponding instruction block.
    - If the phase number is not found, it sets "low_level_plan" to an empty string.
    """
    target_key = "drone_plan" if target.upper() == "DRONE" else "robot_dog_plan"
    for phase in mission_plan[target_key]["phases"]:
        phase_num = str(phase["phase_number"])
        if phase_num in instructions_json:
            phase["low_level_plan"] = instructions_json[phase_num]
        else:
            phase["low_level_plan"] = ""
    return mission_plan[target_key]

def main():
    parser = argparse.ArgumentParser(
        description="Generate low-level plans for a specified target (DRONE or ROBOT_DOG) from a high-level mission plan with a single LLM call."
    )
    parser.add_argument("mission_plan_file", help="Path to mission_plan.json (output from High_Level_Plan_Generator)")
    parser.add_argument("mission_text_file", help="Path to the original mission scenario file")
    parser.add_argument("spec_file", help="Path to the specifications file for the target robot")
    parser.add_argument("target", help="Target robot type: 'DRONE' or 'ROBOT_DOG'")
    parser.add_argument("output_file", help="Path to save the updated low-level plan JSON for the target")
    args = parser.parse_args()

    # Read the high-level mission plan JSON
    with open(args.mission_plan_file, "r") as f:
        mission_plan = json.load(f)

    # Read the original mission text and the robot specifications
    mission_text = read_file(args.mission_text_file)
    spec = read_file(args.spec_file)

    # Determine the key for the target's plan and retrieve its phases
    target_key = "drone_plan" if args.target.upper() == "DRONE" else "robot_dog_plan"
    phases = mission_plan[target_key]["phases"]
    
    load_dotenv()

    # Initialize the LLM with configurable options (uncomment your desired configuration)
    llm = ChatOpenAI(
        # Option 1: Meta-Llama-3.1-405B-Instruct
        # api_key=os.getenv("LLAMA_API_KEY"),
        # base_url="https://api.sambanova.ai/v1",
        # model_name="Meta-Llama-3.1-405B-Instruct",
        # temperature=0.1
        # Option 2: GPT-4O-Mini
        model_name="gpt-4o-mini",
        openai_api_key=os.getenv("OPENAI_API_KEY"),
        temperature=0.1
        # Option 3: Gemini-2.0-Flash
        # model_name="gemini-2.0-flash",
        # google_api_key=os.getenv("GOOGLE_API_KEY"),
        # temperature=0.1
    )

    # Make one single LLM call for all phases for the specified target
    instructions_json = generate_low_level_for_plan(llm, mission_text, phases, spec, args.target)

    # Update the mission plan with the low-level instructions for each phase
    updated_target_plan = update_plan_with_low_level(mission_plan, instructions_json, args.target)

    # Write the updated target plan to a new JSON file
    with open(args.output_file, "w") as f:
        json.dump(updated_target_plan, f, indent=4)

    print(f"Low-level plan for {args.target.upper()} written to {args.output_file}")

if __name__ == "__main__":
    main()

# python Low_Level_Planner.py Plans\mission_plan.json mission_files\mission_scenario.txt  mission_files/drone_specifications.txt DRONE Plans\updated_drone_mission_plan.json
# python Low_Level_Planner.py Plans\mission_plan.json mission_files\mission_scenario.txt  mission_files/dog_specifications.txt DOG Plans\updated_dog_mission_plan.json