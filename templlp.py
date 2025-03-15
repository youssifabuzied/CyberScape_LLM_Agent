import json
import re
import argparse
import os
import subprocess
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from Manager import read_file

# Load environment variables from .env file
load_dotenv()

# ---------- Helper Functions for LLM Prompting ----------

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
            - ONLY use the functions provided in the specification for the drone. Do not invent or assume additional functions.
            - NO loops (for, while) and NO recursive calls.
            - Each phase must be independent, executing only after the previous phase completes.
            - If the drone needs to wait, explicitly include an idle state.
            - You are responsible for the translation of the high-level plan of the drone into a low-level set of executable drone instructions using ONLY the provded instructions. You won't be doing anything for the robot dog.  
            """
        )
    elif target.upper() == "ROBOT_DOG":
        return (
            """
            - ONLY use the functions provided in the specification for the robot dog. Do not invent or assume additional functions.
            - NO loops (for, while) and NO recursive calls.
            - Each phase must be independent, executing only after the previous phase completes.
            - If the robot dog needs to wait, explicitly include an idle state.
            - Every movement must be verified using `if RobotDog.has_reached(X, Y):` before proceeding.
            - You are responsible for the translation of the high-level plan of the robot dog into a low-level set of executable robot dog instructions using ONLY the provded instructions. You won't be doing anything for the drone.  
            """
        )
    else:
        return ""

def generate_low_level_for_plan(llm, mission_text, phases, robot_spec, target):
    """
    Makes a single LLM call for all phases for the given target.
    The prompt includes:
      - The original mission text,
      - A structured list of high-level phases,
      - The robot specifications,
      - Target-specific rules and an example output.
    The LLM is instructed to output a JSON object mapping phase numbers (as strings)
    to the low-level instruction block for that phase.
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

Output a JSON object where each key is the phase number (as a string) and each value is the low-level instruction block for that phase. Don't output any other thing. Don't add assumptions or any other text. Only the JSON. 
    """
    # print("LLM Prompt:\n", prompt)  # For debugging
    response = llm.invoke([
        SystemMessage(content="You are a helpful assistant."),
        HumanMessage(content=prompt)
    ])
    
    # Print raw LLM response before processing
    print("\n⚡ LLM Raw Response:\n", response.content)

    # Extract JSON block using regex (ensures correct extraction)
    json_match = re.search(r"```json\s*(\{.*?\})\s*```", response.content, re.DOTALL)
    if json_match:
        json_str = json_match.group(1).strip()
    else:
        # Handle cases where JSON is not inside a code block
        json_start = response.content.find("{")
        json_end = response.content.rfind("}")
        if json_start != -1 and json_end != -1:
            json_str = response.content[json_start:json_end + 1].strip()
        else:
            raise ValueError(f"🚨 Error: Could not find JSON in LLM response.\nRaw response:\n{response.content}")
    
    try:
        instructions_json = json.loads(json_str)
    except Exception as e:
        raise ValueError(f"🚨 Error parsing JSON output from LLM: {e}\nRaw extracted JSON:\n{json_str}")

    print("\n✅ Parsed Instructions JSON:\n", json.dumps(instructions_json, indent=2))  # Debugging

    return instructions_json

def update_plan_with_low_level(mission_plan, instructions_json, target):
    """
    For the given target, update each phase in the mission plan with the corresponding low-level instructions.
    Each low_level_plan will be stored as a list of instruction lines.
    """
    target_key = "drone_plan" if target.upper() == "DRONE" else "robot_dog_plan"
    for phase in mission_plan[target_key]["phases"]:
        phase_num = str(phase["phase_number"])
        if phase_num in instructions_json:
            # Split the instructions by newline so they become a list
            instructions = instructions_json[phase_num]
            if isinstance(instructions, str):
                phase["low_level_plan"] = instructions.splitlines()
            else:
                phase["low_level_plan"] = instructions
        else:
            phase["low_level_plan"] = []
    return mission_plan[target_key]

# ---------- Helper Functions for Verification and Parsing Steps ----------

def low_level_plan_to_text(phases):
    """
    Convert the list of phases (each with its "low_level_plan" attribute) into a clean, readable text format.
    Each phase is output as:
      Phase {number}:
      {low_level_plan}  <-- each instruction on a new line.
    """
    text = ""
    for phase in phases:
        # If the low_level_plan is a list, join it with actual newlines.
        instructions = phase.get("low_level_plan", "")
        if isinstance(instructions, list):
            instructions = "\n".join(instructions)
        text += f"Phase {phase['phase_number']}:\n{instructions}\n\n"
    return text


def parse_parsed_plan_text(text):
    """
    Parse the verified and parsed plan text (which should include lines like "Phase {number}:")
    into a dictionary mapping phase numbers (as strings) to instruction blocks.
    """
    pattern = re.compile(r"Phase (\d+):\s*(.*?)(?=Phase \d+:|$)", re.DOTALL)
    result = {}
    for match in pattern.finditer(text):
        phase_num = match.group(1).strip()
        instructions = match.group(2).strip()
        result[phase_num] = instructions
    return result

def run_subprocess_command(command, shell=True, cwd=r"\\wsl.localhost\Ubuntu\home\zein\Uni\Thesis\CyberScape_LLM_Agent"):
    """Runs a command using subprocess and waits for it to complete."""
    print(f"Running command: {command}")
    result = subprocess.run(command, shell=True)
    if result.returncode != 0:
        raise RuntimeError(f"Command failed with return code {result.returncode}")

# ---------- Main Function ----------

def main():
    parser = argparse.ArgumentParser(
        description="Generate low-level plans for a specified target (DRONE or ROBOT_DOG) from a high-level mission plan with verification and parsing steps."
    )
    parser.add_argument("mission_plan_file", help="Path to mission_plan.json (output from High_Level_Plan_Generator)")
    parser.add_argument("mission_text_file", help="Path to the original mission scenario file")
    parser.add_argument("spec_file", help="Path to the specifications file for the target robot")
    parser.add_argument("target", help="Target robot type: 'DRONE' or 'ROBOT_DOG'")
    parser.add_argument("output_file", help="Path to save the final verified and parsed low-level plan JSON for the target")
    parser.add_argument("verification_rubric_file", help="Path to the verification rubric file for low-level plans")
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

    # Initialize the LLM with configurable options
    llm = ChatOpenAI(
        # Option 1: Meta-Llama-3.1-405B-Instruct
        api_key=os.getenv("LLAMA_API_KEY"),
        base_url="https://api.sambanova.ai/v1",
        model_name="Meta-Llama-3.1-405B-Instruct",
        temperature=0.1

        # Option 2: GPT-4O-Mini
        # model_name="gpt-4o-mini",
        # openai_api_key=os.getenv("OPENAI_API_KEY"),
        # temperature=0.1

        # Option 3: Gemini-2.0-Flash
        # model_name="gemini-2.0-flash",
        # google_api_key=os.getenv("GOOGLE_API_KEY"),
        # temperature=0.1
    )

    # --------- Step 1: Generate Low-Level Instructions ---------
    instructions_json = generate_low_level_for_plan(llm, mission_text, phases, spec, args.target)
    updated_target_plan = update_plan_with_low_level(mission_plan, instructions_json, args.target)

    # --------- Step 2: Verification Step ---------
    # Convert the low-level plan for the target into text
    low_level_text = low_level_plan_to_text(updated_target_plan["phases"])
    temp_plan_file = "Plans/verified_and_parsed/temp_low_level_plan.txt"
    with open(temp_plan_file, "w") as f:
        f.write(low_level_text)

    # Call the Verification_Module on the low-level plan text
    # This assumes your Verification_Module.py has been adapted to work with low-level plan text
    temp_verified_file = "Plans/verified_and_parsed/temp_verified_plan.txt"
    verification_command = f"python3 Verification_Module.py {args.verification_rubric_file} {temp_plan_file} {args.mission_text_file} {temp_verified_file}"
    run_subprocess_command(verification_command, shell=True, cwd=r"\\wsl.localhost\Ubuntu\home\zein\Uni\Thesis\CyberScape_LLM_Agent")

    # --------- Step 3: Parsing Step ---------
    # Call the plan_parser on the verified plan text to clean/parse it
    # temp_parsed_file = "Plans/verified_and_parsed/temp_parsed_plan.txt"
    parsing_command = f"python3 plan_parser.py {temp_verified_file}"
    run_subprocess_command(parsing_command, shell=True, cwd=r"\\wsl.localhost\Ubuntu\home\zein\Uni\Thesis\CyberScape_LLM_Agent")
    # Assume the plan_parser outputs a file named "parsed_{temp_verified_file}"
    parsed_file = f"Plans/verified_and_parsed/parsed_temp_verified_plan.txt"
    with open(parsed_file, "r") as f:
        parsed_text = f.read()

    print("Parsed Text for Degbugging:", parsed_text)
    # Parse the text output into a dictionary mapping phase numbers to instructions
    parsed_instructions = parse_parsed_plan_text(parsed_text)

    # Update the JSON plan with the parsed instructions
    for phase in updated_target_plan["phases"]:
        phase_num = str(phase["phase_number"])
        if phase_num in parsed_instructions:
            phase["low_level_plan"] = parsed_instructions[phase_num]
        else:
            phase["low_level_plan"] = ""

    # --------- Final Step: Write the Final JSON Output ---------
    with open(args.output_file, "w") as f:
        json.dump(updated_target_plan, f, indent=4)

    print(f"Final verified and parsed low-level plan for {args.target.upper()} written to {args.output_file}")

if __name__ == "__main__":
    main()
