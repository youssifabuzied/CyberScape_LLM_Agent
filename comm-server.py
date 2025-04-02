from flask import Flask, request, jsonify
from flask_cors import CORS
import logging
import os
import json
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage

app = Flask(__name__)
CORS(app)  # Enable CORS if frontend or robots are communicating from different origins

# Setup logging
logging.basicConfig(level=logging.INFO)

# OpenAI API Key
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# Initialize LangChain Chat Model
llm = ChatOpenAI(model_name="gpt-4o-mini", openai_api_key=OPENAI_API_KEY)

# Load instructions from files
plans = {}
progress = {}
import subprocess
import json

@app.route("/generate_plan", methods=["POST"])
def generate_plan():
    data = request.get_json()

    if not data or "mission_title" not in data or "mission_text" not in data:
        return jsonify({"error": "Request must include 'mission_title' and 'mission_text'."}), 400

    mission_title = data["mission_title"]
    mission_text = data["mission_text"]
    
    # Define paths
    mission_files_dir = "mission_files"
    new_mission_file = os.path.join(mission_files_dir, f"{mission_title}.txt")
    
    # Ensure the directory exists
    os.makedirs(mission_files_dir, exist_ok=True)

    try:
        # Write the mission text to a new file
        with open(new_mission_file, "w") as f:
            f.write(mission_text)

        # Update config.json
        config_path = "config.json"
        with open(config_path, "r") as f:
            config = json.load(f)

        config["mission_text_file"] = new_mission_file  # Update the mission text file path

        with open(config_path, "w") as f:
            json.dump(config, f, indent=4)

        # Run the Manager.py script
        subprocess.run(["python", "Manager.py"], check=True)
        load_plan("ROBOT_DOG", "final_dog_low_level_plan.json")
        load_plan("DRONE", "final_drone_low_level_plan.json")
        return jsonify({"message": "Mission plan generation started successfully."}), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500

def load_plan(robot_name, filename):
    with open(os.path.join("Plans", filename)) as f:
        plans[robot_name] = json.load(f)
        progress[robot_name] = {
            "completed_phases": set(),
            "outputs": {}  # Stores outputs of completed phases
        }

load_plan("ROBOT_DOG", "final_dog_low_level_plan.json")
load_plan("DRONE", "final_drone_low_level_plan.json")

# Function to fill in variables in the low-level plan using LangChain
def fill_in_variables(plan_text, robot):
    """Replace placeholders in the plan text with actual values from completed phases using LangChain."""
    if robot not in progress:
        return plan_text  # No progress to apply

    stored_outputs = progress[robot]["outputs"]
    
    prompt = f"""
    You are a programming assistant. Replace placeholders in the following text with actual values from past phases.
    
    Text:
    {plan_text}

    Known Variables:
    {json.dumps(stored_outputs, indent=2)}

    Provide the modified text with variables substituted.
    ONLY MODIFY THE INSTRUCTIONS THAT INCLUDE VARIABLES AS PARAMETERS. DO NOT MODIFY INSTRUCTIONS WITH READY VALUES.
    DO NOT OUPUT ANY COMMENTS OR HEADERS IN THE MESSAGE. JUST LIST THE MODIFIED INTSTRUCTIONS.
    """
    
    response = llm.invoke([HumanMessage(content=prompt)])
    
    return response.content.strip()

@app.route("/complete_phase", methods=["POST"])
def complete_phase():
    data = request.get_json()

    if not data or "robot" not in data or "phase" not in data or "outputs" not in data:
        return jsonify({"error": "Request must include 'robot', 'phase', and 'outputs'."}), 400

    robot = data["robot"]
    phase = data["phase"]
    outputs = data["outputs"]

    try:
        phase = int(phase)
    except ValueError:
        return jsonify({"error": "Phase must be an integer."}), 400

    if robot not in plans:
        return jsonify({"error": f"No plan found for robot '{robot}'."}), 404

    phases = plans[robot]["phases"]
    matching_phase = next((p for p in phases if p["phase_number"] == phase), None)

    if not matching_phase:
        return jsonify({"error": f"Phase {phase} not found for robot {robot}."}), 404

    for p in range(1, phase):
        if p not in progress[robot]["completed_phases"]:
            return jsonify({"error": f"Phase {p} must be completed before marking phase {phase}."}), 400

    required_outputs = matching_phase.get("outputs", {})
    missing = [key for key in required_outputs if key not in outputs]
    if missing:
        return jsonify({"error": f"Missing required output values: {missing}"}), 400

    # Store outputs and mark phase as complete
    progress[robot]["completed_phases"].add(phase)
    progress[robot]["outputs"][phase] = outputs

    # Check if there's a next phase and if it has input variables
    next_phase = next((p for p in phases if p["phase_number"] == phase + 1), None)

    if next_phase:
        input_variables = next_phase.get("inputs", [])
        if input_variables:  # Only call fill_in_variables if the next phase has input variables
            logging.info(f"Updating low-level plan for phase {phase+1} of {robot}")
            updated_plan = fill_in_variables(next_phase["low_level_plan"], robot)
            next_phase["low_level_plan"] = updated_plan

    return jsonify({
        "message": f"Phase {phase} for {robot} marked as completed.",
        "stored_outputs": outputs
    }), 200


@app.route("/get_instruction", methods=["GET"])
def get_instruction():
    robot = request.args.get("robot")
    phase = request.args.get("phase")

    if not robot or not phase:
        return jsonify({"error": "Missing 'robot' or 'phase' query parameters."}), 400

    try:
        phase = int(phase)
    except ValueError:
        return jsonify({"error": "Phase must be an integer."}), 400

    if robot not in plans:
        return jsonify({"error": f"No plan found for robot '{robot}'."}), 404

    phases = plans[robot]["phases"]
    matching_phase = next((p for p in phases if p["phase_number"] == phase), None)

    if not matching_phase:
        return jsonify({"error": f"Phase {phase} not found for robot {robot}."}), 404

    for p in range(1, phase):
        if p not in progress[robot]["completed_phases"]:
            return jsonify({"error": f"Phase {p} must be completed before requesting phase {phase}."}), 400

    return jsonify({
        "phase_number": phase,
        "low_level_plan": matching_phase["low_level_plan"],
        "expected_outputs": matching_phase.get("outputs", {})
    }), 200

@app.route('/')
def index():
    return jsonify({"message": "Multi-Robot System Server is running"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
