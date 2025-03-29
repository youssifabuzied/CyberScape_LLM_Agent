# server.py

from flask import Flask, request, jsonify
from flask_cors import CORS
import logging
import os
import json
app = Flask(__name__)
CORS(app)  # Enable CORS if frontend or robots are communicating from different origins

# Setup logging
logging.basicConfig(level=logging.INFO)


# Load instructions from files
plans = {}
progress = {}

def load_plan(robot_name, filename):
    with open(os.path.join("plans", filename)) as f:
        plans[robot_name] = json.load(f)
        progress[robot_name] = {
            "completed_phases": set(),
            "outputs": {}  # To store outputs of each completed phase
        }

load_plan("ROBOT_DOG", "/home/youssif-abuzied/thesis_project/CyberScape_LLM_Agent/Plans/final_dog_low_level_plan.json")
load_plan("DRONE", "/home/youssif-abuzied/thesis_project/CyberScape_LLM_Agent/Plans/final_drone_low_level_plan.json")  # <-- create this file too
print(plans)
print(progress)

# Health check route

@app.route("/complete_phase", methods=["POST"])
def complete_phase():
    data = request.get_json()

    # Validate required fields
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

    # Get all phases for this robot
    phases = plans[robot]["phases"]
    matching_phase = next((p for p in phases if p["phase_number"] == phase), None)

    if not matching_phase:
        return jsonify({"error": f"Phase {phase} not found for robot {robot}."}), 404

    # Ensure all previous phases are completed
    for p in range(1, phase):
        if p not in progress[robot]["completed_phases"]:
            return jsonify({"error": f"Phase {p} must be completed before marking phase {phase}."}), 400

    # Validate all required outputs are present
    required_outputs = matching_phase.get("outputs", [])
    missing = [key for key in required_outputs if key not in outputs]
    if missing:
        return jsonify({"error": f"Missing required output values: {missing}"}), 400

    # Store outputs and mark phase as complete
    progress[robot]["completed_phases"].add(phase)
    progress[robot]["outputs"][phase] = outputs

    return jsonify({
        "message": f"Phase {phase} for {robot} marked as completed.",
        "stored_outputs": outputs
    }), 200


@app.route("/get_instruction", methods=["GET"])
def get_instruction():
    robot = request.args.get("robot")
    phase = request.args.get("phase")

    # Validate inputs
    if not robot or not phase:
        return jsonify({"error": "Missing 'robot' or 'phase' query parameters."}), 400

    try:
        phase = int(phase)
    except ValueError:
        return jsonify({"error": "Phase must be an integer."}), 400

    if robot not in plans:
        return jsonify({"error": f"No plan found for robot '{robot}'."}), 404

    # Get all phases for this robot
    phases = plans[robot]["phases"]
    matching_phase = next((p for p in phases if p["phase_number"] == phase), None)

    if not matching_phase:
        return jsonify({"error": f"Phase {phase} not found for robot {robot}."}), 404

    # Check if previous phases were completed
    for p in range(1, phase):
        if p not in progress[robot]["completed_phases"]:
            return jsonify({"error": f"Phase {p} must be completed before requesting phase {phase}."}), 400

    return jsonify({
        "phase_number": phase,
        "low_level_plan": matching_phase["low_level_plan"],
        "expected_outputs": matching_phase.get("outputs", [])
    }), 200


@app.route('/')
def index():
    return jsonify({"message": "Multi-Robot System Server is running"}), 200

# --- Placeholder: Add your endpoints below ---

# e.g.
# @app.route('/drone/instruction', methods=['POST'])
# def drone_instruction():
#     data = request.get_json()
#     # Process the instruction
#     return jsonify({"status": "received"}), 200

# @app.route('/robotdog/instruction', methods=['POST'])
# def robotdog_instruction():
#     data = request.get_json()
#     # Process the instruction
#     return jsonify({"status": "received"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
