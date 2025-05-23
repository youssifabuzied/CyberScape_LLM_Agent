import os
import argparse
import re
import json
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from langchain.prompts import PromptTemplate
from Utils import read_file
from langchain_google_genai import GoogleGenerativeAI

import sys
import codecs
sys.stdout = codecs.getwriter("utf-8")(sys.stdout.detach())


# Load Gemini Pro Model

class PlanPhase:
    """Represents a single phase in a mission plan."""
    def __init__(self, target, phase_number, state, phase_target, inputs=None, outputs=None):
        self.target = target  # Identifier for the entity (e.g., "DRONE", "ROBOT_DOG")
        self.phase_number = phase_number
        self.state = state  # Represents the mission state before executing this phase
        self.phase_target = phase_target  # Independent mission objective for this phase
        self.inputs = inputs if inputs else []  # Required inputs for this phase
        self.outputs = outputs if outputs else []  # Outputs passed to the next phase

    def to_dict(self):
        """Converts the phase details to a dictionary."""
        return {
            "target": self.target,
            "phase_number": self.phase_number,
            "state": self.state,
            "phase_target": self.phase_target,
            "inputs": self.inputs,
            "outputs": self.outputs
        }


class Plan:
    """Represents a structured plan for a specific entity (e.g., drone, robot dog)."""
    def __init__(self, target):
        self.target = target  # Identifier for the entity (e.g., "DRONE", "ROBOT_DOG")
        self.phases = []

    def add_phase(self, phase):
        """Adds a phase to the plan."""
        self.phases.append(phase)

    def to_dict(self):
        """Converts the plan into a dictionary format."""
        return {
            "target": self.target,
            "phases": [phase.to_dict() for phase in self.phases]
        }


import re

def parse_generated_plan(llm, generated_text, target):
    """Converts LLM-generated structured text into structured Plan and PlanPhase objects."""

    parsing_prompt = f"""
    The following text describes a mission plan for a {target} in an unstructured format:

    {generated_text}

    **Instructions:**
    - Reformat the plan in a structured way using the following template:
    
    ```
    Target: {target}

    Phase 1:
    - State: <state before execution>
    - Target: <goal of this phase>
    - Inputs: [input1, input2, ...]
    - Outputs: [output1, output2, ...]

    Phase 2:
    - State: <state before execution>
    - Target: <goal of this phase>
    - Inputs: [input1, input2, ...]
    - Outputs: [output1, output2, ...]
    ```

    **Rules:**
    - **Strictly follow the format above.** No explanations.
    - **Only return the plan text** (do NOT use markdown formatting like ```).
    - Ensure **inputs and outputs are written as comma-separated lists inside square brackets.**
    """

    response = llm.invoke([SystemMessage(content="You are a helpful assistant"), HumanMessage(content=parsing_prompt)])

    if not response:
        raise ValueError("Error: LLM returned an empty response.")

    structured_text = response.content.strip()

    # Extract the target
    target_match = re.search(r'Target:\s*(\w+)', structured_text)
    extracted_target = target_match.group(1).strip() if target_match else target

    # Extract phases
    phase_pattern = re.findall(
        r'Phase (\d+):\s*'
        r'- State:\s*(.*?)\s*'
        r'- Target:\s*(.*?)\s*'
        r'- Inputs:\s*\[(.*?)\]\s*'
        r'- Outputs:\s*\[(.*?)\]',
        structured_text, re.DOTALL
    )

    plan = Plan(extracted_target)
    def parse_variables(variable_text):
        """Extracts variable names and types from text like 'X <float>, Y <int>' into a dictionary."""
        variable_dict = {}
        variable_matches = re.findall(r'(\w+)\s*<(\w+)>', variable_text)
        for var_name, var_type in variable_matches:
            variable_dict[var_name.strip()] = {"type": var_type.strip()}
        return variable_dict

    for phase_num, state, phase_target, inputs, outputs in phase_pattern:
        inputs_dict = parse_variables(inputs)
        outputs_dict = parse_variables(outputs)
        plan.add_phase(PlanPhase(
            extracted_target, 
            int(phase_num.strip()), 
            state.strip(), 
            phase_target.strip(), 
            inputs_dict, 
            outputs_dict
        ))

    return plan

def generate_plan(llm, mission_text, target):
    """Generates an initial plan using the LLM with enhanced clarity and constraints."""
    prompt = f"""
    **Mission Overview:**
    {mission_text}

    **Your Task:**  
    Generate a structured, step-by-step mission plan specifically for the **{target}**.

    **Plan Structure:**  
    - Each phase must describe an independent **mission objective**.
    - The **drone** is responsible for aerial scanning and providing location data.
    - The **robot dog** handles ground operations and object interaction.
    - The **output of each phase** must serve as the **input for the next relevant phase**.
    - Clearly specify **inputs and outputs** for each phase.

    **Key Rules:**  
    1. **Thorough State Descriptions**  
       - Clearly describe the **robot's current state** before each phase.  
       - Provide relevant **sensor data, positional information, or active tasks** in the state.  
       - Avoid vague descriptions like "waiting" or "processing"; be specific.  
       - Summarize the main mission we have in the state and also summarize the progress we had so far in the mission.
       - Make state descrption around 70 words for each phase. 

    2. **Phase Independence**  
       - Each phase must be fully independent and self-contained.  
       - If a phase requires an input (e.g., a coordinate from a previous phase), **explicitly state it** as:  
         - `"Given a coordinate (X, Y), perform the following action..."`  
       - This ensures clarity in execution.  

    3. **Strict Data Type Constraints**  
       - **All variables can be either strings or floats**  
       - There are **no complex types or arrays** in the inputs or outputs.  
       - Example: If the output of one phase is a location, it should be defined as:  
         - `Outputs: [X <float>, Y  <float>]`  
       - Similarly, if an object is detected, the output should be string:  
         - `Outputs: [detection_status <string>]`  

    **Example Format:**  

    **Drone Plan:**  
    ```
    Phase 1:  
    - State: We had a mission to locate an object and it requires a drone. The drone is hovering at altitude 10.0 meters, scanning for objects with its onboard camera.  
    - Target: Identify the object's coordinates.  
    - Inputs: []  
    - Outputs: [X <float> , Y <float>]  
    ```

    **Robot Dog Plan:**  
    ```
    Phase 1:  
    - State: We have a mission to locate and grap an object. We had a drone that locates the object and the dog should grap it. The robot dog is standing at the base station, ready to navigate.  
    - Target: Given a coordinate (X, Y), move to the object's location and retrieve it.  
    - Inputs: [X <float>, Y <float>]  
    - Outputs: [retrieval_status <string>]  
    ```

    **Final Constraints:**  
    - **Strictly follow the given format.**  
    - **No explanations—return only structured text.**  
    - **Ensure logical input-output flow between phases.**
    - **If the plan is only targetting a single robot. Do it provide any plan for the other robot just return nothing under the name of the other robot.  
    - **The drone and robot dog should communicate through outputs and inputs only.**  
    - **The input and output variables are variables related to the environment and the progress of the mission not processing.** 
    """
    

    response = llm.invoke([SystemMessage(content="You are a helpful assistant"), HumanMessage(content=prompt)])
    print(type(response))
    return parse_generated_plan(llm,response, target)


def refine_plan(llm, mission_text, plan):
    """Refines the plan iteratively until it meets rubric criteria with a rating of 10."""
    rating = 0
    count = 0
    final_plan = plan

    prompt_template = PromptTemplate(
        input_variables=["mission", "plan"],
        template="""
        We had the following mission:
        {mission}

        And we used an LLM to generate the following high-level plan:
        {plan}

        Evaluate this plan based on the following rubric:

        - **Logical Soundness (1-10):** Is the plan coherent and free of contradictions?
        - **Feasibility (1-10):** Can the robots execute the tasks given their capabilities?
        - **Completeness (1-10):** Does the plan cover all mission objectives?
        - **Data Flow Integrity (1-10):** Do the outputs from one phase correctly match the inputs of the next?

        Provide a structured response in the same format, ensuring proper input-output matching.

        - If the plan is already perfect, assign it a rating of **10** and return the same plan.
        - Do **not** provide explanations—just return the structured output.
        """
    )

    while count < 3:
        response = llm.invoke([HumanMessage(content=prompt_template.format(mission=mission_text, plan=str(final_plan)))])
        new_rating, modified_plan_text = extract_rating_and_plan(response.content)

        if new_rating is None or new_rating <= rating or new_rating == 10:
            break

        rating = new_rating
        final_plan = parse_generated_plan(modified_plan_text, final_plan.target)
        count += 1

    return final_plan


def main():
    with open("config.json", "r") as f:
        config = json.load(f)
    
    # Load the mission text file from the config, default to "mission_files/mission_scenario.txt" if not provided
    mission_text_file = config.get("mission_text_file", "mission_files/mission_scenario.txt")

    print("Generating structured mission plan...")

    mission_text = read_file(mission_text_file)
    if not mission_text:
        print("Error: Mission scenario file is empty or missing.")
        return

    OPENAI_API_KEY = config.get("openai_api_key", "")
    
    llm = ChatOpenAI(
        model_name="gpt-4o",
        openai_api_key=OPENAI_API_KEY,
        temperature=0.1
    )

    mission_output = {}
    for robot in config["robots_in_curr_mission"]:
        plan = generate_plan(llm, mission_text, robot)
        mission_output[ config["robots_config"][robot]["high_plan_key"] ] = plan.to_dict()

    mission_plan_file = config.get("mission_plan_file", "Plans/mission_plan.json")
    with open(mission_plan_file, 'w') as output_file:
        json.dump(mission_output, output_file, indent=4)

    print("Mission Plan written to", mission_plan_file)

if __name__ == "__main__":
    main()
