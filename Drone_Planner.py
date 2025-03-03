from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from langchain.prompts import PromptTemplate
import asyncio
import argparse
import subprocess
from Manager import read_file
import warnings
warnings.filterwarnings("ignore")

# Hardcoded rubric files
MIDDLE_LEVEL_RUBRIC_FILE = "Rubrics/Drone_Middle_Rubric.txt"
LOW_LEVEL_RUBRIC_FILE = "Rubrics/Drone_Low_Rubric.txt"

async def run_python_script(command):
    """Executes a Python script asynchronously and prints output."""
    try:
        process = await asyncio.create_subprocess_shell(
            command, 
            stdout=asyncio.subprocess.PIPE, 
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await process.communicate()

        if stdout:
            print("Output of the script:\n", stdout.decode())  
        if stderr:
            print("Error output:\n", stderr.decode())  

    except Exception as e:
        print(f"An error occurred: {e}")

def generate_middle_level_plan(llm, mission_text, drone_specifications, high_level_plan):
    """Uses LangChain to generate a structured middle-level plan."""
    prompt_template = PromptTemplate(
        input_variables=["mission", "high_level_plan", "drone_specifications"],
        template="""
        Mission Details:
        {mission}

        High-Level Drone Plan:
        {high_level_plan}

        You must translate the above high-level plan into a human-readable, structured middle-level set of steps.
        The middle-level plan should break down the high-level objectives into detailed but still readable instructions
        that describe the sequence of actions the drone will take.

        **Rules:**
        - Maintain clarity and structure.
        - Break down actions logically, describing each phase step by step.
        - Avoid implementation details; this is a conceptual breakdown.

        **Example Output:**
        ```
        Phase 1: The drone moves to the specified starting location and begins scanning the area.
        Phase 2: If the scan detects an anomaly, the drone takes a close-up image and sends it to the base.
        ```
        """
    )

    response = llm.invoke([
        SystemMessage(content="You are a helpful assistant."),
        HumanMessage(content=prompt_template.format(
            mission=mission_text, 
            high_level_plan=high_level_plan, 
            drone_specifications=drone_specifications
        ))
    ])
    return response.content.strip()

def generate_low_level_plan(llm, middle_level_plan, drone_specifications):
    """Uses LangChain to convert a structured middle-level plan into low-level executable commands."""
    prompt_template = PromptTemplate(
        input_variables=["middle_level_plan", "drone_specifications"],
        template="""
        Middle-Level Plan:
        {middle_level_plan}

        Convert the above plan into a low-level set of executable drone instructions using ONLY the following functions:
        {drone_specifications}

        **Rules:**
        - ONLY use the functions provided. Do not invent or assume additional functions.
        - NO loops (for, while) and NO recursive calls.
        - Each phase must be independent, executing only after the previous phase completes.
        - If the drone needs to wait, explicitly include an idle state.

        **Output format example:**
        ```
        Phase 1:
        Drone.move_to_point((100,200,50))
        Drone.scan_area()

        Phase 2:
        if Drone.scan_successful():
            Drone.send_data_to_apm()
        ```
        """
    )

    response = llm.invoke([
        SystemMessage(content="You are a helpful assistant."),
        HumanMessage(content=prompt_template.format(
            middle_level_plan=middle_level_plan, 
            drone_specifications=drone_specifications
        ))
    ])
    return response.content.strip()

def main():
    parser = argparse.ArgumentParser(description="Generate structured execution plans for a drone.")
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")
    parser.add_argument("drone_specifications", help="Path to the drone_specifications.txt file")
    parser.add_argument("drone_high_level_plan", help="Path to the drone's high-level plan")
    parser.add_argument("verified_middle_level_output", help="Path to store the verified middle-level plan")
    parser.add_argument("verified_low_level_output", help="Path to store the verified low-level plan")
    args = parser.parse_args()

    print("Generating structured drone execution plans...")

    mission_text = read_file(args.mission_scenario)
    drone_specifications = read_file(args.drone_specifications)
    drone_high_level_plan = read_file(args.drone_high_level_plan)

    llm = ChatOpenAI(
        api_key="c7e68755-3cfd-4f4a-a695-6a41af9ffd23",
        base_url="https://api.sambanova.ai/v1",
        model_name="Meta-Llama-3.1-405B-Instruct",
        temperature=0.1
    )

    # Generate middle-level plan
    middle_level_plan = generate_middle_level_plan(llm, mission_text, drone_specifications, drone_high_level_plan)
    print("Middle-Level Plan:")
    print(middle_level_plan)
    
    with open("Plans/Middle_Level_Plans/drone_middle_level_plan.txt", "w") as file:
        file.write(middle_level_plan)

    # Verify middle-level plan
    verification_command = f"python3 Verification_Module.py {MIDDLE_LEVEL_RUBRIC_FILE} Plans/Middle_Level_Plans/drone_middle_level_plan.txt {args.mission_scenario} {args.verified_middle_level_output}"
    print("Verifying the middle-level plan using plan_verifier.py...")
    asyncio.run(run_python_script(verification_command))

    print(f"Verified middle-level plan written to {args.verified_middle_level_output}")

    # Read the verified middle-level plan
    verified_middle_level_plan = read_file(args.verified_middle_level_output)

    # Generate low-level plan
    low_level_plan = generate_low_level_plan(llm, verified_middle_level_plan, drone_specifications)
    print("Low-Level Plan:")
    print(low_level_plan)
    
    with open("Plans/Low_Level_Plans/drone_low_level_plan.txt", "w") as file:
        file.write(low_level_plan)

    # Verify low-level plan
    low_level_verification_command = f"python3 Verification_Module.py {LOW_LEVEL_RUBRIC_FILE} Plans/Low_Level_Plans/drone_low_level_plan.txt {args.verified_middle_level_output} {args.verified_low_level_output}"
    print("Verifying the low-level plan using plan_verifier.py...")
    asyncio.run(run_python_script(low_level_verification_command))

    print(f"Verified low-level plan written to {args.verified_low_level_output}")

    # Read the verified low-level plan before execution
    final_low_level_plan = read_file(args.verified_low_level_output)

    # Execute the verified low-level plan
    command = "python3 plan_parser.py Plans/Low_Level_Plans/drone_low_level_plan.txt"
    asyncio.run(run_python_script(command))

if __name__ == "__main__":
    main()
