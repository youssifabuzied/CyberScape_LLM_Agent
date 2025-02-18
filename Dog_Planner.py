from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from langchain.prompts import PromptTemplate
import asyncio
import argparse
import subprocess
from Manager import read_file

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

def generate_low_level_plan(llm, mission_text, dog_specifications, high_level_plan):
    """Uses LangChain to generate a structured low-level plan."""
    
    prompt_template = PromptTemplate(
        input_variables=["mission", "high_level_plan", "dog_specifications"],
        template="""
        Mission Details:
        {mission}

        High-Level Robot Dog Plan:
        {high_level_plan}

        **Objective:**  
        Convert the above high-level plan into a low-level sequence of instructions using **ONLY** the following functions:  
        {dog_specifications}

        **Strict Rules:**  
        - **Use ONLY the listed functions.** Do not create or assume additional functionality.  
        - **Replace placeholders with actual values.** Do not output (X, Y). Instead, use specific values.  
        - **No loops (for, while) and no recursion.**  
        - **Every movement must be verified.** Use `if RobotDog.has_reached(X, Y):` before proceeding.  
        - **If a command fails, retry once.** If it still fails, notify the APM and wait.  
        - **If multiple actions can happen in parallel, execute them concurrently.**  
        - **If synchronization with another agent is required, include an explicit wait state.**  

        **Expected Output Format:**  
        ```
        Phase 1:
        RobotDog.move_to(100,200)
        if RobotDog.has_reached(100,200):
            RobotDog.scan_area()

        Phase 2:
        if RobotDog.scan_successful():
            RobotDog.send_data_to_apm()
        ```

        **DO NOT** include explanations or headers in your response.
        """
    )

    response = llm.invoke([
        SystemMessage(content="You are a helpful assistant."),
        HumanMessage(content=prompt_template.format(
            mission=mission_text, 
            high_level_plan=high_level_plan, 
            dog_specifications=dog_specifications
        ))
    ])
    
    return response.content.strip()

def main():
    parser = argparse.ArgumentParser(description="Generate a low-level execution plan for a robot dog.")
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")
    parser.add_argument("dog_specifications", help="Path to the dog_specifications.txt file")
    parser.add_argument("dog_high_level_plan", help="Path to the dog's high level plan")
    args = parser.parse_args()

    print("Generating low-level dog execution plan...")

    mission_text = read_file(args.mission_scenario)
    dog_specifications = read_file(args.dog_specifications)
    dog_high_level_plan = read_file(args.dog_high_level_plan)

    llm = ChatOpenAI(
        api_key="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",
        base_url="https://openrouter.ai/api/v1",
        model_name="deepseek/deepseek-chat:free",
        temperature=0.05
    )

    low_level_plan = generate_low_level_plan(llm, mission_text, dog_specifications, dog_high_level_plan)

    with open("dog_low_level_plan.txt", "w") as file:
        file.write(low_level_plan)

    print("Low-Level Plan written to dog_low_level_plan.txt")

    command = "python3 plan_parser.py dog_low_level_plan.txt"
    asyncio.run(run_python_script(command))

if __name__ == "__main__":
    main()
