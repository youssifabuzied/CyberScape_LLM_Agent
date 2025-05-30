import argparse
import asyncio
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from Utils import read_file
import warnings
import json
warnings.filterwarnings("ignore")

def generate_verification_prompt(mission, plan, rubric):
    """Generates a structured verification prompt."""
    return f"""
    Mission Details:
    {mission}

    Generated Plan:
    {plan}

    Rubric for Evaluation:
    {rubric}

    You must evaluate the above plan based on the provided rubric. Identify any weaknesses, missing steps, or inconsistencies. 
    
    **Response format:**
    - Strengths of the plan.
    - Weaknesses of the plan.
    - Suggested improvements to address weaknesses.

    Do NOT generate a new plan yet—just evaluate the given plan first.
    """

def generate_improvement_prompt(mission, plan, rubric, evaluation):
    """Generates a prompt to improve the plan based on evaluation feedback."""
    return f"""
    Mission Details:
    {mission}

    Initial Plan:
    {plan}

    Rubric for Evaluation:
    {rubric}

    Evaluation Feedback:
    {evaluation}

    Using the evaluation feedback above, generate an improved version of the plan (in the same format) that corrects the weaknesses while preserving the strengths. 
    Make sure the new plan strictly follows the rubric and addresses all identified issues.

    **Output only the revised plan.** Do NOT include explanations or commentary.
    """

def verify_plan(llm, mission, plan, rubric):
    """Runs up to two rounds of verification and improvement."""
    for _ in range(2):
        # Step 1: Evaluate the plan
        eval_prompt = generate_verification_prompt(mission, plan, rubric)
        evaluation_response = llm.invoke([
            SystemMessage(content="You are a critical evaluator for autonomous robot plans."),
            HumanMessage(content=eval_prompt)
        ])
        evaluation = evaluation_response.content.strip()
        
        # If evaluation finds no weaknesses, finalize the plan
        if "Weaknesses:" not in evaluation:
            break
        
        # Step 2: Improve the plan
        improve_prompt = generate_improvement_prompt(mission, plan, rubric, evaluation)
        improved_plan_response = llm.invoke([
            SystemMessage(content="You are a robotics planning expert."),
            HumanMessage(content=improve_prompt)
        ])
        plan = improved_plan_response.content.strip()
    
    return plan

def main():
    parser = argparse.ArgumentParser(description="Verifies and improves a generated plan based on a rubric.")
    parser.add_argument("target", help="Target robot for the plan")
    args = parser.parse_args()

    with open("config.json", "r") as f:
        config = json.load(f)

    mission_file = config["mission_text_file"]
    robot_cfg = config["robots_config"][args.target]
    output_file = robot_cfg["verified"]

    rubric   = read_file(robot_cfg["low_rubric_file"])
    plan = read_file(robot_cfg["temp_low"])
    mission = read_file(mission_file)
    
    llm = ChatOpenAI(
        api_key="c7e68755-3cfd-4f4a-a695-6a41af9ffd23",
        base_url="https://api.sambanova.ai/v1",
        model_name="Meta-Llama-3.1-405B-Instruct",
        temperature=0.1
    )
    
    verified_plan = verify_plan(llm, mission, plan, rubric)
    
    with open(output_file, "w") as file:
        file.write(verified_plan)
    
    print("Verification complete. Finalized plan saved to", output_file)

if __name__ == "__main__":
    main()
