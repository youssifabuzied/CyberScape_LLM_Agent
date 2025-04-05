import openai
import os
import argparse
import json
from Utils import read_file
from dotenv import load_dotenv

load_dotenv()

# OpenAI API Key
CONFIG_FILE = "config.json"
with open(CONFIG_FILE, "r") as f:
    config = json.load(f)

OPENAI_API_KEY = config.get("openai_api_key", "")

def Pparser(plan_file: str, target:str) -> str:
    """
    Processes a plan file:
    - Deletes lines starting with 'Phase'.
    - For lines starting with 'if' or 'if not', removes 'if' or 'if not' and retains the rest of the line.
    - Removes colons (:) if they exist at the end of a filtered line.
    - Ensures the first character of the output plan is always uppercase.

    :param plan_file: Path to the file containing the plan.
    :return: A string containing the filtered plan with proper formatting.
    """
    prompt = f'''
    We have the following instructions to be sent to a {target} to execute
    {plan_file}
    Your task is to extract the instructions that the robot will execute. Instructions are in this format:
    Drone.instruction() // for the drone
    RobotDog.instruction() // for the robot dog
    You will find the instructions mixed with if statments or assign statments. You need to only to extract the instructions and the paramters passed to them. 
    Each instruction belongs to a specific phase in the original plan. You **must** include "Phase X:" before listing the instructions for that phase.
    Output instructions such that each instruction is on a line. You need to extract the instructions yourself. Do nor output a code that does so. Do it yourself and output the parsed instructions to me.
    Output the instructions directly without saying here are the instructions.
    '''
    client = openai.OpenAI(
        api_key=OPENAI_API_KEY,
        # base_url="https://openrouter.ai/api/v1",
        base_url="https://api.openai.com/v1", # for openai?
        # base_url="https://api.sambanova.ai/v1", # for llama

    )
    message_history = [
        {"role": "system", "content": "You are a helpful assistant"},
        {"role": "user", "content":prompt},
    ]
    response = client.chat.completions.create(
        # model='deepseek/deepseek-chat:free',
        model='gpt-4o-mini',
        messages=message_history,
        temperature=0.1,
        top_p=0.1
    )
    return response.choices[0].message.content

def main():
    parser = argparse.ArgumentParser(description="Parsing instructions")

    parser.add_argument("target", help="Target robot for the plan")
    args = parser.parse_args()

    with open("config.json", "r") as f:
        config = json.load(f)

    if args.target == "ROBOT_DOG":
        plan_file_path = config["dog_verified_plan_file"]
        parsed_file = config["dog_parsed_plan_file"]
    else:
        plan_file_path = config["drone_verified_plan_file"]
        parsed_file = config["drone_parsed_plan_file"]

    print("Instructions are being parsed.............")

    plan_file = read_file(plan_file_path)
    # print(plan_file)
    filtered_plan = Pparser(plan_file, args.target)
    # with open(f"parsed_{plan_file_path}", "w") as file:
    with open(f"{parsed_file}", "w") as file:
        file.write(filtered_plan)
if __name__ == "__main__":
    main()
