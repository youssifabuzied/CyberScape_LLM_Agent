import openai
import os
import argparse
from Manager import read_file


def Pparser(plan_file: str) -> str:
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
    We had the following instructions sent to a robot to execute
    {plan_file}
    Your task is to extract the instructions that the robot will execute. Instructions are in this format:
    Drone.instruction()
    RobotDog.instruction() // depends on the type of the robot
    You will find the instructions mixed with if statments or assign statments. You need to only to extract the instructions and the paramters passed to them. 
    Output instructions such that each instruction is on a line. You need to extract the instructions yourself. Do nor output a code that does so. Do it yourself and output the parsed instructions to me.
    Output the instructions directly without saying here are the instructions.
    '''
    client = openai.OpenAI(
        api_key=os.getenv("samba_nova_api_key"),
        base_url="https://api.sambanova.ai/v1",
    )
    message_history = [
        {"role": "system", "content": "You are a helpful assistant"},
        {"role": "user", "content":prompt},
    ]
    response = client.chat.completions.create(
        model='Meta-Llama-3.1-405B-Instruct',
        messages=message_history,
        temperature=0.1,
        top_p=0.1
    )
    return response.choices[0].message.content

def main():
    parser = argparse.ArgumentParser(description="Parsing instructions")

    # Add arguments for the file paths
    parser.add_argument("plan_file_path", help="Path to the plan file")

    # Parse the arguments
    args = parser.parse_args()
    print("Instructions are being parsed.............")

    plan_file_path = args.plan_file_path
    plan_file = read_file(plan_file_path)
    # print(plan_file)
    filtered_plan = Pparser(plan_file)
    with open(f"parsed_{plan_file_path}", "w") as file:
        file.write(filtered_plan)
if __name__ == "__main__":
    main()
