import openai
import os
import argparse
from Manager import read_file
import asyncio
import subprocess
async def run_python_script(command):
    try:
        # Create the subprocess and await its completion
        count = 0
        # for i in range (0,1000000):
        #     count +=1
        process = await asyncio.create_subprocess_shell(
            command, 
            stdout=asyncio.subprocess.PIPE, 
            stderr=asyncio.subprocess.PIPE
        )

        # Wait for the process to complete and capture the output and error
        stdout, stderr = await process.communicate()

        # Decode and print the result
        print("Output of the script:\n")
        if stdout:
            print(stdout.decode())  # Print the standard output of the script
        if stderr:
            print("Error output:\n")
            print(stderr.decode())  # Print any error messages

    except Exception as e:
        print(f"An error occurred: {e}")

def main():
    parser = argparse.ArgumentParser(description="Read and display the contents of mission, dog, and dog specifications files.")

    # Add arguments for the file paths
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")
    parser.add_argument("dog_specifications", help="Path to the dog_specifications.txt file")
    parser.add_argument("dog_high_level_plan", help="Path to the dog's high level plan")

    # Parse the arguments
    args = parser.parse_args()
    print("Dog is planning.............")

    initial_mission = read_file(args.mission_scenario)
    dog_specifications = read_file(args.dog_specifications)
    dog_high_level_plan = read_file(args.dog_high_level_plan)
    # print(type(dog_high_level_plan))

    # adapted_scenario_content = f'''{initial_mission} \n 
    # We had the above mission passed to an LLM and it generated the below high level plan for the dog: \n
    # {dog_high_level_plan} \n
    # Your goal is to translate the above dog's high level plan to a low level one using only the functions supported by this dog python class definition:
    # {dog_specifications} \n
    # You should output a python code that realizes the above plan. Your code should be full.
    # Do not assume that someone will edit it and fill in the missing details. 
    # Also, when you need to communicate with the APM module, use the communicate_with_apm function.
    # Your output should be a mere python code ready to run. Do not add any explanations or headers in the reponse.
    # Do not initialize any values in the code and expect someone to edit it afterwards. Your code should be ready to run directly.
    # For every phase you finish, you should communicate with the APM about it. After you finish, you should stay idle waiting for any messages from the apm.
    # '''
    adapted_scenario_content = f'''{initial_mission} \n 
    We had the above mission passed to an LLM and it generated the below high level plan for the dog: \n
    {dog_high_level_plan} \n
    Your goal is to translate the above dog's high level plan to a low level one using only the functions supported by this dog python class definition:
    {dog_specifications} \n
    Your output format should be as follows:
    Phase 1:
    RobotDog.move_to(X,Y)
    .........
    Phase 2:
    .........
    Phase 3:
    .........
    You should stick to this format and do not write any side notes or explanations. You should only write a set of instructions. If the objective of some phase is reached, you should move immediately to the following one.
    Feel free to use only if statmentents. You cannot use loops. Do not use instructions other than the listed functions in the specifications file.
    '''
    client = openai.OpenAI(
        api_key="b37a4309-f1a2-4fd9-b015-eacac68fd6e5",
        base_url="https://api.sambanova.ai/v1",
    )
    message_history = [
        {"role": "system", "content": "You are a helpful assistant"},
        {"role": "user", "content":adapted_scenario_content},
    ]
    response = client.chat.completions.create(
        model='Meta-Llama-3.1-405B-Instruct',
        messages=message_history,
        temperature=0.05,
        top_p=0.05
    )
    with open("dog_low_level_plan.txt", "w") as file:
        file.write(response.choices[0].message.content)
    command = ("python3 plan_parser.py dog_low_level_plan.txt")
    asyncio.run(run_python_script(command))
if __name__ == "__main__":
    main()
