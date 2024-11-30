import openai
import os
import argparse
from Manager import read_file
import asyncio
import subprocess

from Similarity_Mapping_Module import parse_plan, validate_plan

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

def regenerate_plan_with_errors(errors, message_history):
    # Add the errors to the message history
    error_messages = "\n".join(errors)
    message_history.append({"role": "user", "content": f"The following errors were found in your last plan:\n{error_messages}\nPlease regenerate the plan taking these errors into account."})

    # Make the API call again with the updated conversation history
    response = openai.ChatCompletion.create(
        model='Meta-Llama-3.1-405B-Instruct',
        messages=message_history,
        temperature=0.1,
        top_p=0.1
    )
    return response.choices[0].message.content


def main():
    parser = argparse.ArgumentParser(description="Read and display the contents of mission, drone, and dog specifications files.")

    # Add arguments for the file paths
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")
    parser.add_argument("drone_specifications", help="Path to the drone_specifications.txt file")
    parser.add_argument("drone_high_level_plan", help="Path to the drone's high level plan")

    # Parse the arguments
    args = parser.parse_args()
    print("Drone is planning.............")

    initial_mission = read_file(args.mission_scenario)
    drone_specifications = read_file(args.drone_specifications)
    drone_high_level_plan = read_file(args.drone_high_level_plan)
    print(type(drone_high_level_plan))

    # adapted_scenario_content = f'''{initial_mission} \n 
    # We had the above mission passed to an LLM and it generated the below high level plan for the drone: \n
    # {drone_high_level_plan} \n
    # Your goal is to translate the above drone's high level plan to a low level one using only the functions supported by this drone python class definition:
    # {drone_specifications} \n
    # You should output a python code that realizes the above plan. Your code should be full. Do not assume that someone will edit it and fill in the missing details. Also, when you need to communicate with the APM module, use the communicate_with_apm function. Your output should be a mere python code ready to run. Do not add any explanations or headers in the reponse.
    # Do not initialize any values in the code and expect someone to edit it afterwards. Your code should be ready to run directly.
    # For every phase you finish, you should communicate with the APM about it. After you finish, you should stay idle waiting for any messages from the apm.

    # '''
    adapted_scenario_content = f'''{initial_mission} \n 
    We had the above mission passed to an LLM and it generated the below high level plan for the drone: \n
    {drone_high_level_plan} \n
    Your goal is to translate the above drone's high level plan to a low level one using only the following instructions
    {drone_specifications} \n
    Your output format should be as follows:
    Phase 1:
    Drone.move_to_point((X,Y,Z))
    .........
    Phase 2:
    .........
    Phase 3:
    .........
    You should stick to this format and do not write any side notes or explanations. You should only write a set of instructions. If the objective of some phase is reached, you should move immediately to the following one.
    Feel free to use only if statmentents. You are not allowed to use any kind of loops. Do not use functions other than the listed functions in the specifications file. If the high level plan says that the drone should not do anything, the drone needs to stay idle waiting for messages from the APM and do nothing.
    You should only care about the drone part not any other robot. Again if the plan is for the robot dog only, you should stand idle and only wait for messages.
    '''
    client = openai.OpenAI(
        api_key=os.getenv("samba_nova_api_key"),
        base_url="https://api.sambanova.ai/v1",
    )
    message_history = [
        {"role": "system", "content": "You are a helpful assistant"},
        {"role": "user", "content":adapted_scenario_content},
    ]
    response = client.chat.completions.create(
        model='Meta-Llama-3.1-405B-Instruct',
        messages=message_history,
        temperature=0.1,
        top_p=0.1
    )
    low_level_plan = response.choices[0].message.content
    with open("drone_low_level_plan.txt", "w") as file:
        file.write(low_level_plan)

    command = "python3 plan_parser.py drone_low_level_plan.txt"
    asyncio.run(run_python_script(command))

    # Read the plan from the file and validate it
    with open("drone_low_level_plan.txt", "r") as file:
        drone_low_level_plan = file.read()

    parsed_plan = parse_plan(drone_low_level_plan)
    validation_results, new_plan = validate_plan(parsed_plan)

    # Check if there are errors in the plan
    while validation_results:
        print("Errors found in the plan. Regenerating the plan...")

        # Append errors to message history and regenerate the plan
        updated_plan = regenerate_plan_with_errors(validation_results, message_history)

        # Save the updated plan and validate again
        with open("drone_low_level_plan.txt", "w") as file:
            file.write(updated_plan)

        # Run the parser again with the new plan
        command = "python3 plan_parser.py drone_low_level_plan.txt"
        asyncio.run(run_python_script(command))

        # Re-validate the plan
        with open("drone_low_level_plan.txt", "r") as file:
            drone_low_level_plan = file.read()

        parsed_plan = parse_plan(drone_low_level_plan)
        validation_results, new_plan = validate_plan(parsed_plan)

    print("No errors found. New plan is:")
    for command in new_plan:
        print(command)


if __name__ == "__main__":
    main()