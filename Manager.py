import openai
import os
import asyncio
import subprocess
import argparse
import warnings
warnings.filterwarnings("ignore")


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

def read_file(file_path):
    """Reads the content of a file and returns it."""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except FileNotFoundError:
        print(f"Error: The file {file_path} does not exist.")
        return None
    except Exception as e:
        print(f"An error occurred while reading the file {file_path}: {e}")
        return None
def initiate_plan():
    # Accept the file paths for the three files
    # mission_scenario_path = input("Enter the path to mission_scenario.txt: ")
    # drone_specifications_path = input("Enter the path to drone_specifications.txt: ")
    # dog_specifications_path = input("Enter the path to dog_specifications.txt: ")
    mission_scenario_path = "mission_files/octagon_mission.txt"
    drone_specifications_path = "mission_files/drone_specifications.txt"
    dog_specifications_path = "mission_files/dog_specifications.txt"


    # Read the contents of the files
    mission_scenario_content = read_file(mission_scenario_path)
    drone_specifications_content = read_file(drone_specifications_path)
    dog_specifications_content = read_file(dog_specifications_path)

    # Print the contents of the files
    # if mission_scenario_content:
    #     print("\n--- Mission Plan ---")
    #     print(mission_scenario_content)

    # if drone_specifications_content:
    #     print("\n--- Drone Specifications ---")
    #     print(drone_specifications_content)

    # if dog_specifications_content:
    #     print("\n--- Dog Specifications ---")
    #     print(dog_specifications_content)

    adapted_scenario_content = mission_scenario_content+ "/n"+  '''Your plan should be of this format:
        Drone Plan:
        Phase 1: ----------------------------
        Phase 2: ---------------------------
        -------
        Dog Plan:
        Phase 1: -----------------------------
        Phase 2: ------------------------------
        Do not write any header in the response or side notes or explanations. If the mission require scanning an area greating than the scanning capabailities of the robots, define a sprial search algorithm to make them scan the large area.
        If the robots need to send messages to other robots. this should only be done through the adaptive planning module. Also, they should do the same when wanting to receive messages.'''
    print(os.getenv("samba_nova_api_key"))
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
        temperature=0.1,
        top_p=0.1
    )
    with open("Plans/initial_plan.txt", "w") as file:
        file.write(response.choices[0].message.content)

    print("Mission Plan is written to Plans/initial_plan.txt")
def parse_drone_plan(file_path, output_file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Flags to determine when we are in the drone plan section
        drone_plan_started = False
        drone_plan = []

        # Loop through the lines and extract the drone plan
        for line in lines:
            if "Drone Plan:" in line:
                drone_plan_started = True  # Start capturing the drone plan
                continue  # Skip the "Drone Plan:" line

            if drone_plan_started:
                # Stop capturing if we encounter "Dog Plan:"
                if "Dog Plan:" in line:
                    break

                # Append the line to the drone plan
                drone_plan.append(line.strip())

        # Write the extracted drone plan to the new file
        with open(output_file_path, 'w') as output_file:
            output_file.write("\n".join(drone_plan))

        print(f"Drone plan has been successfully written to {output_file_path}")

    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
def parse_dog_plan(file_path, output_file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Flags to determine when we are in the drone plan section
        dog_plan_started = False
        dog_plan = []

        # Loop through the lines and extract the drone plan
        for line in lines:
            if "Dog Plan:" in line:
                dog_plan_started = True  # Start capturing the drone plan
                continue  # Skip the "Drone Plan:" line

            if dog_plan_started:
                # Append the line to the drone plan
                dog_plan.append(line.strip())

        # Write the extracted drone plan to the new file
        with open(output_file_path, 'w') as output_file:
            output_file.write("\n".join(dog_plan))

        print(f"Dog plan has been successfully written to {output_file_path}")

    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plan Generator for robot missions")

    # Add arguments for the file paths
    parser.add_argument("mission_file", help="Path to the plan file")

    # Parse the arguments
    args = parser.parse_args()
    # initiate_plan()
    mission_file = args.mission_file
    high_level_plan_command = f"python High_Level_Plan_Generator.py {mission_file}"
    asyncio.run(run_python_script(high_level_plan_command))
    input_plan_file = 'Plans/initial_plan.txt'  # The input file containing both plans
    output_drone_file = 'Plans/Initial_Robot_Plans/drone_initial_plan.txt'
    output_dog_file = "Plans/Initial_Robot_Plans/dog_initial_plan.txt"  # The output file for the drone plan

    # Call the function to parse the drone plan
    parse_drone_plan(input_plan_file, output_drone_file)
    parse_dog_plan(input_plan_file, output_dog_file)
    # Call the function to parse the drone plan

    drone_command = (
        f"python3 Drone_Planner.py {mission_file} mission_files/drone_specifications.txt {output_drone_file} Plans/Middle_Level_Plans/drone_middle_level_plan.txt Plans/Low_Level_Plans/drone_low_level_plan.txt"
    )
 #   asyncio.run(run_python_script(drone_command))
    dog_command = (
        f"python3 Dog_Planner.py {mission_file} mission_files/dog_specifications.txt {output_dog_file} Plans/Middle_Level_Plans/dog_middle_level_plan.txt Plans/Low_Level_Plans/dog_low_level_plan.txt"
    )
  #  asyncio.run(run_python_script(dog_command))
