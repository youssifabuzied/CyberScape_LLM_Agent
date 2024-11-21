import openai
import os
import argparse
from Manager import read_file



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

    adapted_scenario_content = f'''{initial_mission} \n 
    We had the above mission passed to an LLM and it generated the below high level plan for the drone: \n
    {drone_high_level_plan} \n
    Your goal is to translate the above drone's high level plan to a low level one using only the following instructions
    {drone_specifications} \n
    Your output format should be as follows:
    Phase 1:
    Fly_Over(X,Y)
    .........
    Phase 2:
    .........
    Phase 3:
    .........
    You should stick to this format and do not write any side notes or explanations.
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
        temperature=0.1,
        top_p=0.1
    )
    with open("drone_low_level_plan.txt", "w") as file:
        file.write(response.choices[0].message.content)
if __name__ == "__main__":
    main()
