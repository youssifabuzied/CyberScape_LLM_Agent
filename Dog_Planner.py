import openai
import os
import argparse
from Manager import read_file



def main():
    parser = argparse.ArgumentParser(description="Read and display the contents of mission, dog, and dog specifications files.")

    # Add arguments for the file paths
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")
    parser.add_argument("dog_specifications", help="Path to the dog_specifications.txt file")
    parser.add_argument("dog_high_level_plan", help="Path to the dog's high level plan")

    # Parse the arguments
    args = parser.parse_args()
    print("Dخل is planning.............")

    initial_mission = read_file(args.mission_scenario)
    dog_specifications = read_file(args.dog_specifications)
    dog_high_level_plan = read_file(args.dog_high_level_plan)
    print(type(dog_high_level_plan))

    adapted_scenario_content = f'''{initial_mission} \n 
    We had the above mission passed to an LLM and it generated the below high level plan for the dog: \n
    {dog_high_level_plan} \n
    Your goal is to translate the above dog's high level plan to a low level one using only the following instructions
    {dog_specifications} \n
    Your output format should be as follows:
    Phase 1:
    GO_TO(X,Y)
    .........
    Phase 2:
    .........
    Phase 3:
    .........
    You should stick to this format and do not write any side notes or explanations. Be very careful in translating the high level plan and follow every step in it.  
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
        temperature=0.05,
        top_p=0.05
    )
    with open("dog_low_level_plan.txt", "w") as file:
        file.write(response.choices[0].message.content)
if __name__ == "__main__":
    main()
