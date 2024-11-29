import openai
import os
import argparse
from Manager import read_file
import re

def extract_rating_and_plan(text):
    # Regular expression to find the rating (a number following "Rating: ")
    rating_match = re.search(r'Rating: (\d+)', text)
    
    # Regular expression to capture everything following "Modified_Plan:"
    plan_match = re.search(r'Modified_Plan:\s*(.*)', text, re.DOTALL)
    
    # Extract the rating and plan
    if rating_match and plan_match:
        rating = int(rating_match.group(1))
        modified_plan = plan_match.group(1).strip()
        return rating, modified_plan
    else:
        return None, None

def main():
    parser = argparse.ArgumentParser(description="Generate and refine a high level plan for the mission")

    # Add arguments for the file paths
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")

    # Parse the arguments
    args = parser.parse_args()
    print("High level plan is being generated.............")

    initial_mission = read_file(args.mission_scenario)
    
    adapted_scenario_content = initial_mission+ "/n"+  '''Your plan should be of this format:
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
    generated_plan = response.choices[0].message.content
    print(generated_plan)
    rating = 0
    count = 0
    while(True):
        prompt = f'''
        We had the following mission:
        {initial_mission}
        And we used an LLM to generate the following high level plan:
        {generated_plan}
        Give a rating for this plan from 1 to 10 and fine tune it to make its rating 10 if it is lower than 10.
        Just give the rating and the refined plan without any explanation. In your rating you should only focus on whether the plan is logical and identify any issues that would make it unable to finish the mission.
        Do not consider optimizations related to speed, latency, or communication.
        Your output should be of this format:
        Rating: #
        Modified_Plan:
        Drone Plan:
        Phase 1: ----------------------------
        Phase 2: ---------------------------
        -------
        Dog Plan:
        Phase 1: -----------------------------
        Phase 2: ------------------------------
        In case of no modifcation, give the plan a rating of 10 and print the same plan in the modified plan. Do not write any explanation or reasons for the modification.
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
        new_rating, modified_plan = extract_rating_and_plan(response.choices[0].message.content)
        print(response.choices[0].message.content)
        print("Rating:", new_rating)
        print("Modified Plan:", modified_plan)
        if(new_rating <= rating or new_rating == 10 or count == 2):
            break
        rating = new_rating
        generated_plan = modified_plan
        count += 1



    # print("Mission Plan is written to plan.txt")
if __name__ == "__main__":
    main()