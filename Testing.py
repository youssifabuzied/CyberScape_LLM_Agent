import openai
import os
# Initialize your OpenAI client (replace with your actual API key)
client = openai.OpenAI(
    api_key="b37a4309-f1a2-4fd9-b015-eacac68fd6e5",
    base_url="https://api.sambanova.ai/v1",
)

# Initialize an empty message history list
message_history = [
    {"role": "system", "content": "You are a helpful assistant"},
    {"role": "user", "content":'''Mission Scenario: We have a drone and a robot dog. 
    We also have a mission to pick up a ball from a field of size 50x50 meters. The goal is to locate a ball residing in a 1x1 cell.
    The drone can fly over the field and can determine the 10x10 meters square having the ball. The drone is initially located at point (5,5). Also, it can communicate
    with the robot dog when necessary. The dog robot must scan the area, locate the ball, and bring it back to the starting point located at point (25,25) and has to start scanning from there.
    While scanning the area, the dog can scan up to 2 meters in all directions. So, for example, if it is located at x = 10, it can see the range of x = 8 to x = 12 only) and same for the y direction. 
    Generate a high-level plan for the collaboration between the drone and the robot dog to finish this mission. Make sure that the drone follows a systematic approach to cover the whole field area. 
    Your plan should be of this format:
    Drone Plan:
    Phase 1: ----------------------------
    Phase 2: ---------------------------
    -------
    Dog Plan:
    Phase 1: -----------------------------
    Phase 2: ------------------------------'''},
]

# Request initial response from the model
response = client.chat.completions.create(
    model='Meta-Llama-3.1-405B-Instruct',
    messages=message_history,
    temperature=0.1,
    top_p=0.1
)

# # Update message history with the assistant's response
# message_history.append({"role": "assistant", "content": response.choices[0].message.content})

# # Now, user asks for the next part (lower-level plan)
# message_history.append({
#     "role": "user",
#     "content": "Translate the plan you just generated to be a little bit lower level. Try to make them as a set of instructions that do not require further planning. Do not use the word repeat. Rather, provide all the steps needed even if they follow the same pattern. You need to provide an exhaustive list of instructions as they will be given to a robot who needs them to be straightforward. Make sure that the whole area is scanned and covered."
# })

# # Request the second response from the model with updated history
# response = client.chat.completions.create(
#     model='Meta-Llama-3.1-405B-Instruct',
#     messages=message_history,
#     temperature=0.1,
#     top_p=0.1
# )

# # Update message history with the assistant's second response
# message_history.append({"role": "assistant", "content": response.choices[0].message.content})

# Write the final mission plan to a file
with open("Plans/initial_plan.txt", "w") as file:
    file.write(response.choices[0].message.content)

print("Mission Plan is written to Plans/initial_plan.txt")
