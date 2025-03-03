from langchain_community.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
import langgraph
from langgraph.graph import StateGraph, END
import asyncio

# Define the state of our system
class RobotState:
    def __init__(self, plan=None, feedback=None):
        self.plan = plan
        self.feedback = feedback

    def __repr__(self):
        return f"RobotState(plan={self.plan}, feedback={self.feedback})"

# Define the nodes in our adaptive planning system
def llm_generate_plan(state):
    """LLM generates an initial plan for the robot dog."""
    llm = ChatOpenAI(
        api_key="c7e68755-3cfd-4f4a-a695-6a41af9ffd23",
        base_url="https://api.sambanova.ai/v1",
        model_name="Meta-Llama-3.1-405B-Instruct",
        temperature=0.1
    )
    prompt = """
    You control a robot dog with the following capabilities:
    - move_forward(distance: float)
    - rotate(angle: float)

    The mission: The robot must move forward 15 meters.

    Generate a step-by-step plan using only the available functions. Do not include any explanations or thought. Just output a plan using the given instructions. 
    """

    response = llm.invoke([
        SystemMessage(content="You are an expert in robot path planning."),
        HumanMessage(content=prompt)
    ])
    
    state.plan = response.content.strip()
    print("\n[LLM INITIAL PLAN GENERATED]:\n", state.plan)
    return state

def dog_execute_plan(state):
    """Simulates the robot dog executing the plan and encountering an obstacle."""
    print("\n[ROBOT DOG EXECUTING PLAN]")

    # Simulating the obstacle after 4 meters
    executed_distance = 4
    if executed_distance < 15:
        state.feedback = f"Obstacle detected at {executed_distance} meters. Cannot proceed further."
        print(f"[ROBOT DOG FEEDBACK]: {state.feedback}")
    else:
        state.feedback = "Mission completed successfully."

    return state

def llm_generate_alternative_plan(state):
    """LLM generates an alternative plan based on feedback from the robot dog."""
    if "Obstacle detected" in state.feedback:
        llm = ChatOpenAI(
            api_key="c7e68755-3cfd-4f4a-a695-6a41af9ffd23",
            base_url="https://api.sambanova.ai/v1",
            model_name="Meta-Llama-3.1-405B-Instruct",
            temperature=0.1
        )
        prompt = f"""
        The initial plan to move forward 15 meters  but the mission failed because after moving 4 meters, the robot hit an obstacle. We are now at 4 meters from the start and we still need to reach the end point.
        Suggest an alternative plan using only:
        - move_forward(distance: float)
        - rotate(angle: float)
        
        Do not include any explanations or thought. Just output a plan using the given instructions. 
        """

        response = llm.invoke([
            SystemMessage(content="You are an expert in robot path planning."),
            HumanMessage(content=prompt)
        ])
        
        state.plan = response.content.strip()
        print("\n[LLM ALTERNATIVE PLAN GENERATED]:\n", state.plan)

    return state

# Define the graph
graph = StateGraph(RobotState)
graph.add_node("GenerateInitialPlan", llm_generate_plan)
graph.add_node("ExecutePlan", dog_execute_plan)
graph.add_node("GenerateAlternativePlan", llm_generate_alternative_plan)

# Define transitions
graph.set_entry_point("GenerateInitialPlan")
graph.add_edge("GenerateInitialPlan", "ExecutePlan")
graph.add_edge("ExecutePlan", "GenerateAlternativePlan")  # If plan fails, try again
graph.add_edge("GenerateAlternativePlan", END)  # End after alternative plan

# Compile the graph
robot_planner = graph.compile()

# Run the adaptive planning process
async def run_simulation():
    state = RobotState()
    async for output in robot_planner.astream(state):
        pass  # Just iterating through states

# Run the async function
asyncio.run(run_simulation())