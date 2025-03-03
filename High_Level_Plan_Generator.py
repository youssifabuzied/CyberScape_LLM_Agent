import os
import argparse
import re
from langchain.chat_models import ChatOpenAI
from langchain.schema import SystemMessage, HumanMessage
from langchain.prompts import PromptTemplate
from Manager import read_file

def extract_rating_and_plan(text):
    """Extracts the rating and modified plan using regex."""
    rating_match = re.search(r'Rating: (\d+)', text)
    plan_match = re.search(r'Modified_Plan:\s*(.*)', text, re.DOTALL)
    
    if rating_match and plan_match:
        rating = int(rating_match.group(1))
        modified_plan = plan_match.group(1).strip()
        return rating, modified_plan
    return None, None

def generate_plan(llm, mission_text):
    """Generates an initial plan using the LLM."""
    prompt = f"""
    {mission_text}
    
    Your plan should be structured as clear, step-by-step instructions.
    Each phase must be a concise and actionable sentence.
    
    Example format:
    
    Drone Plan:
    Phase 1: Take off and navigate to the designated search area.
    Phase 2: Perform a spiral search to scan the entire region.
    -------
    Dog Plan:
    Phase 1: Move to the mission start point and begin ground-level scanning.
    Phase 2: Relay findings to the adaptive planning module before proceeding.
    
    """

    """
    Ensure that:
    - All instructions are straightforward and executable.
    - If an area exceeds scanning capabilities, define a spiral search pattern.
    - Communication between robots happens only through the adaptive planning module.
    
    Do not include any headers, explanations, or additional notes in your response."""
    print(llm)
    response = llm.invoke([SystemMessage(content="You are a helpful assistant"), HumanMessage(content=prompt)])
    return response.content.strip()

def refine_plan(llm, mission_text, generated_plan):
    """Refines the plan iteratively until it meets rubric criteria with a rating of 10."""
    rating = 0
    count = 0
    final_plan = generated_plan

    prompt_template = PromptTemplate(
        input_variables=["mission", "plan"],
        template="""
        We had the following mission:
        {mission}

        And we used an LLM to generate the following high-level plan:
        {plan}

        Evaluate this plan based on the following rubric:

        - **Logical Soundness (1-10):** Is the plan coherent and free of contradictions?
        - **Feasibility (1-10):** Can the robots execute the tasks given their capabilities?
        - **Completeness (1-10):** Does the plan cover all mission objectives?
        - **Communication Rules (Pass/Fail):** Does the plan respect that all communication happens only via the adaptive planning module?

        Provide a structured response as follows:

        ```
        Rating: [Overall Rating (1-10)]
        Issues Identified:
        - [List any logical, feasibility, or completeness issues]
        - [If the plan violates communication rules, mention it]

        Modified_Plan:
        Drone Plan:
        Phase 1: [Provide a clear instruction]
        Phase 2: [Provide a clear instruction]
        -------
        Dog Plan:
        Phase 1: [Provide a clear instruction]
        Phase 2: [Provide a clear instruction]
        ```

        - If the plan is already perfect, assign it a rating of **10** and return the same plan.
        - Do **not** provide explanations—just return the structured output.
        """
    )

    while count < 3:
        response = llm.invoke([HumanMessage(content=prompt_template.format(mission=mission_text, plan=final_plan))])
        new_rating, modified_plan = extract_rating_and_plan(response.content)

        if new_rating is None or new_rating <= rating or new_rating == 10:
            break

        rating = new_rating
        final_plan = modified_plan
        count += 1

    return final_plan


def main():
    parser = argparse.ArgumentParser(description="Generate and refine a high-level plan for the mission")
    parser.add_argument("mission_scenario", help="Path to the mission_scenario.txt file")
    args = parser.parse_args()

    print("Generating high-level mission plan...")

    mission_text = read_file(args.mission_scenario)
    if not mission_text:
        print("Error: Mission scenario file is empty or missing.")
        return

    # Initialize LangChain LLM
    llm = ChatOpenAI(
        api_key="c7e68755-3cfd-4f4a-a695-6a41af9ffd23",
        base_url="https://api.sambanova.ai/v1",
        model_name="Meta-Llama-3.1-405B-Instruct",
        temperature=0.1
    )

    generated_plan = generate_plan(llm, mission_text)
    refined_plan = refine_plan(llm, mission_text, generated_plan)

    with open("Plans/initial_plan.txt", 'w') as output_file:
        output_file.write(refined_plan)

    print("Mission Plan written to Plans/initial_plan.txt")

if __name__ == "__main__":
    main()
