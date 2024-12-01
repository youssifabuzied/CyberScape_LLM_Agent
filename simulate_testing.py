import os
from Manager import read_file
from Similarity_Mapping_Module import parse_plan, validate_plan

def simulate_testing(plan_file):
    # Read the plan from the file
    with open(plan_file, "r") as file:
        plan_content = file.read()

    # Parse the plan
    parsed_plan = parse_plan(plan_content)
    
    # Validate the plan using the similarity module
    validation_results, new_plan = validate_plan(parsed_plan)

    # Check if there are errors
    if validation_results:
        print(f"Errors found in the plan from {plan_file}. Regenerating the plan...\n")
        print("Errors:")
        for error in validation_results:
            print(error)
    else:
        print(f"Plan from {plan_file} is valid and error-free.\n")
        print("Generated Plan:")
        for command in new_plan:
            print(command)


def main():
    # Paths to test plans
    valid_plan_path = "testing_plans/valid_plan.txt"
    invalid_plan_path = "testing_plans/nonexistent_commands_plan.txt"
    
    # Simulate testing with the valid plan
    print("Testing with valid plan:")
    simulate_testing(valid_plan_path)

    # Simulate testing with the invalid plan
    print("\nTesting with invalid plan:")
    simulate_testing(invalid_plan_path)


if __name__ == "__main__":
    main()
