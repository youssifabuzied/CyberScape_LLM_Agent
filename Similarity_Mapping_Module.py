import ast
import sys
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

# Define the RobotDog and Drone commands with expected arguments
COMMANDS = {
    "RobotDog": {
        "move_to": [("coordinates", tuple)],
        "rotate": [("angle", float)],
        "detect_with_camera": [("object", str)],
        "jump": [],
        "get_lidar_info": [],
        "check_obstacle_height": [],
        "check_distance_to_object": [],
        "get_position_data": [],
        "process_messages": [],
        "wait_for_signal": [],
        "communicate_with_apm": [("coordinates", tuple)],
        "monitor_task": [("task", str)],
        "send_feedback_for_rethinking": [],
        "return_to_base": []
    },
    "Drone": {
        "move_forward": [("distance", float)],
        "move_to_point": [("coordinates", tuple)],
        "rotate": [("angle", float)],
        "get_position_data": [],
        "get_camera_data": [],
        "detect_with_camera": [("object", str)],
        "scan_area": [("start", tuple), ("end", tuple)],
        "process_messages": [],
        "wait_for_signal": [],
        "communicate_with_apm": [("coordinates", tuple)],
        "monitor_task": [("task", str)],
        "send_feedback_for_rethinking": [],
        "fly": [("height", float)],
        "return_to_base": []
    }
}

# Load a pre-trained SentenceTransformer model
model = SentenceTransformer('all-MiniLM-L6-v2')

# Embed the function names for both Dog and Drone
command_embeddings = {
    entity: {
        command: model.encode(command)
        for command in commands.keys()
    }
    for entity, commands in COMMANDS.items()
}

# Parse the LLM-generated plan using the `ast` module
def parse_plan(plan_text):
    """
    Parses a string of function calls and extracts the method names and arguments.
    """
    tree = ast.parse(plan_text)
    methods_called = []

    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            entity_name = node.func.value.id  # Get the entity (e.g., 'RobotDog', 'Drone')
            method_name = node.func.attr      # Get the method name
            try:
                args = [ast.literal_eval(arg) for arg in node.args]  # Evaluate arguments
            except Exception:
                args = ["Error evaluating argument"]  # Handle non-literal arguments
            methods_called.append({"entity": entity_name, "function": method_name, "arguments": args})

    return methods_called

# Validate the parsed plan using cosine similarity and argument checking
def validate_plan(parsed_plan):
    """
    Validates parsed plan against the RobotDog and Drone commands.
    """
    results = []
    new_plan = []  # To store the matched commands for output
    exact_match_score = 0  # To count how many commands matched exactly

    def is_valid_type(value, expected_type):
        """
        Check if the value matches the expected type.
        Integers are valid for floats, but not the other way around.
        """
        if expected_type == float and isinstance(value, int):
            return True  # Allow ints where floats are expected
        return isinstance(value, expected_type)

    for step in parsed_plan:
        entity = step["entity"]  # Either "RobotDog" or "Drone"
        function_name = step["function"]
        arguments = step["arguments"]

        if entity not in command_embeddings:
            results.append(f"Error: Unknown entity '{entity}'.")
            continue

        # Embed the function name and compute similarity
        function_embedding = model.encode(function_name)
        similarities = {
            command: cosine_similarity([function_embedding], [embedding])[0][0]
            for command, embedding in command_embeddings[entity].items()
        }

        # Debug: Log similarity scores
        # print(f"Similarity scores for '{function_name}' ({entity}): {similarities}")

        # Check for exact match first
        if function_name in command_embeddings[entity]:
            best_match = function_name
            similarity_score = 1.0
            exact_match_score += 1
        else:
            # Find the best match using similarity
            best_match = max(similarities, key=similarities.get)
            similarity_score = similarities[best_match]

        if similarity_score > 0.4:  # Adjusted threshold
            # Validate arguments
            expected_args = COMMANDS[entity][best_match]
            if len(arguments) != len(expected_args):
                results.append(f"Error: Argument count mismatch for '{best_match}' in '{entity}'.")
                continue

            for arg, (arg_name, arg_type) in zip(arguments, expected_args):
                if not is_valid_type(arg, arg_type):
                    results.append(
                        f"Error: Argument type mismatch for '{best_match}' in '{entity}' "
                        f"(expected {arg_name} to be {arg_type}, got {type(arg)})."
                    )
                    break
            else:
                # Add matched command to the new plan
                new_plan.append(f"{entity}.{best_match}({', '.join(map(str, arguments))})")
                # results.append(f"'{function_name}' matched to '{best_match}' in '{entity}' with similarity {similarity_score:.2f} and valid arguments.")
        else:
            results.append(f"Error: '{function_name}' did not match any known function in '{entity}'.")

    match_score = exact_match_score / len(parsed_plan)
    print(f"Exact matches: {match_score}")
    return results, new_plan

# Read the plan from a file
def read_plan_from_file(filename):
    with open(filename, 'r') as file:
        return file.read()

# Main entry point for the program
def main():
    if len(sys.argv) != 2:
        print("Usage: python Similarity_Mapping_Module.py <path_to_plan_file>")
        sys.exit(1)

    filename = sys.argv[1]
    plan_text = read_plan_from_file(filename)

    # Parse and validate the plan
    parsed_plan = parse_plan(plan_text)
    validation_results, new_plan = validate_plan(parsed_plan)

    # Print the results
    if validation_results:
        print("Validation Errors:")
        for result in validation_results:
            print(result)
    else:
        print("No errors found. New plan:")
        for command in new_plan:
            print(command)

if __name__ == "__main__":
    main()
