import ast
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

# Step 1: Define the RobotDog and Drone commands with expected arguments
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

# Step 2: Load a pre-trained SentenceTransformer model
model = SentenceTransformer('all-MiniLM-L6-v2')

# Step 3: Embed the function names for both Dog and Drone
command_embeddings = {
    entity: {
        command: model.encode(command)
        for command in commands.keys()
    }
    for entity, commands in COMMANDS.items()
}

# Step 4: Parse the LLM-generated plan using the `ast` module
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

# Step 5: Validate the parsed plan using cosine similarity and argument checking
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
        print(f"Similarity scores for '{function_name}' ({entity}): {similarities}")

        # # Find the best match
        # best_match = max(similarities, key=similarities.get)
        # similarity_score = similarities[best_match]

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
                results.append(f"'{function_name}' matched to '{best_match}' in '{entity}' with similarity {similarity_score:.2f} and valid arguments.")
        else:
            results.append(f"Error: '{function_name}' did not match any known function in '{entity}'.")

    match_score = exact_match_score / len(parsed_plan)
    print(f"Exact matches: {match_score}")
    return results, new_plan

# Step 6: Example LLM-generated plan
# Test 1: All valid commands
plan_text_valid = """
RobotDog.move_to((10.0, 20.0))  # Valid
RobotDog.rotate(90.0)           # Valid
RobotDog.detect_with_camera()   # Valid
RobotDog.jump()                 # Valid
RobotDog.get_lidar_info()       # Valid
RobotDog.check_obstacle_height()  # Valid
RobotDog.check_distance_to_object()  # Valid
RobotDog.get_position_data()    # Valid
RobotDog.process_messages()     # Valid
RobotDog.wait_for_signal()      # Valid
RobotDog.communicate_with_apm() # Valid
RobotDog.monitor_task("patrol") # Valid
RobotDog.send_feedback_for_rethinking()  # Valid
RobotDog.return_to_base()       # Valid

Drone.move_forward(15.0)     # Valid
Drone.move_to_point((50.0, 30.0, 10.0))  # Valid
Drone.rotate(45.0)           # Valid
Drone.get_position_data()    # Valid
Drone.get_camera_data()      # Valid
Drone.detect_with_camera()   # Valid
Drone.scan_area((10.0, 20.0, 5.0), (15.0, 25.0, 10.0))  # Valid
Drone.process_messages()     # Valid
Drone.wait_for_signal()      # Valid
Drone.communicate_with_apm() # Valid
Drone.monitor_task("delivery")  # Valid
Drone.send_feedback_for_rethinking()  # Valid
Drone.fly(100.0)             # Valid
Drone.return_to_base()       # Valid
"""

plan_text_invalid = """
RobotDog.move_to([10.0, 20.0])  # Invalid: List instead of tuple
RobotDog.rotate("ninety")       # Invalid: String instead of float
RobotDog.detect_with_camera(42) # Invalid: Unexpected argument
RobotDog.jump(5)                # Invalid: Unexpected argument
RobotDog.get_lidar_data()       # Invalid: Non-existent function
RobotDog.check_obstacle_height("low")  # Invalid: String instead of no arguments
RobotDog.check_distance_to_object("far")  # Invalid: String instead of no arguments
RobotDog.get_position_data((10, 10))  # Invalid: Unexpected argument
RobotDog.process_message()      # Invalid: Non-existent function
RobotDog.wait_for_signal("start")  # Invalid: Unexpected argument
RobotDog.communicate_with_apm("message")  # Invalid: Unexpected argument
RobotDog.monitor_task(123)      # Invalid: Integer instead of string
RobotDog.send_feedback()        # Invalid: Non-existent function
RobotDog.return_to_base(5)      # Invalid: Unexpected argument

Drone.move_forward("fast")   # Invalid: String instead of float
Drone.move_to_point([50.0, 30.0, 10.0])  # Invalid: List instead of tuple
Drone.rotate("forty-five")   # Invalid: String instead of float
Drone.get_position_data(5)   # Invalid: Unexpected argument
Drone.get_camera_data("image")  # Invalid: Unexpected argument
Drone.detect_with_camera(10, 20)  # Invalid: Unexpected arguments
Drone.scan_area((10, 20), (15, 25))  # Invalid: Tuples with missing elements
Drone.process_message()      # Invalid: Non-existent function
Drone.wait_for_signal(10)    # Invalid: Unexpected argument
Drone.communicate_with_apm("error")  # Invalid: Unexpected argument
Drone.monitor_task(12345)    # Invalid: Integer instead of string
Drone.send_feedback()        # Invalid: Non-existent function
Drone.fly("high")            # Invalid: String instead of float
Drone.return_to_base("now")  # Invalid: String instead of no arguments
"""

plan_text_nonexistent = """
RobotDog.get_lidar_data()       # Invalid: Non-existent function
RobotDog.process_message()      # Invalid: Non-existent function
RobotDog.send_feedback()        # Invalid: Non-existent function
Drone.send_feedback()        # Invalid: Non-existent function
Drone.report_back()
RobotDog.go_to((10.0, 20.0))  # Expected Mapping: RobotDog.move_to((10.0, 20.0))
RobotDog.turn(90.0)  # Expected Mapping: RobotDog.rotate(90.0)
RobotDog.gather_sensor_data()  # Expected Mapping: RobotDog.get_lidar_info()
RobotDog.assess_obstacle_height()  # Expected Mapping: RobotDog.check_obstacle_height()
RobotDog.measure_distance_to_object()  # Expected Mapping: RobotDog.check_distance_to_object()
RobotDog.current_position()  # Expected Mapping: RobotDog.get_position_data()
RobotDog.process_incoming_messages()  # Expected Mapping: RobotDog.process_messages()
RobotDog.wait_for_instruction()  # Expected Mapping: RobotDog.wait_for_signal()
RobotDog.communicate_with_central_system()  # Expected Mapping: RobotDog.communicate_with_apm()
RobotDog.evaluate_task_progress("delivery")  # Expected Mapping: RobotDog.monitor_task()
RobotDog.provide_feedback_for_replanning()  # Expected Mapping: RobotDog.send_feedback_for_rethinking()
RobotDog.return_to_starting_point()  # Expected Mapping: RobotDog.return_to_base()
RobotDog.check_obstacle_height_in_front()  # Expected Mapping: RobotDog.check_obstacle_height()
RobotDog.determine_distance_to_closest_object()  # Expected Mapping: RobotDog.check_distance_to_object()
RobotDog.get_current_position()  # Expected Mapping: RobotDog.get_position_data()
RobotDog.handle_incoming_messages()  # Expected Mapping: RobotDog.process_messages()
RobotDog.pause_until_signal()  # Expected Mapping: RobotDog.wait_for_signal()
RobotDog.exchange_data_with_central_station()  # Expected Mapping: RobotDog.communicate_with_apm()
RobotDog.assess_task_completion("delivery")  # Expected Mapping: RobotDog.monitor_task()
RobotDog.offer_suggestions_for_replanning()  # Expected Mapping: RobotDog.send_feedback_for_rethinking()
RobotDog.return_to_home_base()  # Expected Mapping: RobotDog.return_to_base()
"""

# Parse the valid plan
parsed_valid_plan = parse_plan(plan_text_valid)
validation_results_valid, new_plan_valid = validate_plan(parsed_valid_plan)

# Print results for the valid plan
print("Test 1: All Valid Commands")
print("Validation Results:")
for result in validation_results_valid:
    print(result)

print("\nNew Plan:")
for command in new_plan_valid:
    print(command)

# Parse the invalid plan
parsed_invalid_plan = parse_plan(plan_text_nonexistent)
validation_results_invalid, new_plan_invalid = validate_plan(parsed_invalid_plan)

# Print results for the invalid plan
print("\nTest 2: Invalid Commands")
print("Validation Results:")
for result in validation_results_invalid:
    print(result)

print("\nNew Plan:")
for command in new_plan_invalid:
    print(command)
