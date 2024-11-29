def parser(plan_file: str) -> str:
    """
    Processes a plan file:
    - Deletes lines starting with 'Phase'.
    - For lines starting with 'if' or 'if not', removes 'if' or 'if not' and retains the rest of the line.
    - Removes colons (:) if they exist at the end of a filtered line.
    - Ensures the first character of the output plan is always uppercase.

    :param plan_file: Path to the file containing the plan.
    :return: A string containing the filtered plan with proper formatting.
    """
    # Read the plan from the file
    with open(plan_file, 'r') as f:
        plan_lines = f.readlines()

    filtered_lines = []
    for line in plan_lines:
        stripped_line = line.strip()
        # Skip lines starting with "Phase"
        if stripped_line.startswith("Phase"):
            continue
        # Remove "if not" and keep the rest of the line
        elif stripped_line.startswith("if not"):
            filtered_line = stripped_line[7:].strip()
        # Remove "if" and keep the rest of the line
        elif stripped_line.startswith("if"):
            filtered_line = stripped_line[3:].strip()
        else:
            filtered_line = stripped_line

        # Remove colon at the end of the line if it exists
        if filtered_line.endswith(":"):
            filtered_line = filtered_line[:-1].strip()

        if filtered_line:  # Only add non-empty lines
            filtered_lines.append(filtered_line)

    # Join lines and ensure the first character of the plan is uppercase
    for i in range(0, len(filtered_lines)):
        filtered_lines[i] = filtered_lines[i][0].upper() + filtered_lines[i][1:]

    filtered_plan = "\n".join(filtered_lines)
  
    return filtered_plan

# Example usage
plan_file_path = input("Enter the plan file:")

filtered_plan = parser(plan_file_path)
print(filtered_plan)
