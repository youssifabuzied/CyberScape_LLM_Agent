import os
import json
import asyncio
import warnings
warnings.filterwarnings("ignore")

async def run_python_script(command):
    try:
        process = await asyncio.create_subprocess_shell(
            command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await process.communicate()
        if stdout:
            print(stdout.decode())
        if stderr:
            print(stderr.decode())
    except Exception as e:
        print(f"Error running command '{command}': {e}")

def load_config():
    with open("config.json", "r") as f:
        return json.load(f)

def main():
    config = load_config()

    print("=== Starting High-Level Planning ===")
    # Run High-Level Planner (now uses config.json internally)
    high_level_command = "python High_Level_Plan_Generator.py"
    asyncio.run(run_python_script(high_level_command))

    for robot in config["robots_in_curr_mission"]:
        print(f"=== Starting Low-Level Planning for {robot} ===")
        # Run unified Low-Level Planner for each robot
        robot_ll_command = f"python Low_Level_Planner.py {robot}"
        asyncio.run(run_python_script(robot_ll_command))

    print("=== All Planning Stages Completed ===")

if __name__ == "__main__":
    main()
