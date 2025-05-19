CyberScape\_LLM\_Agent
======================

Overview
--------

CyberScape\_LLM\_Agent is a sophisticated framework for autonomous robotic mission planning that leverages Large Language Models (LLMs) to generate, verify, and execute multi-agent robotic missions. The system employs a hierarchical planning approach that transforms high-level mission objectives into executable low-level instructions for different robotic platforms.

System Architecture
-------------------

The framework implements a multi-layered planning architecture:

1.  **High-Level Planning**: Generates abstract mission plans with independent phases for each robotic agent
    
2.  **Low-Level Planning**: Translates high-level objectives into executable instructions for specific robotic platforms
    
3.  **Verification System**: Evaluates and refines plans based on predefined rubrics
    
4.  **Plan Parsing**: Formats verified plans into robot-executable commands
    
5.  **Adaptive Plan Management**: Handles execution failures and generates recovery strategies
    
6.  **Execution Management**: Orchestrates the execution flow across planning components Manager.py:26-30
    
7.  **Communication Server**: Provides API endpoints for robot coordination and plan execution comm-server.py:9-13
    

Prerequisites
-------------

*   Python 3.8+
*   OpenAI API key or alternatives (LLaMa, Gemini) for accessing LLM capabilities
*   Required Python packages (specified in requirements.txt):
    *   langchain
    *   openai
    *   pydantic
    *   python-dotenv
    *   flask
    *   flask-cors

Installation
------------

1.  Clone the repository:
    
        git clone https://github.com/youssifabuzied/CyberScape_LLM_Agent.git  
        cd CyberScape_LLM_Agent  
        
    
2.  Install dependencies:
    
        pip install -r requirements.txt  
        
    
3.  Configure the system:
    
    *   Add your API keys to the `config.json` file
    *   Modify robot specifications in the `mission_files` directory as needed

Usage
-----

The CyberScape\_LLM\_Agent system uses a communication server to coordinate robot actions during mission execution. The server provides RESTful API endpoints for robots to retrieve instructions, report completion, and handle errors.

### Communication Server

Start the communication server:

    python comm-server.py  
    

The server runs on port 5000 and provides the following API endpoints:

1.  **GET /get\_instruction**
    *   Purpose: Retrieves instructions for a specific robot and phase
    *   Required parameters:
        *   `robot`: Robot identifier (e.g., "DRONE", "ROBOT\_DOG")
        *   `phase`: Phase number (integer)
    *   Response: JSON object containing phase instructions and expected outputs comm-server.py:235-239
2.  **POST /complete\_phase**
    *   Purpose: Marks a phase as completed and stores outputs for use in subsequent phases
    *   Required JSON body:
        *   `robot`: Robot identifier
        *   `phase`: Phase number
        *   `outputs`: Object containing output values as specified in the plan
    *   Response: Confirmation message and stored outputs comm-server.py:181-185
3.  **POST /report\_error**
    *   Purpose: Reports execution errors for adaptive correction
    *   Required JSON body:
        *   `robot`: Robot identifier
        *   `phase`: Phase number
        *   `instruction_number`: Index of the failed instruction
        *   `description`: Error description
    *   Response: Updated plan with corrected instructions comm-server.py:32-36
4.  **POST /generate\_plan**
    *   Purpose: Generates a new mission plan from text
    *   Required JSON body:
        *   `mission_title`: Title for the mission
        *   `mission_text`: Detailed mission description
    *   Response: Confirmation of plan generation comm-server.py:91-95

### Example API Usage

    # Get instructions for phase 1  
    requests.get("http://localhost:5000/get_instruction?robot=DRONE&phase=1")  
      
    # Complete a phase with outputs  
    requests.post("http://localhost:5000/complete_phase",   
                  json={"robot": "DRONE", "phase": 1, "outputs": {"detection_status": "detected", "X": 25.5, "Y": 30.2}})  
      
    # Report an error  
    requests.post("http://localhost:5000/report_error",   
                  json={"robot": "ROBOT_DOG", "phase": 2, "instruction_number": 3, "description": "Unable to navigate to coordinates"})  
      
    # Generate a new plan  
    requests.post("http://localhost:5000/generate_plan",   
                  json={"mission_title": "Ball Retrieval", "mission_text": "Find and retrieve a red ball in the field."})

Notes:

*   The comm-server.py file implements a Flask server that handles communication between robots and the planning system.
*   The server loads plans from JSON files and manages the execution flow between robots.
*   The API endpoints enforce phase ordering to ensure phases are completed sequentially.


### Testing Framework

The system includes a comprehensive testing framework to validate mission plans before deployment:

1.  Import the testing framework:
    
        from Testing.testing_framework import MultiRobotTestingFramework
    
2.  Initialize the framework with mission plans:
    
        framework = MultiRobotTestingFramework(  
            "Plans/mission_plan.json",  
            "Plans/final_drone_low_level_plan.json",  
            "Plans/final_dog_low_level_plan.json"  
        )
    
3.  Run the simulation and get metrics:
    
        metrics = framework.run_simulation()
    
4.  The simulation will:
    
    *   Simulate drone and robot dog movements testing\_framework.py:334-338
    *   Track robot positions and object detection
    *   Handle inter-robot communication
    *   Measure key performance metrics:
        *   Executability (% of actions robots can execute)
        *   Task Completion Rate
        *   Success Rate
        *   Area Coverage

System Components
-----------------

### High-Level Plan Generator

Generates structured mission plans with phases for each robot. Each phase has state descriptions, targets, inputs, and outputs.

### Low-Level Planner

Transforms abstract plans into concrete, executable robot instructions.

### Verification Module

Ensures plans meet quality criteria and refines them when necessary.

### Plan Parser

Formats verified plans for proper robot execution.

### Adaptive Plan Management

Handles execution errors and generates recovery strategies.

### Communication Server

Provides RESTful API endpoints for robot coordination and plan execution. comm-server.py:9-13

### Testing Framework

Simulates mission execution to validate plans before deployment. testing\_framework.py:5-9

Robot Types
-----------

The system currently supports two robot types:

1.  **Drone**: Responsible for aerial operations like scanning and providing location data
    
2.  **Robot Dog**: Handles ground operations and object interaction
    

File Structure
--------------

    CyberScape_LLM_Agent/  
    ├── Plans/                  # Output directory for generated plans  
    ├── mission_files/          # Mission specifications and robot capabilities  
    ├── Testing/                # Testing framework components  
    │   ├── testing_framework.py  # Multi-robot simulation framework  
    │   ├── execution_module.py   # Action execution module  
    │   └── robot.py              # Robot model implementation  
    ├── APM.py                  # Adaptive Plan Management  
    ├── High_Level_Plan_Generator.py  # High-level planning module  
    ├── Low_Level_Planner.py    # Low-level instruction generation  
    ├── Manager.py              # Orchestration and execution control  
    ├── Plan_Parser.py          # Plan formatting for execution  
    ├── comm-server.py          # Communication server for robot interaction  
    ├── Utils.py                # Utility functions  
    ├── Verification_Module.py  # Plan verification and improvement  
    └── config.json             # System configuration  
    

Authors and Contributors
------------------------

*   Youssif Abuzied
*   Abdelaziz Zakareya
*   Ibrahim Gohar
*   Mostafa Mahmoud
*   Zein Noureddin
*   Farida Mossaad

Citation
--------

If you use this system in your research, please cite as follows:

    @software{CyberScape_LLM_Agent,  
      author = {Abuzied, Youssif and Zakareya, Abdelaziz and Gohar, Ibrahim and Mahmoud, Mostafa and Noureddin, Zein and Mossaad, Farida},  
      title = {CyberScape_LLM_Agent: A Framework for LLM-based Robotic Mission Planning},  
      year = {2023},  
      url = {https://github.com/youssifabuzied/CyberScape_LLM_Agent}  
    }  
    

Notes
-----

The CyberScape\_LLM\_Agent project is a comprehensive framework for autonomous multi-robot mission planning and execution. The system uses a communication server (comm-server.py) to coordinate robot actions and handle real-time execution, while the testing framework allows for simulation-based validation before deployment. The project supports collaborative missions between aerial drones and ground-based robot dogs, with extensive error recovery capabilities.
