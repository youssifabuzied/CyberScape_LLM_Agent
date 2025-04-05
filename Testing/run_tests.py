# Initialize the testing framework
from testing_framework import MultiRobotTestingFramework

framework = MultiRobotTestingFramework(
    '../Plans/mission_plan.json',
    '../Plans/final_drone_low_level_plan.json',
    '../Plans/final_dog_low_level_plan.json'
)

# Run the simulation and get metrics
metrics = framework.run_simulation()

# Print the results
print("Simulation Results:")
print(f"Executability - Drone: {metrics['executability']['drone']:.2f}, Dog: {metrics['executability']['dog']:.2f}, Overall: {metrics['executability']['overall']:.2f}")
print(f"Task Completion Rate (TCR): {metrics['tcr']:.2f}")
print(f"Success Rate (SR): {metrics['sr']:.2f}")
print("Area Coverage:")
print(f"  Drone: {metrics['covered_cells_drone']} ({metrics['covered_cells_drone']}/{metrics['total_cells_drone']} cells)")
print(f"  Dog: {metrics['covered_cells_dog']} ({metrics['covered_cells_dog']}/{metrics['total_cells_dog']} cells)")
print(f"  Overall: {metrics['area_coverage']['overall']}/{metrics['total_cells_dog']} cells")
print(f"Ball Position: {metrics['ball_position']}")
print(f"Final Drone Position: {metrics['final_drone_position']}")
print(f"Final Dog Position: {metrics['final_dog_position']}")
