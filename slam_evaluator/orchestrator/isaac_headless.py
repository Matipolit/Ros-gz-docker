import argparse
import os
import sys

# We need to set up the environment for SimulationApp
try:
    from omni.isaac.kit import SimulationApp
except ImportError:
    print(
        "Error: Could not import SimulationApp. This script must be run within the Isaac Sim environment."
    )
    sys.exit(1)

parser = argparse.ArgumentParser()
parser.add_argument("--usd_path", required=True, help="Path to the USD world file")
args, unknown = parser.parse_known_args()

# Start SimulationApp in headless mode
# We also set renderer to 'RayTracedLighting' (default) or 'PathTracing'
simulation_app = SimulationApp({"headless": True})

# Import other omni modules AFTER SimulationApp is started
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import open_stage

# Enable ROS 2 Bridge extension
print("Enabling ROS 2 Bridge extension...")
enable_extension("omni.isaac.ros2_bridge")

# Open the USD stage
# Paths in the container are absolute
usd_path = args.usd_path
if not os.path.isabs(usd_path):
    usd_path = os.path.join("/home/ubuntu/shared/worlds", usd_path)

print(f"Opening stage: {usd_path}")
if not os.path.exists(usd_path):
    print(f"Error: USD path {usd_path} does not exist.")
    simulation_app.close()
    sys.exit(1)

open_stage(usd_path)

# Initialize simulation context and start playing
simulation_context = SimulationContext()
print("Starting simulation...")
simulation_context.play()

# Keep the simulation running
# The pipeline will kill this process when done
try:
    while simulation_app.is_running():
        simulation_app.update()
except KeyboardInterrupt:
    print("Simulation stopped by user.")
finally:
    simulation_app.close()
