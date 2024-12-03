import mujoco_py
import time
import os

# Load the Mujoco model

# Construct the full path to the URDF file
urdf_path = os.path.join( os.getcwd(), 'robot_urdf.urdf')
model = mujoco_py.load_model_from_path(urdf_path)

# Create a simulation instance
sim = mujoco_py.MjSim(model)

# Get the number of joints in the model
num_joints = len(sim.model.joint_names)

# Create a viewer for visualization (optional)
viewer = mujoco_py.MjViewer(sim)

# Run the simulation for a few steps
for _ in range(100):
    # Step the simulation forward
    sim.step()

    # Get the joint states (positions, velocities, etc.)
    joint_angles = sim.data.qpos.tolist()  # qpos stores joint positions

    # Print the joint angles
    print(f"Joint angles: {joint_angles}")

    # Render the simulation
    viewer.render()

    # Sleep to simulate real-time visualization
    time.sleep(1. / 240.)

# Optional: Close the viewer
viewer.finish()
