import pybullet as p
import time
import os



# Construct the full path to the URDF file
urdf_path = os.path.join( os.getcwd(), 'robot_urdf.urdf')


# Check if the URDF file exists
if os.path.exists(urdf_path):
    print(f"File found at: {urdf_path}")
else:
    print(f"URDF file not found: {urdf_path}")

# Connect to PyBullet (GUI or DIRECT mode)
p.connect(p.DIRECT)  # Use p.GUI for graphical interface


# Load a URDF model
robot_id = p.loadURDF(urdf_path)

# Get the number of joints in the robot
num_joints = p.getNumJoints(robot_id)


# Set up time measurement
start_time = time.time()  # Get the current time in seconds
simulation_steps = 100000  # Number of simulation steps to run
real_time_factor = 0

# Run simulation and track real-time factor
for i in range(simulation_steps):
    p.stepSimulation()
    real_time = time.time() - start_time
    simulation_time = (i + 1) * p.getPhysicsEngineParameters()['fixedTimeStep']
    
    if real_time > 0:
        real_time_factor = simulation_time / real_time

    print(f"Step {i+1}, Real-time Factor: {real_time_factor:.2f}")
    time.sleep(1. / 240.)


    ##add the controller
    ke: 0.0001
    kv: 1.0
    kx: 1000.0
    kdelta: 0.0001

    # 0.0 angle - upward position. When absolute angle is less than this threshold
    # transition from swing up to LQR
    lqr_transition_angle: 0.523599

        # Get joint states (joint position, velocity, force/torque)
    joint_states = p.getJointStates(robot_id, range(num_joints))
    
    # Extract and print the joint angles
    joint_angles = [joint_state[0] for joint_state in joint_states]
    print(f"Joint angles: {joint_angles}")

    # Extract and print the joint velocities
    joint_velocities = [joint_state[1] for joint_state in joint_states]
    print(f"Joint velocities: {joint_velocities}")

p.disconnect()
