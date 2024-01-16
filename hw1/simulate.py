import mujoco_py
import numpy as np

# Load the MuJoCo model from XML
model = mujoco_py.load_model_from_path("cartpole.xml")

# Create a MuJoCo simulator
sim = mujoco_py.MjSim(model)

# Specify the number of simulation steps
num_steps = 1000

# Run the simulation
for _ in range(num_steps):
    # Perform some actions, for example, applying torque to the joint
    torque = np.array([0.0])  # Example torque value
    sim.data.ctrl[:] = torque

    # Step the simulation
    sim.step()

    # Print or visualize the state (modify based on your needs)
    print("Time Step:", sim.data.time)
    print("Joint Position:", sim.data.qpos)

# Close the simulation
sim.close()
