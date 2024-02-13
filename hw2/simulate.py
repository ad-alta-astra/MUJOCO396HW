import dm_control.mujoco
from dm_control import mjcf
import mujoco.viewer
import time
import numpy as np

# Create Creature Structure
xml_filename = "the_spider.xml"

class Crab:
    def __init__(self):
        self.model = mjcf.RootElement()
        self.model.option.timestep = 0.01
        self.model.visual.headlight.ambient = [0.5, 0.5, 0.5]
        self.model.worldbody.add('light', diffuse='.5 .5 .5', pos='0 0 3', dir='0 0 -1')
        self.model.worldbody.add('geom', type='plane', size='50 50 0.1')

        self.main_body = self.model.worldbody.add('body', pos='0 0 4')
        self.main_body.add('joint', type='free')
        #Cephalothorax
        self.main_body.add('geom', type='ellipsoid', size='0.1 0.1 0.05', pos='0 0 0')
        #Abdomen
        self.main_body.add('geom', type='ellipsoid', size='0.15 0.15 0.1', pos='0.2 0 0')

        self.limbs = []
        limb_positions_and_eulers = [
            ("0.05 0.15 -0.1"),    # Limb 1
            ("0.05 -0.15 -0.1"),    # Limb 2
            ("0.3 0.2 -0.1"),  # Limb 3
            ("0.3 -0.2 -0.1")  # Limb 4
        ]

        for i, (pos) in enumerate(limb_positions_and_eulers, start=1):
            self.create_limb(f'limb{i}', pos)

    def create_limb(self, name, pos):
        limb = self.main_body.add('body', name=name, pos=pos)
        limb.add('geom', type='cylinder', size='0.02 0.1', rgba='0 1 0 1')
        joint_name = f'{name}_joint'
        limb.add('joint', name=joint_name, type='hinge', axis='0 1 0', pos='0 0 0.1', range='-45 45')
        
        self.limbs.append(limb)

    def create_actuator(self):
        print(self.limbs)
        for idx, limb in enumerate(self.limbs):
            joint_name = f'limb{idx+1}_joint'
            self.model.actuator.add('motor', name=f'motor_{joint_name}', joint=joint_name, gear='1', ctrllimited='true', ctrlrange='-45 45')


crab_creature = Crab()
crab_creature.create_actuator()

xml_str = crab_creature.model.to_xml_string()

# Write the XML string to a file
with open(xml_filename, "w") as file:
    file.write(xml_str)

# Move the Body
model = dm_control.mujoco.MjModel.from_xml_path(xml_filename)
data = dm_control.mujoco.MjData(model)

# Viewing Parameters
total_movt = 1500 
timestep = 0.01  

# Open the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 20
    viewer.cam.azimuth = 60
    viewer.cam.elevation = -15  

    wave_speed = 2  # Controls the speed of the wave
    wave_length = 3  # Controls the length of the wave

    for step in range(total_movt):
        for limb in range(4):  # 6 limbs
            joint_index = limb

            # Calculate the phase based on limb and part position
            phase = (limb) * np.pi / wave_length

            # Create a moving wave based on the step and phase
            angle = np.deg2rad(45) * np.sin(2 * np.pi * wave_speed * step * timestep + phase)

            # Apply the angle to the joint
            data.ctrl[joint_index] = angle*60

        # Step the simulation and sync the viewer
        dm_control.mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timestep)
