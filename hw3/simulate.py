import dm_control.mujoco
from dm_control import mjcf
import mujoco.viewer
import time
import numpy as np
import secrets

# Create Creature Structure
xml_filename = "the_spider.xml"

class Spider:
    def __init__(self):
        self.model = mjcf.RootElement()
        self.model.option.timestep = 0.01
        self.model.visual.headlight.ambient = [0.5, 0.5, 0.5]
        self.model.worldbody.add('light', diffuse='.5 .5 .5', pos='0 0 3', dir='0 0 -1')
        self.model.worldbody.add('geom', type='plane', size='50 50 0.1')

        self.main_body = self.model.worldbody.add('body', pos='0 0 4')
        self.main_body.add('joint', type='free')
        #Cephalothorax
        self.main_body.add('geom', type='ellipsoid', size='0.1 0.1 0.05', pos='0 0 0', rgba='0.91 0.278 0.071 1')
        #Abdomen
        self.main_body.add('geom', type='ellipsoid', size='0.15 0.15 0.1', pos='0.2 0 0', rgba='0.247 0.035 0.98 1')

        self.limbs = []
        
        self.limbs_back = []
        
        self.limb_positions_and_eulers = [
            ("0.05 0.15 -0.1"),    # Limb 1
            ("0.05 -0.15 -0.1"),    # Limb 2
            ("0.3 0.2 -0.1"),  # Limb 3
            ("0.3 -0.2 -0.1")  # Limb 4
        ]
        
        self.back_limb_pe = [
            #("0.2 0 0.2"),
            ("0.05 0.15 0.2"),    # Limb 1
            ("0.05 -0.15 0.2"),    # Limb 2
            ("0.3 0.2 0.2"),  # Limb 3
            ("0.3 -0.2 0.2")  # Limb 4
        ]
            
        self.num_limbs = 0
        self.name = "nanashinobot.xml"
    
    def changeNameTo(self, newName):
        self.name = newName
    
    def leg_generation(self):
        for i, (pos) in enumerate(self.limb_positions_and_eulers, start=1):
            self.create_limb(f'limb{i}', pos)
        
        for i, (pos) in enumerate(self.back_limb_pe, start=1):
            self.create_limb_back(f'back_limb{i}', pos)
            
        self.num_limbs = len(self.limbs_back) + len(self.limbs)
    
    # randomize the number of legs
    def leg_randomization(self):
        num_to_remove = secrets.SystemRandom().randint(1, len(self.limb_positions_and_eulers))
        for _ in range(num_to_remove):
            random_index = secrets.SystemRandom().randint(0, len(self.limb_positions_and_eulers) - 1)
            del self.limb_positions_and_eulers[random_index]
            
        num_to_remove = secrets.SystemRandom().randint(1, len(self.back_limb_pe))
        for _ in range(num_to_remove):
            random_index = secrets.SystemRandom().randint(0, len(self.back_limb_pe) - 1)
            del self.back_limb_pe[random_index]
    
    # create a limb for flipping
    def create_limb_back(self, name, pos):
        limb = self.main_body.add('body', name=name, pos=pos)
        limb.add('geom', type='cylinder', size='0.02 0.1', rgba='0 1 0 1')
        joint_name = f'{name}_joint'
        limb.add('joint', name=joint_name, type='hinge', axis='0 1 0', pos='0 0 -0.1', range='-45 45')
        
        self.limbs_back.append(limb)

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
    
    def create_actuator_back(self):
        print(self.limbs_back)
        for idx, limb in enumerate(self.limbs_back):
            joint_name = f'back_limb{idx+1}_joint'
            self.model.actuator.add('motor', name=f'motor_back_{joint_name}', joint=joint_name, gear='1', ctrllimited='true', ctrlrange='-45 45')

def evaluate_fitness(distance, stability_measure, steps_num):
    # Adjusting the fitness function
    # Rewarding distance more and reducing the penalty for stability
    return distance - np.sqrt(stability_measure) / (10 * steps_num)

childNum = 10

# we need to randomly generate childNum of different spider

# firstly the original
spider_creature = Spider()
spider_creature.changeNameTo("the_spider.xml")
spider_creature.leg_generation()
spider_creature.create_actuator()
spider_creature.create_actuator_back()

children = []
# the rest 10 children
for i in range(childNum):
    # change the leg
    child = Spider()
    child.leg_randomization()
    child.leg_generation()
    child.create_actuator()
    child.create_actuator_back()
    children.append(child)

xml_str = spider_creature.model.to_xml_string()

# Write the XML string to a file
with open(xml_filename, "w") as file:
    file.write(xml_str)
    
# write them down in a for loop
for i in range(childNum):
    curr_child = children[i]
    curr_child.changeNameTo(f"children{i}.xml")
    child_xml = curr_child.model.to_xml_string()
    with open(f"children{i}.xml", "w") as file:
        file.write(child_xml)

# Move the Body
model = dm_control.mujoco.MjModel.from_xml_path(xml_filename)
data = dm_control.mujoco.MjData(model)

# Viewing Parameters
total_movt = 1500 
timestep = 0.01  

# # Open the viewer
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     viewer.cam.distance = 5
#     viewer.cam.azimuth = 10
#     viewer.cam.elevation = -15  

#     wave_speed = 2  # Controls the speed of the wave
#     wave_length = 3  # Controls the length of the wave

#     for step in range(total_movt):
#         for limb in range(spider_creature.num_limbs):  # 6 limbs
#             joint_index = limb

#             # Calculate the phase based on limb and part position
#             phase = (limb) * np.pi / wave_length

#             # Create a moving wave based on the step and phase
#             angle = np.deg2rad(45) * np.sin(2 * np.pi * wave_speed * step * timestep + phase)

#             # Apply the angle to the joint
#             data.ctrl[joint_index] = angle*60

#         # Step the simulation and sync the viewer
#         dm_control.mujoco.mj_step(model, data)
#         viewer.sync()
#         time.sleep(timestep)

fitnessOfChildren = []

for child in children:
    # Move the Body
    model = dm_control.mujoco.MjModel.from_xml_path(child.name)
    data = dm_control.mujoco.MjData(model)
    
    #setting initial parameters
    initial_position = np.copy(data.qpos)
    total_distance = 0
    stability_measure = 0

    # Viewing Parameters
    total_movt = 1500 
    timestep = 0.01  
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 5
        viewer.cam.azimuth = 10
        viewer.cam.elevation = -15 

        wave_speed = 2  # Controls the speed of the wave
        wave_length = 3  # Controls the length of the wave

        for step in range(total_movt):
            for limb in range(child.num_limbs):  # 6 limbs
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
            
            # fitness metrics
            total_distance += np.linalg.norm(data.qpos[:2] - initial_position[:2])
            initial_position = np.copy(data.qpos)
            stability_measure += np.sum(data.qvel**2)
    
    fitness = evaluate_fitness(total_distance, stability_measure, total_movt)
    fitnessOfChildren.append(fitness)

with open("fitness.txt", "w") as file:
    for i, value in enumerate(fitnessOfChildren):
        file.write(f"children {i} fitness: {value}\n")
