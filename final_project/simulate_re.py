import dm_control.mujoco
from dm_control import mjcf
import mujoco.viewer
import time
import numpy as np
import random
import matplotlib.pyplot as plt

class Spider:
    # we should not do anything here
    def __init__(self):
        # provide very important context
        self.model = mjcf.RootElement()
        self.model.option.timestep = 0.01
        self.model.visual.headlight.ambient = [0.5, 0.5, 0.5]
        self.model.worldbody.add('light', diffuse='.5 .5 .5', pos='0 0 3', dir='0 0 -1')
        self.model.worldbody.add('geom', type='plane', size='50 50 0.1')

        self.main_body = self.model.worldbody.add('body', pos='0 0 4')
        self.main_body.add('joint', type='free')
        
        #initialization
        self.body_size = []
        # generate x , y , z
        
        self.head_size = []
        
        self.limb_positions_and_eulers = []
        
        self.all_specs = []
        
        ###################
        
        self.limbs = []
        
        self.limbs_back = []
        
        # self.back_limb_pe = []
        
        # self.num_limbs = 0
        # self.leg_spec = [0.1, 0.1]
        # self.leg_back_spec = [0.1, 0.1]
        self.name = "nanashinobot.xml"
    
    def changeNameTo(self, newName):
        self.name = newName

    def check_limb_num(self):
        return len(self.limb_positions_and_eulers)
    
    def create_new(self):
        # create a body
        self.body_size.append(random.uniform(0.15, 0.4))
        self.body_size.append(random.uniform(0.15, 0.4))
        self.body_size.append(random.uniform(0.1, 0.3))
        
        self.all_specs.append(self.body_size)
        
        # create a head
        self.head_size.append(random.uniform(0.01, self.body_size[0] / 2.0))
        self.head_size.append(random.uniform(0.01, self.body_size[1] / 2.0))
        self.head_size.append(random.uniform(0.01, self.body_size[2] / 2.0))
        
        self.all_specs.append(self.head_size)
        
        init_legs_num = 2
        # create positions for limb
        for i in range(init_legs_num):
            # 0x 1y 2z(also height) 3 diameter
            self.limb_positions_and_eulers.append([random.uniform(-1 * self.body_size[0], self.body_size[0]), random.choice([1, -1]) * self.body_size[1], random.choice([1, -1]) * random.uniform(0.1, 0.5), random.uniform(0.01, 0.04)])
        
        self.all_specs.append(self.limb_positions_and_eulers)

    
    def provide_parent(self, parent):
        # parent = [body, head, limb]
        self.all_specs = parent
        
        # get the body
        self.body_size = parent[0]
        
        # get the head
        self.head_size = parent[1]
        
        # get the limbs
        self.limb_positions_and_eulers = parent[2]
    
    def evlove(self):
        # choose to evolve the body or not
        if random.choice([True, False]):
            # generate a new body
            self.body_evolve()
            
        # choose to evolve the head or not
        # if True:
        if random.choice([True, False]):
            # generate a new head
            self.head_evolve()
        
        # choose to evolve the limbs or not
        if True:
        # if random.choice([True, False]):
            # generate limbs
            self.leg_evolve()
    
    def body_evolve(self):
        self.body_size = []
        # generate a new body
        self.body_size.append(random.uniform(0.15, 0.4))
        self.body_size.append(random.uniform(0.15, 0.4))
        self.body_size.append(random.uniform(0.1, 0.3))
        
        # update the all specs
        self.all_specs[0] = self.body_size
        
        # change the y coordinates of limbs coordingly
        for single_limb in self.limb_positions_and_eulers:
            # 0x 1y 2z(also height) 3 diameter
            if single_limb[1] < 0:
                single_limb[1] = -1 * self.body_size[1]
            else:
                single_limb[1] = self.body_size[1]
        
        # update the all specs
        self.all_specs[2] = self.limb_positions_and_eulers
        
    def head_evolve(self):
        self.head_size = []
        # choose if to get a new head or be head less
        # if False:
        if random.choice([True, False]):
            # choose a new head
            self.head_size.append(random.uniform(0.01, self.body_size[0] / 2.0))
            self.head_size.append(random.uniform(0.01, self.body_size[1] / 2.0))
            self.head_size.append(random.uniform(0.01, self.body_size[2] / 2.0))
        else:
            # choose to be head less
            pass
        
        # update the all specs
        self.all_specs[1] = self.head_size
        
    def leg_evolve(self):
        # choose to decrease leg or not
        if random.choice([True, False]):
            # Ensure that at least one element remains after deletion
            remaining_elements = len(self.limb_positions_and_eulers) - 1 if len(self.limb_positions_and_eulers) > 1 else 1
            
            delete_max = 6
            # Randomly select number of elements to delete
            num_to_delete = random.randint(0, min(delete_max, remaining_elements))
            
            # Delete random elements
            for _ in range(num_to_delete):
                random_index = random.randint(0, len(self.limb_positions_and_eulers) - 1)
                del self.limb_positions_and_eulers[random_index]
                
        # leg number limiter
        leg_max = 8
        increase_max = 3
        
        # choose to increase leg or not
        if random.choice([True, False]):
            # Randomly select number of elements to add
            num_to_add = random.randint(0, min(leg_max - len(self.limb_positions_and_eulers), increase_max))
            
            # Add random elements
            for _ in range(num_to_add):
                self.limb_positions_and_eulers.append([random.uniform(-1 * self.body_size[0], self.body_size[0]), random.choice([1, -1]) * self.body_size[1], random.choice([1, -1]) * random.uniform(0.1, 0.5), random.uniform(0.01, 0.04)])

        
        # update the all specs
        self.all_specs[2] = self.limb_positions_and_eulers
    
    def generation(self):
        # generate a body
        #Abdomen should be at center!!! X Y Z
        self.main_body.add('geom', type='ellipsoid', size=f'{self.body_size[0]} {self.body_size[1]} {self.body_size[2]}', pos='0 0 0', rgba='0.247 0.035 0.98 1') #, density='10')
        
        
        # generate a head
        #Cephalothorax
        if self.head_size:
            self.main_body.add('geom', type='ellipsoid', size=f'{self.head_size[0]} {self.head_size[1]} {self.head_size[2]}', pos=f'{self.body_size[0] + self.head_size[0]} 0 0', rgba='0.91 0.278 0.071 1') #, density='10')
        
        
        # generate limbs and actuators
        # 0x 1y 2z(also height) 3 diameter
        for idx, single_limb in enumerate(self.limb_positions_and_eulers):
            limb = self.main_body.add('body', name=f'limb{idx}', pos=f"{single_limb[0]} {single_limb[1]} {single_limb[2]}")
            limb.add('geom', type='cylinder', size= f'{single_limb[3]} {abs(single_limb[2])}', rgba='0 1 0 1')
            joint_name = f'{idx}_joint'
            
            # actuator
            limb.add('joint', name=joint_name, type='hinge', axis='0 1 0', pos=f'0 0 {-1 * single_limb[2]}', range='-45 45')
            self.model.actuator.add('motor', name=f'motor_{joint_name}', joint=joint_name, gear=f'{10}', ctrllimited='true', ctrlrange='-45 45')
    
# fitness function beta
def evaluate_fitness(distance, stability_measure, steps_num):
    return distance - np.sqrt(stability_measure) / (10 * steps_num)

# the function for running the simulation
def simulation_running(child):
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
            for limb in range(child.check_limb_num()):  # 6 limbs
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
    return fitness

# create the origin
spider_parent = Spider()
spider_parent.changeNameTo("the_spider.xml")
spider_parent.create_new()
# spider_parent.evlove()
spider_parent.generation()

xml_str = spider_parent.model.to_xml_string()

# Create Creature Structure
xml_filename = "the_spider.xml"

# check spec
# print(spider_parent.all_specs)

# Write the XML string to a file
with open(xml_filename, "w") as file:
    file.write(xml_str)

fitest = spider_parent.all_specs

# generate the parent

#############################################################################################
# edit the number of rounds and number of children here
# generate children and run
generation_num = 10
children_num = 10

# fitnessOfChildren = []

max_fitness = []

for curr_gen in range(generation_num):
    parent_para = fitest
    children_para = []
    fitnessOfChildren = []
    
    # show information
    print(f"-------------------executing Generation {curr_gen} right now-------------------")
    
    # generate children and run simulation
    for child_num in range(children_num):
        
        # create children based on parent
        child = Spider()
        child.provide_parent(parent_para)
        child.evlove()
        child.generation()
        child.changeNameTo(f"children{child_num}.xml")
        child_xml = child.model.to_xml_string()
        with open(f"children{child_num}.xml", "w") as file:
            file.write(child_xml)
        
        # run the simulation
        fitness_value = simulation_running(child)
        # we want to avoid outliers and anomaly
        if fitness_value < 500:
            fitnessOfChildren.append(simulation_running(child))
        
            # store parameter
            children_para.append(child.all_specs)
        
    # find the best fitness
    max_fitness.append(max(fitnessOfChildren))
    highest_fitness_index = fitnessOfChildren.index(max(fitnessOfChildren))
    fitest = children_para[highest_fitness_index]
    
    # record fitness of this generation
    with open(f"fitness_gen_{curr_gen}.txt", "w") as file:
        for i, value in enumerate(fitnessOfChildren):
            file.write(f"children {i} fitness: {value}\n")

# record the maximum fitness
with open("max_fitness_per_gen.txt", "w") as file:
        for i, value in enumerate(max_fitness):
            file.write(f"generation {i} fitness: {value}\n")

# show information
print("-------------------Simulation Complete---------------------------")

# print out a graph

# Create x and y coordinates
x = range(len(max_fitness))
y = max_fitness

# Plot the points
plt.scatter(x, y)

# Add labels and title
plt.xlabel('number of generations')
plt.ylabel('fitness value')
plt.title('the fitness graph for each generations')

# Display the plot
plt.show()
