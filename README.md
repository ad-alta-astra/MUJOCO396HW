# 396 FINAL PROJECT

## how to run the program

make sure you have dm_control.mujoco, mujoco.viewer, time, numpy, random and matplotlib.pyplot library installed prior to runing the program

in the folder /final_project run the command: 
## python simulate_re.py 

optional:
you can plot the graph by command: 
## python plot.py

edit the number of rounds and number of children in line 274 to change the process

# explanation of the code
In this program, I want the spider (the model) to move as far as away from the starting point. In additon, I want the spider to move as stable as possible. Thus, I added the penalty for unstability in my fitness algorithm.

At first, a parent spider model will be generated. And then number of children will be generated based on the parent spider.
The children of parent spider have these ways to choose to evolve: regenerating a new head, removing a head, regenerating a new body, removing limbs and adding new limbs. These evolving ways are chosen based on random probability. In addition, there will be at least 1 limb and at most 8 limbs as restrictions. 
After the simulation of all the children, the one with highest fitness value will be chosen as the parent of next generation. And then, this new parent will generate a number of new children with different evolving strategy. And it just goes on until the number of generation has reached.

you can check the graphs on my youtube video: https://www.youtube.com/watch?v=BGv8oJOBfkQ

# references:
code from chatgpt:
polt method in plot.py
fitness evaluation function in simulate_re.py
