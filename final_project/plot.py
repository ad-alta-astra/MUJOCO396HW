import matplotlib.pyplot as plt

max_fitness = []

with open('max_fitness_per_gen.txt', 'r') as file:
    for line in file:
        if 'fitness:' in line:
            fitness_value = float(line.split('fitness:')[1].strip())
            max_fitness.append(fitness_value)

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