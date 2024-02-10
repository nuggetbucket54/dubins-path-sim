import random
import matplotlib.pyplot as plt

# constants (feel free to play around and change these values)
width=100
height=80
turningRadius = 10

def genCoords():
    """generates two random coordinates representing drone position and waypoint"""
    return [[random.randint(1,width-1),random.randint(1,height-1)],
            [random.randint(1,width-1),random.randint(1,height-1)]]


fig, ax = plt.subplots(figsize=(8,6))
m,n = genCoords()
ax.plot(m[0], m[1], 'ro', label='Point 1')
ax.plot(n[0], n[1], 'ro', label='Point 2')

# Add labels and legend
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_title('Plotting Two Points')
ax.legend()

# Display the plot
plt.show()