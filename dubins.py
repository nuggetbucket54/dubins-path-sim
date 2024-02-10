import random
import matplotlib.pyplot as plt

# constants (feel free to play around and change these values)
width=100
height=80
turningRadius = 10

# generates two random coordinates representing drone position and waypoint
def genCoords():  
    return [[random.randint(1,width-1),random.randint(1,height-1)],
            [random.randint(1,width-1),random.randint(1,height-1)]]

# plot settings
fig, ax = plt.subplots(figsize=(8,6))
ax.set_xlim(0,width)
ax.set_ylim(0,height)
ax.set_title('Dubins path sim')


# plot points
m,n = genCoords()
ax.plot(m[0], m[1], 'ro', label='Point 1')
ax.plot(n[0], n[1], 'bo', label='Point 2')



# Display the plot
plt.show()