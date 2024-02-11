import random, math
import matplotlib.pyplot as plt

# constants (feel free to play around and change these values)
width=100
height=80
turningRadius = 10

# generates two random coordinates representing drone position and waypoint
def genCoords():  
    return [[random.randint(0,width),random.randint(0,height)],
            [random.randint(0,width),random.randint(0,height)]]

# generates two vectors representing drone orientation
def genVecs():
    x1 = random.uniform(-1,1)
    x2 = random.uniform(-1,1)

    # generate y component from x component
    y1 = (1-x1**2)**.5
    y2 = (1-x2**2)**.5

    # determine positive/negative y component
    if (random.randint(0,1)%2):
        y1 *= -1
    if (random.randint(0,1)%2):
        y2 *= -1

    return [[x1,y1],[x2,y2]]

# plot settings
fig, ax = plt.subplots(figsize=(8,6))
ax.set_xlim(0,width)
ax.set_ylim(0,height)
ax.set_title('Dubins path sim')

# plot points
M,N = genCoords()
A,B = genVecs()
ax.plot(M[0], M[1], 'ro', label='Point 1')
ax.plot(N[0], N[1], 'bo', label='Point 2')
ax.quiver(M[0], M[1], A[0], A[1], color='red')
ax.quiver(N[0], N[1], B[0], B[1], color='blue')



# Display the plot
plt.show()