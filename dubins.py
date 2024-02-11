import random, math
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from typing import Tuple

# constants (feel free to play around and change these values)
width=100
height=80
turningRadius = 10

dronePos: Tuple[int, int]
droneVec: Tuple[float, float]
pointPos: Tuple[int, int]
pointVec: Tuple[float, float]

# generates two random coordinates representing drone position and waypoint
# (coordinates generated at least 10 units from edge of plot for easier visualization)
def genCoords(): 
    return [random.randint(10,width-10),random.randint(10,height-10)]

# generates a vector representing orientation
def genVec():
    x1 = random.uniform(-1,1)

    # generate y component from x component
    y1 = (1-x1**2)**.5

    # determine positive/negative y component
    if (random.randint(0,1)%2):
        y1 *= -1

    return [x1,y1]

# generates the next waypoint
def nextPoint(event):
    print("HI!")
    global dronePos, droneVec, pointPos, pointVec

    dronePos = pointPos
    droneVec = pointVec

    pointPos = genCoords()
    pointVec = genVec()

    draw()

def draw(ax):
    
    ax.plot(dronePos[0], dronePos[1], 'ro', label='Point 1')
    ax.plot(pointPos[0], pointPos[1], 'bo', label='Point 2')
    ax.quiver(dronePos[0], dronePos[1], droneVec[0], droneVec[1], color='red')
    ax.quiver(pointPos[0], pointPos[1], pointVec[0], pointVec[1], color='blue')

dronePos = genCoords()
pointPos = genCoords()
droneVec = genVec()
pointVec = genVec()

# plot settings
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_xlim(0,width)
ax.set_ylim(0,height)
ax.set_title('Dubins path sim')

draw(ax)

button_ax = plt.axes([0.05, 0.05, 0.9, 0.1])  # [left, bottom, width, height]
button = Button(button_ax, 'Generate next waypoint')
button.on_clicked(nextPoint)

# Display the plot
plt.show()