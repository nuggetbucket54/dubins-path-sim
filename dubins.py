import random, math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
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
    global dronePos, droneVec, pointPos, pointVec, ax

    dronePos = pointPos
    droneVec = pointVec

    pointPos = genCoords()
    pointVec = genVec()

    draw(ax)

# redrawing canvas
def draw(ax):
    ax.clear()
    ax.set_xlim(0,width)
    ax.set_ylim(0,height)
    ax.set_aspect('equal')
    ax.set_title('Dubins path sim')

    ax.plot(dronePos[0], dronePos[1], 'bo', label='Point 1')
    ax.plot(pointPos[0], pointPos[1], 'ro', label='Point 2')
    ax.quiver(dronePos[0], dronePos[1], droneVec[0], droneVec[1], color='blue')
    ax.quiver(pointPos[0], pointPos[1], pointVec[0], pointVec[1], color='red')
    plt.draw()

# plot settings
fig, ax = plt.subplots(figsize=(6,6))
plt.subplots_adjust(bottom=0.25)


dronePos = [20,20]
pointPos = [80,40]
droneVec = [0,1]
pointVec = [0,1]
# droneVec = [0.5,0.75**.5]
# pointVec = [0.5,-0.75**.5]

draw(ax)

p1 = [dronePos[0] + droneVec[1]*turningRadius, dronePos[1]-droneVec[0]*turningRadius]
p2 = [pointPos[0] - pointVec[1]*turningRadius, pointPos[1]-pointVec[0]*turningRadius]
c1 = Circle((p1[0], p1[1]), turningRadius, fill=False, color='blue')
c2 = Circle((p2[0], p2[1]), turningRadius, fill=False, color='blue')

D = ((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)**.5


ax.plot(p1[0], p1[1], 'bo', label='Point 1')
ax.plot(p2[0], p2[1], 'ro', label='Point 2')
ax.add_patch(c1)
ax.add_patch(c2)



button_ax = plt.axes([0.05, 0.05, 0.9, 0.1])  # [left, bottom, width, height]
button = Button(button_ax, 'Generate next waypoint')
button.on_clicked(nextPoint)

# Display the plot
plt.show()