import random, math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Arc
from matplotlib.widgets import Button

# constants (feel free to play around and change these values)
width=100
height=80
TURNRADIUS = 5
PI = math.pi

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

def drawPath(ax):
    # finding focii of circles for drone
    droneAngle = math.atan2(droneVec[1], droneVec[0])
    droneLeftAngle = droneAngle + PI/2
    droneRightAngle = droneAngle - PI/2

    if (droneLeftAngle > PI):
        droneLeftAngle -= 2*PI    

    if (droneRightAngle < PI):
        droneRightAngle += 2*PI

    droneLeft = [dronePos[0] + TURNRADIUS * math.cos(droneLeftAngle), dronePos[1] + TURNRADIUS * math.sin(droneLeftAngle)]
    droneRight = [dronePos[0] + TURNRADIUS * math.cos(droneRightAngle), dronePos[1] + TURNRADIUS * math.sin(droneRightAngle)]

    # same thing as above but for waypoint
    pointAngle = math.atan2(pointVec[1], pointVec[0])
    pointLeftAngle = pointAngle + PI/2
    pointRightAngle = pointAngle - PI/2

    if (pointLeftAngle > PI):
        pointLeftAngle -= 2*PI    

    if (pointRightAngle < PI):
        pointRightAngle += 2*PI

    pointLeft = [pointPos[0] + TURNRADIUS * math.cos(pointLeftAngle), pointPos[1] + TURNRADIUS * math.sin(pointLeftAngle)]
    pointRight = [pointPos[0] + TURNRADIUS * math.cos(pointRightAngle), pointPos[1] + TURNRADIUS * math.sin(pointRightAngle)]

    # RSR(droneRight, pointRight, dronePos, pointPos)
    LSL(droneLeft, pointLeft, dronePos, pointPos)

    # listed below are points/lines used for calculations that can be visualized if wanted
    # ax.plot(droneLeft[0], droneLeft[1], 'bo', label='Point 1')
    # ax.plot(droneRight[0], droneRight[1], 'bo', label='Point 1')
    # ax.plot(pointLeft[0], pointLeft[1], 'ro', label='Point 1')
    # ax.plot(pointRight[0], pointRight[1], 'ro', label='Point 1')
    # ax.plot(pf1[0], pf1[1], 'bo', label='Point 1')
    # ax.plot(pf2[0], pf2[1], 'ro', label='Point 2')
    # ax.add_patch(c1)
    # ax.add_patch(c2)

# path for right-straight-right path
def RSR(p1, p2, dronePos, pointPos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.acos(2*TURNRADIUS/D) + math.atan2(V[1],V[0])

    xdiff = TURNRADIUS * math.cos(angle)
    ydiff = TURNRADIUS * math.sin(angle)

    # tangent points of circles
    pf1 = [p1[0] + xdiff, p1[1] + ydiff]
    pf2 = [p2[0] + xdiff, p2[1] + ydiff]

    # angle calculations for drawing curves
    curveAng1A = math.atan2(dronePos[1]-p1[1], dronePos[0]-p1[0])
    curveAng1B = math.atan2(pf1[1]-p1[1], pf1[0]-p1[0])

    curveAng2A = math.atan2(pointPos[1]-p2[1], pointPos[0]-p2[0])
    curveAng2B = math.atan2(pf2[1]-p2[1], pf2[0]-p2[0])

    curve1 = Arc((p1[0],p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1=curveAng1B*180/PI, theta2=curveAng1A*180/PI, color='purple', linewidth=1)
    curve2 = Arc((p2[0],p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1=curveAng2A*180/PI, theta2=curveAng2B*180/PI, color='purple', linewidth=1)

    # add curves to graph
    ax.add_patch(curve1)
    ax.add_patch(curve2)
    ax.plot([pf1[0], pf2[0]], [pf1[1], pf2[1]], color='purple', linewidth=1)

# path for left-straight-left path
def LSL(p1, p2, dronePos, pointPos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.acos(2*TURNRADIUS/D) + math.atan2(V[1],V[0])

    xdiff = TURNRADIUS * math.cos(angle)
    ydiff = TURNRADIUS * math.sin(angle)

    # tangent points of circles
    pf1 = [p1[0] - xdiff, p1[1] - ydiff]
    pf2 = [p2[0] - xdiff, p2[1] - ydiff]

    # angle calculations for drawing curves
    curveAng1A = math.atan2(dronePos[1]-p1[1], dronePos[0]-p1[0])
    curveAng1B = math.atan2(pf1[1]-p1[1], pf1[0]-p1[0])

    curveAng2A = math.atan2(pointPos[1]-p2[1], pointPos[0]-p2[0])
    curveAng2B = math.atan2(pf2[1]-p2[1], pf2[0]-p2[0])

    curve1 = Arc((p1[0],p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1=curveAng1A*180/PI, theta2=curveAng1B*180/PI, color='purple', linewidth=1)
    curve2 = Arc((p2[0],p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1=curveAng2B*180/PI, theta2=curveAng2A*180/PI, color='purple', linewidth=1)

    # add curves to graph
    ax.add_patch(curve1)
    ax.add_patch(curve2)
    ax.plot([pf1[0], pf2[0]], [pf1[1], pf2[1]], color='purple', linewidth=1)

# redraws canvas
def draw(ax):
    ax.clear()
    ax.set_xlim(0,width)
    ax.set_ylim(0,height)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Dubins path sim')

    ax.plot(dronePos[0], dronePos[1], 'bo', label='Point 1')
    ax.plot(pointPos[0], pointPos[1], 'ro', label='Point 2')
    ax.quiver(dronePos[0], dronePos[1], droneVec[0], droneVec[1], color='blue')
    ax.quiver(pointPos[0], pointPos[1], pointVec[0], pointVec[1], color='red')
    plt.draw()
    drawPath(ax)

# plot settings
fig, ax = plt.subplots(figsize=(6,6))
plt.subplots_adjust(bottom=0.2)

dronePos = genCoords()
pointPos = genCoords()
droneVec = genVec()
pointVec = genVec()

draw(ax)

# button for next waypoint
button_ax = plt.axes([0.05, 0.05, 0.9, 0.08])
button = Button(button_ax, 'Generate next waypoint')
button.on_clicked(nextPoint)

# Display the plot
plt.show()
