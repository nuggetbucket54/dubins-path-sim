import random, math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc
from matplotlib.widgets import Button, Slider

# constants (feel free to play around and change these values)
# color names for matplotlib: https://matplotlib.org/stable/gallery/color/named_colors.html
DISCRETIZE_LABELS   =   ["Discretization: OFF", "Discretization: ON"]
DISCRETIZE_COLORS   =   ["lightpink", "palegreen"]
DISCRETIZE_FLAG     =   False
INCREMENTS          =   10
PI                  =   math.pi
WIDTH               =   100
HEIGHT              =   80
TURNRADIUS          =   5

# generates two random coordinates representing drone position and waypoint
# (coordinates generated at least 10 units from edge of plot for easier visualization)
def gen_coords(): 
    return [random.randint(10, WIDTH - 10), random.randint(10, HEIGHT - 10)]

# generates a vector representing orientation
def gen_vec():
    x1 = random.uniform(-1, 1)

    # generate y component from x component
    y1 = (1 - x1**2)**.5

    # determine positive/negative y component
    if (random.randint(0, 1) % 2):
        y1 *= -1

    return [x1, y1]

# generates the next waypoint
def next_point(event):
    global drone_pos, drone_vec, point_pos, point_vec, ax

    drone_pos = point_pos
    drone_vec = point_vec

    point_pos = gen_coords()
    point_vec = gen_vec()

    draw(ax)

# iterates through and finds the shortest (optimal) route
def find_best(paths):
    shortest = paths[0][0]
    ind = 0
    for i in range(len(paths)):
        if paths[i][0] < shortest:
            ind = i

    return ind

# finds optimal route
def find_path(ax):
    # finding focii of circles for drone
    drone_angle = math.atan2(drone_vec[1], drone_vec[0])
    drone_left_angle = drone_angle + PI/2
    drone_right_angle = drone_angle - PI/2

    if (drone_left_angle > PI):
        drone_left_angle -= 2*PI    

    if (drone_right_angle < PI):
        drone_right_angle += 2*PI

    drone_left = [drone_pos[0] + TURNRADIUS * math.cos(drone_left_angle), drone_pos[1] + TURNRADIUS * math.sin(drone_left_angle)]
    drone_right = [drone_pos[0] + TURNRADIUS * math.cos(drone_right_angle), drone_pos[1] + TURNRADIUS * math.sin(drone_right_angle)]

    # same thing as above but for waypoint
    pointAngle = math.atan2(point_vec[1], point_vec[0])
    point_left_angle = pointAngle + PI/2
    point_right_angle = pointAngle - PI/2

    if (point_left_angle > PI):
        point_left_angle -= 2*PI    

    if (point_right_angle < PI):
        point_right_angle += 2*PI

    point_left = [point_pos[0] + TURNRADIUS * math.cos(point_left_angle), point_pos[1] + TURNRADIUS * math.sin(point_left_angle)]
    point_right = [point_pos[0] + TURNRADIUS * math.cos(point_right_angle), point_pos[1] + TURNRADIUS * math.sin(point_right_angle)]

    # absolute distance between drone & waypoint for route optimization
    dist = ((point_pos[0] - drone_pos[0])**2 + (point_pos[1] - drone_pos[1])**2)**.5
    paths = []

    # CCC paths only work if drone & waypoint within 4 turn-radii
    if (dist < 4 * TURNRADIUS):
        try: paths.append(RLR(drone_right, point_right, drone_pos, point_pos))
        except: pass

        try: paths.append(LRL(drone_left, point_left, drone_pos, point_pos))
        except: pass

    # CSC paths only work if drone & waypoint are at least 2 turn-radii apart
    if (dist > 2 * TURNRADIUS):
        try: paths.append(RSR(drone_right, point_right, drone_pos, point_pos))
        except: pass

        try: paths.append(LSL(drone_left, point_left, drone_pos, point_pos))
        except: pass

        try: paths.append(RSL(drone_right, point_left, drone_pos, point_pos))
        except: pass

        try: paths.append(LSR(drone_left, point_right, drone_pos, point_pos))
        except: pass

    best_path = paths[find_best(paths)]

    if not DISCRETIZE_FLAG:
        line_draw(best_path)
    else:
        dot_draw(best_path)

    # drone and waypoint positions and orientations if needed:
    # print(f"Drone position: {drone_pos}")
    # print(f"Drone heading: {drone_vec}")
    # print(f"Waypoint position: {point_pos}")
    # print(f"Waypoint heading: {point_vec}")

# draws the path as a line
def line_draw(path):
    if path[1] == "CSC":
        ax.plot([path[4][0], path[5][0]], [path[4][1], path[5][1]], color='purple', linewidth = 1)
        ax.add_patch(path[2])
        ax.add_patch(path[3])
    else:
        ax.add_patch(path[2])
        ax.add_patch(path[3])
        ax.add_patch(path[4])

def dot_draw(path):
    # array of points for drone
    points = []

    if path[1] == "CSC":        
        # start/end angles for drone arc
        angle_1A = path[2].theta1 * PI / 180
        angle_1B = path[2].theta2 * PI / 180

        angle_increment_1 = ((angle_1B - angle_1A) % (2*PI)) / INCREMENTS

        # discretizing points for drone arc
        for i in range(INCREMENTS):
            temp_x = path[6][0] + TURNRADIUS * math.cos(angle_1A + angle_increment_1 * i)
            temp_y = path[6][1] + TURNRADIUS * math.sin(angle_1A + angle_increment_1 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        # start/end angles for waypoint arc
        angle_2A = path[3].theta1 * PI / 180
        angle_2B = path[3].theta2 * PI / 180

        angle_increment_2 = ((angle_2B - angle_2A) % (2*PI)) / INCREMENTS

        # discretizing points for waypoint arc
        for i in range(INCREMENTS):
            temp_x = path[7][0] + TURNRADIUS * math.cos(angle_2A + angle_increment_2 * i)
            temp_y = path[7][1] + TURNRADIUS * math.sin(angle_2A + angle_increment_2 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        x_increment = (path[5][0] - path[4][0]) / INCREMENTS
        y_increment = (path[5][1] - path[4][1]) / INCREMENTS

        # discretizing straight segment (not returned, solely for visualization)
        for i in range(INCREMENTS + 1):
            temp_x = path[4][0] + (x_increment * i)
            temp_y = path[4][1] + (y_increment * i)

            ax.plot(temp_x, temp_y, color='gray', marker='.')
    
    else:
        # start/end angles for arc 1
        angle_1A = path[2].theta1 * PI / 180
        angle_1B = path[2].theta2 * PI / 180

        angle_increment_1 = ((angle_1B - angle_1A) % (2*PI)) / INCREMENTS

        # discretizing points for arc 1
        for i in range(INCREMENTS):
            temp_x = path[5][0] + TURNRADIUS * math.cos(angle_1A + angle_increment_1 * i)
            temp_y = path[5][1] + TURNRADIUS * math.sin(angle_1A + angle_increment_1 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        # start/end angles for arc 2
        angle_2A = path[3].theta1 * PI / 180
        angle_2B = path[3].theta2 * PI / 180

        angle_increment_2 = ((angle_2B - angle_2A) % (2*PI)) / INCREMENTS

        # discretizing points for arc 2
        for i in range(INCREMENTS):
            temp_x = path[7][0] + TURNRADIUS * math.cos(angle_2A + angle_increment_2 * i)
            temp_y = path[7][1] + TURNRADIUS * math.sin(angle_2A + angle_increment_2 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        # start/end angles for arc 3
        angle_3A = path[4].theta1 * PI / 180
        angle_3B = path[4].theta2 * PI / 180

        angle_increment_3 = ((angle_3B - angle_3A) % (2*PI)) / INCREMENTS

        # discretizing points for arc 3
        for i in range(INCREMENTS):
            temp_x = path[6][0] + TURNRADIUS * math.cos(angle_3A + angle_increment_3 * i)
            temp_y = path[6][1] + TURNRADIUS * math.sin(angle_3A + angle_increment_3 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

    return points


# path for right-left-right route
def RLR(p1, p2, drone_pos, point_pos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.acos(D/(4 * TURNRADIUS)) + math.atan2(V[1], V[0])

    # focus of third circle in calculations of route
    p3 = [p1[0] + 2 * TURNRADIUS * math.cos(angle), p1[1] + 2 * TURNRADIUS * math.sin(angle)]

    # vectors from circles 1 & 2 to circle 3
    p1p3 = [p3[0] - p1[0], p3[1] - p1[1]]
    p2p3 = [p3[0] - p2[0], p3[1] - p2[1]]

    # tangent points between circles
    pt1 = [p1[0] + (p1p3[0] / 2), p1[1] + (p1p3[1] / 2)]
    pt2 = [p2[0] + (p2p3[0] / 2), p2[1] + (p2p3[1] / 2)]

    # absolute angles of path
    angle1 = math.atan2(drone_pos[1] - p1[1], drone_pos[0] - p1[0])
    angle2 = math.atan2(pt1[1] - p1[1], pt1[0] - p1[0])
    angle3 = math.atan2(pt1[1] - p3[1], pt1[0] - p3[0])
    angle4 = math.atan2(pt2[1] - p3[1], pt2[0] - p3[0])
    angle5 = math.atan2(pt2[1] - p2[1], pt2[0] - p2[0])
    angle6 = math.atan2(point_pos[1] - p2[1], point_pos[0] - p2[0])

    # arcs that make up path from the above angles
    curve1 = Arc((p1[0], p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = angle2*180/PI, theta2 = angle1*180/PI, color = 'purple', linewidth = 1)
    curve2 = Arc((p3[0], p3[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = angle3*180/PI, theta2 = angle4*180/PI, color = 'purple', linewidth = 1)
    curve3 = Arc((p2[0], p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = angle6*180/PI, theta2 = angle5*180/PI, color = 'purple', linewidth = 1)

    return [(abs(angle1 - angle2) + abs(angle4 - angle3) + abs(angle5 - angle6)) * TURNRADIUS, "CCC", curve1, curve2, curve3, p1, p2, p3]

# path for left-right-left route
def LRL(p1, p2, drone_pos, point_pos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.acos(D/(4 * TURNRADIUS)) + math.atan2(V[1], V[0])

    # focus of third circle in calculations of route
    p3 = [p1[0] + 2 * TURNRADIUS * math.cos(angle), p1[1] + 2 * TURNRADIUS * math.sin(angle)]

    # vectors from circles 1 & 2 to circle 3
    p1p3 = [p3[0] - p1[0], p3[1] - p1[1]]
    p2p3 = [p3[0] - p2[0], p3[1] - p2[1]]

    # tangent points between circles
    pt1 = [p1[0] + (p1p3[0] / 2), p1[1] + (p1p3[1] / 2)]
    pt2 = [p2[0] + (p2p3[0] / 2), p2[1] + (p2p3[1] / 2)]

    # absolute angles of path
    angle1 = math.atan2(drone_pos[1] - p1[1], drone_pos[0] - p1[0])
    angle2 = math.atan2(pt1[1] - p1[1], pt1[0] - p1[0])
    angle3 = math.atan2(pt1[1] - p3[1], pt1[0] - p3[0])
    angle4 = math.atan2(pt2[1] - p3[1], pt2[0] - p3[0])
    angle5 = math.atan2(pt2[1] - p2[1], pt2[0] - p2[0])
    angle6 = math.atan2(point_pos[1] - p2[1], point_pos[0] - p2[0])

    # arcs that make up path from the above angles
    curve1 = Arc((p1[0], p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = angle1*180/PI, theta2 = angle2*180/PI, color = 'purple', linewidth = 1)
    curve2 = Arc((p3[0], p3[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = angle4*180/PI, theta2 = angle3*180/PI, color = 'purple', linewidth = 1)
    curve3 = Arc((p2[0], p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = angle5*180/PI, theta2 = angle6*180/PI, color = 'purple', linewidth = 1)

    return [(abs(angle2 - angle1) + abs(angle3 - angle4) + abs(angle6 - angle5)) * TURNRADIUS, "CCC", curve1, curve2, curve3, p1, p2, p3]

# path for right-straight-right route
def RSR(p1, p2, drone_pos, point_pos):

    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.atan2(V[1], V[0]) + PI/2

    x_diff = TURNRADIUS * math.cos(angle)
    y_diff = TURNRADIUS * math.sin(angle)

    # tangent points of circles
    pf1 = [p1[0] + x_diff, p1[1] + y_diff]
    pf2 = [p2[0] + x_diff, p2[1] + y_diff]

    # angle calculations for drawing curves
    curve1_angle_a = math.atan2(drone_pos[1] - p1[1], drone_pos[0] - p1[0])
    curve1_angle_b = math.atan2(pf1[1] - p1[1], pf1[0] - p1[0])

    curve2_angle_a = math.atan2(point_pos[1] - p2[1], point_pos[0] - p2[0])
    curve2_angle_b = math.atan2(pf2[1] - p2[1], pf2[0] - p2[0])

    # curve objects (actually graphed onto matplotlib)
    curve1 = Arc((p1[0], p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve1_angle_b*180/PI, theta2 = curve1_angle_a*180/PI, color = 'purple', linewidth = 1)
    curve2 = Arc((p2[0], p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve2_angle_a*180/PI, theta2 = curve2_angle_b*180/PI, color = 'purple', linewidth = 1)

    return [(abs(curve1_angle_a - curve1_angle_b) + abs(curve2_angle_b - curve2_angle_a)) * TURNRADIUS + D, "CSC", curve1, curve2, pf1, pf2, p1, p2]

# path for left-straight-left route
def LSL(p1, p2, drone_pos, point_pos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.atan2(V[1], V[0]) + PI/2

    x_diff = TURNRADIUS * math.cos(angle)
    y_diff = TURNRADIUS * math.sin(angle)

    # tangent points of circles
    pf1 = [p1[0] - x_diff, p1[1] - y_diff]
    pf2 = [p2[0] - x_diff, p2[1] - y_diff]

    # angle calculations for drawing curves
    curve1_angle_a = math.atan2(drone_pos[1] - p1[1], drone_pos[0] - p1[0])
    curve1_angle_b = math.atan2(pf1[1] - p1[1], pf1[0] - p1[0])

    curve2_angle_a = math.atan2(point_pos[1] - p2[1], point_pos[0] - p2[0])
    curve2_angle_b = math.atan2(pf2[1] - p2[1], pf2[0] - p2[0])

    # curve objects (actually graphed onto matplotlib)
    curve1 = Arc((p1[0], p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve1_angle_a*180/PI, theta2 = curve1_angle_b*180/PI, color = 'purple', linewidth = 1)
    curve2 = Arc((p2[0], p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve2_angle_b*180/PI, theta2 = curve2_angle_a*180/PI, color = 'purple', linewidth = 1)

    return [(abs(curve1_angle_b - curve1_angle_a) + abs(curve2_angle_a - curve2_angle_b)) * TURNRADIUS + D, "CSC", curve1, curve2, pf1, pf2, p1, p2]

# path for right-straight-left route
def RSL(p1, p2, drone_pos, point_pos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = math.acos(2*TURNRADIUS/D) + math.atan2(V[1], V[0])

    x_diff = TURNRADIUS * math.cos(angle)
    y_diff = TURNRADIUS * math.sin(angle)

    # tangent points of circles
    pf1 = [p1[0] + x_diff, p1[1] + y_diff]
    pf2 = [p2[0] - x_diff, p2[1] - y_diff]

    # angle calculations for drawing curves
    curve1_angle_a = math.atan2(drone_pos[1] - p1[1], drone_pos[0] - p1[0])
    curve1_angle_b = math.atan2(pf1[1] - p1[1], pf1[0] - p1[0])

    curve2_angle_a = math.atan2(point_pos[1] - p2[1], point_pos[0] - p2[0])
    curve2_angle_b = math.atan2(pf2[1] - p2[1], pf2[0] - p2[0])

    # curve objects (actually graphed onto matplotlib)
    curve1 = Arc((p1[0], p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve1_angle_b*180/PI, theta2 = curve1_angle_a*180/PI, color = 'purple', linewidth = 1)
    curve2 = Arc((p2[0], p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve2_angle_b*180/PI, theta2 = curve2_angle_a*180/PI, color = 'purple', linewidth = 1)

    return [(abs(curve1_angle_a - curve1_angle_b) + abs(curve2_angle_a - curve2_angle_b)) * TURNRADIUS + D, "CSC", curve1, curve2, pf1, pf2, p1, p2]

# path for left-straight-right route
def LSR(p1, p2, drone_pos, point_pos):
    V = [p2[0] - p1[0], p2[1] - p1[1]]
    D = (V[0]**2 + V[1]**2)**.5

    angle = -math.acos(2*TURNRADIUS/D) + math.atan2(V[1], V[0])

    x_diff = TURNRADIUS * math.cos(angle)
    y_diff = TURNRADIUS * math.sin(angle)

    # tangent points of circles
    pf1 = [p1[0] + x_diff, p1[1] + y_diff]
    pf2 = [p2[0] - x_diff, p2[1] - y_diff]

    # angle calculations for drawing curves
    curve1_angle_a = math.atan2(drone_pos[1] - p1[1], drone_pos[0] - p1[0])
    curve1_angle_b = math.atan2(pf1[1] - p1[1], pf1[0] - p1[0])

    curve2_angle_a = math.atan2(point_pos[1] - p2[1], point_pos[0] - p2[0])
    curve2_angle_b = math.atan2(pf2[1] - p2[1], pf2[0] - p2[0])

    # curve objects (actually graphed onto matplotlib)
    curve1 = Arc((p1[0], p1[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve1_angle_a*180/PI, theta2 = curve1_angle_b*180/PI, color = 'purple', linewidth = 1)
    curve2 = Arc((p2[0], p2[1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = curve2_angle_a*180/PI, theta2 = curve2_angle_b*180/PI, color = 'purple', linewidth = 1)

    return [(abs(curve1_angle_b - curve1_angle_a) + abs(curve2_angle_b - curve2_angle_a)) * TURNRADIUS + D, "CSC", curve1, curve2, pf1, pf2, p1, p2]

# redraws canvas to include new waypoint + vector
def draw(ax):
    ax.clear()
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(0, HEIGHT)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('Dubins path sim')

    ax.plot(drone_pos[0], drone_pos[1], 'bo', label='Point 1')
    ax.plot(point_pos[0], point_pos[1], 'ro', label='Point 2')
    ax.quiver(drone_pos[0], drone_pos[1], drone_vec[0], drone_vec[1], color='blue')
    ax.quiver(point_pos[0], point_pos[1], point_vec[0], point_vec[1], color='red')
    plt.draw()
    find_path(ax)

# toggles discretization
def discretize_callback(event):
    global DISCRETIZE_FLAG
    DISCRETIZE_FLAG = not DISCRETIZE_FLAG

    discretize_button.label.set_text(DISCRETIZE_LABELS[DISCRETIZE_FLAG])
    discretize_button.color = DISCRETIZE_COLORS[DISCRETIZE_FLAG]
    draw(ax)

# adjusts number of discrete points
def slider_update(val):
    global INCREMENTS
    INCREMENTS = int(val)

    if DISCRETIZE_FLAG:
        draw(ax)

# plot settings
fig, ax = plt.subplots(figsize=(6,6))
ax.set_axisbelow(True)
plt.subplots_adjust(bottom=0.35)

drone_pos = gen_coords()
point_pos = gen_coords()
drone_vec = gen_vec()
point_vec = gen_vec()

draw(ax)

# button for next waypoint
waypoint_ax = plt.axes([0.05, 0.20, 0.9, 0.08])
waypoint_button = Button(waypoint_ax, 'Generate next waypoint', color="lightcyan")
waypoint_button.on_clicked(next_point)

discretize_ax = plt.axes([0.05, 0.10, 0.9, 0.08])
discretize_button = Button(discretize_ax, DISCRETIZE_LABELS[0], color=DISCRETIZE_COLORS[0])
discretize_button.on_clicked(discretize_callback)

slider_ax = plt.axes([0.05, 0.04, 0.85, 0.04])
slider = Slider(slider_ax, "", 2, 20, valinit=5, valfmt="%i")
slider.on_changed(slider_update)

# Display the plot
plt.show()
