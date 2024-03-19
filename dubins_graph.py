import random, math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc
from matplotlib.widgets import Button, Slider
from dubins_utils import RLR, LRL, RSL, LSR, RSR, LSL
# from gps_utils import GPS_UTILS

# constants (feel free to play around and change these values)
# color names for matplotlib: https://matplotlib.org/stable/gallery/color/named_colors.html
DISCRETIZE_LABELS   =   ["Discretization: OFF", "Discretization: ON"]
DISCRETIZE_COLORS   =   ["lightpink", "palegreen"]
DISCRETIZE_FLAG     =   False
INCREMENTS          =   8
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
    angle = random.uniform(-180, 180)
    return [math.cos(angle*PI/180), math.sin(angle*PI/180)]

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
        try: paths.append(RLR(drone_right, point_right, drone_pos, point_pos, TURNRADIUS))
        except: pass

        try: paths.append(LRL(drone_left, point_left, drone_pos, point_pos, TURNRADIUS))
        except: pass

    # CSC paths only work if drone & waypoint are at least 2 turn-radii apart
    if (dist > 2 * TURNRADIUS):
        try: paths.append(RSR(drone_right, point_right, drone_pos, point_pos, TURNRADIUS))
        except: pass

        try: paths.append(LSL(drone_left, point_left, drone_pos, point_pos, TURNRADIUS))
        except: pass

        try: paths.append(RSL(drone_right, point_left, drone_pos, point_pos, TURNRADIUS))
        except: pass

        try: paths.append(LSR(drone_left, point_right, drone_pos, point_pos, TURNRADIUS))
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
        curve1 = Arc((path[6][0], path[6][1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = path[2][0], theta2 = path[2][1], color = 'purple', linewidth = 1)
        curve2 = Arc((path[7][0], path[7][1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = path[3][0], theta2 = path[3][1], color = 'purple', linewidth = 1)

        ax.plot([path[4][0], path[5][0]], [path[4][1], path[5][1]], color='purple', linewidth = 1)
        ax.add_patch(curve1)
        ax.add_patch(curve2)
    else:
        curve1 = Arc((path[5][0], path[5][1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = path[2][0], theta2 = path[2][1], color = 'purple', linewidth = 1)
        curve2 = Arc((path[6][0], path[6][1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = path[4][0], theta2 = path[4][1], color = 'purple', linewidth = 1)
        curve3 = Arc((path[7][0], path[7][1]), 2*TURNRADIUS, 2*TURNRADIUS, theta1 = path[3][0], theta2 = path[3][1], color = 'purple', linewidth = 1)

        ax.add_patch(curve1)
        ax.add_patch(curve2)
        ax.add_patch(curve3)

# helper function to find appropriate increments for dot path
def increments(angle):
    return int(angle * 180 / PI // 30) + 1 # one point every 20 degrees

# draws the path as a series of discretized points
def dot_draw(path):
    # array of points for drone
    points = []

    if path[1] == "CSC":
        # start/end angles for drone arc
        angle_1A = path[2][0] * PI / 180
        angle_1B = path[2][1] * PI / 180


        angle_1 = ((angle_1B - angle_1A) % (2*PI))
        INCREMENT_A = increments(angle_1)
        angle_increment_1 = angle_1 / INCREMENT_A

        # discretizing points for drone arc
        for i in range(INCREMENT_A + 1):
            temp_x = path[6][0] + TURNRADIUS * math.cos(angle_1A + angle_increment_1 * i)
            temp_y = path[6][1] + TURNRADIUS * math.sin(angle_1A + angle_increment_1 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        # start/end angles for waypoint arc
        angle_2A = path[3][0] * PI / 180
        angle_2B = path[3][1] * PI / 180

        angle_2 = (angle_2B - angle_2A) % (2*PI)
        INCREMENT_B = increments(angle_2)
        angle_increment_2 = angle_2 / INCREMENT_B

        # discretizing points for waypoint arc
        for i in range(INCREMENT_B + 1):
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
        angle_1A = path[2][0] * PI / 180
        angle_1B = path[2][1] * PI / 180

        angle_1 = ((angle_1B - angle_1A) % (2*PI))
        INCREMENT_A = increments(angle_1)
        angle_increment_1 = angle_1 / INCREMENT_A

        # discretizing points for arc 1
        for i in range(INCREMENT_A + 1):
            temp_x = path[5][0] + TURNRADIUS * math.cos(angle_1A + angle_increment_1 * i)
            temp_y = path[5][1] + TURNRADIUS * math.sin(angle_1A + angle_increment_1 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        # start/end angles for arc 2
        angle_2A = path[3][0] * PI / 180
        angle_2B = path[3][1] * PI / 180

        angle_2 = (angle_2B - angle_2A) % (2*PI)
        INCREMENT_B = increments(angle_2)
        angle_increment_2 = angle_2 / INCREMENT_B

        # discretizing points for arc 2
        for i in range(INCREMENT_B + 1):
            temp_x = path[7][0] + TURNRADIUS * math.cos(angle_2A + angle_increment_2 * i)
            temp_y = path[7][1] + TURNRADIUS * math.sin(angle_2A + angle_increment_2 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

        # start/end angles for arc 3
        angle_3A = path[4][0] * PI / 180
        angle_3B = path[4][1] * PI / 180

        angle_3 = (angle_3B - angle_3A) % (2*PI)
        INCREMENT_C = increments(angle_3)
        angle_increment_3 = angle_3 / INCREMENT_C

        # discretizing points for arc 3
        for i in range(INCREMENT_C + 1):
            temp_x = path[6][0] + TURNRADIUS * math.cos(angle_3A + angle_increment_3 * i)
            temp_y = path[6][1] + TURNRADIUS * math.sin(angle_3A + angle_increment_3 * i)

            ax.plot(temp_x, temp_y, color='purple', marker='.')
            points.append([temp_x, temp_y])

    return points

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

# adjusts turn radius
def slider_update(val):
    global TURNRADIUS
    initial = TURNRADIUS

    TURNRADIUS = int(val)

    if initial != TURNRADIUS:
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
slider = Slider(slider_ax, "", 1, 10, valinit=5, valfmt="%i")
slider.on_changed(slider_update)

# Display the plot
plt.show()
