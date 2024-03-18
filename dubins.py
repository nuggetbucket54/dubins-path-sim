import math
from dubins_utils import RLR, LRL, RSL, LSR, RSR, LSL
from gps_utils import GPS_utils

INCREMENTS          =   8
PI                  =   math.pi
TURNRADIUS          =   5

# iterates through and finds the shortest (optimal) route
def find_best(paths):
    shortest = paths[0][0]
    ind = 0
    for i in range(len(paths)):
        if paths[i][0] < shortest:
            ind = i

    return ind

# finds optimal route
def find_path(drone_pos, point_pos, drone_vec, point_vec):
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
    return dot_draw(best_path)

# helper function to find appropriate increments for dot path
def increments(angle):
    return int(angle * 180 / PI // 30) + 1 # one point every 20 degrees

# draws the path as a series of discretized points
def dot_draw(path):
    # array of points for drone
    points = []

    if path[1] == "CSC":
        # start/end angles for drone arc
        angle_1A = path[2].theta1 * PI / 180
        angle_1B = path[2].theta2 * PI / 180


        angle_1 = ((angle_1B - angle_1A) % (2*PI))
        INCREMENT_A = increments(angle_1)
        angle_increment_1 = angle_1 / INCREMENT_A

        # discretizing points for drone arc
        for i in range(INCREMENT_A + 1):
            temp_x = path[6][0] + TURNRADIUS * math.cos(angle_1A + angle_increment_1 * i)
            temp_y = path[6][1] + TURNRADIUS * math.sin(angle_1A + angle_increment_1 * i)

            points.append([temp_x, temp_y])

        # start/end angles for waypoint arc
        angle_2A = path[3].theta1 * PI / 180
        angle_2B = path[3].theta2 * PI / 180

        angle_2 = (angle_2B - angle_2A) % (2*PI)
        INCREMENT_B = increments(angle_2)
        angle_increment_2 = angle_2 / INCREMENT_B

        # discretizing points for waypoint arc
        for i in range(INCREMENT_B + 1):
            temp_x = path[7][0] + TURNRADIUS * math.cos(angle_2A + angle_increment_2 * i)
            temp_y = path[7][1] + TURNRADIUS * math.sin(angle_2A + angle_increment_2 * i)

            points.append([temp_x, temp_y])

        x_increment = (path[5][0] - path[4][0]) / INCREMENTS
        y_increment = (path[5][1] - path[4][1]) / INCREMENTS

        # discretizing straight segment (not returned, solely for visualization)
        for i in range(INCREMENTS + 1):
            temp_x = path[4][0] + (x_increment * i)
            temp_y = path[4][1] + (y_increment * i)
    
    else:
        # start/end angles for arc 1
        angle_1A = path[2].theta1 * PI / 180
        angle_1B = path[2].theta2 * PI / 180

        angle_1 = ((angle_1B - angle_1A) % (2*PI))
        INCREMENT_A = increments(angle_1)
        angle_increment_1 = angle_1 / INCREMENT_A

        # discretizing points for arc 1
        for i in range(INCREMENT_A + 1):
            temp_x = path[5][0] + TURNRADIUS * math.cos(angle_1A + angle_increment_1 * i)
            temp_y = path[5][1] + TURNRADIUS * math.sin(angle_1A + angle_increment_1 * i)

            points.append([temp_x, temp_y])

        # start/end angles for arc 2
        angle_2A = path[3].theta1 * PI / 180
        angle_2B = path[3].theta2 * PI / 180

        angle_2 = (angle_2B - angle_2A) % (2*PI)
        INCREMENT_B = increments(angle_2)
        angle_increment_2 = angle_2 / INCREMENT_B

        # discretizing points for arc 2
        for i in range(INCREMENT_B + 1):
            temp_x = path[7][0] + TURNRADIUS * math.cos(angle_2A + angle_increment_2 * i)
            temp_y = path[7][1] + TURNRADIUS * math.sin(angle_2A + angle_increment_2 * i)

            points.append([temp_x, temp_y])

        # start/end angles for arc 3
        angle_3A = path[4].theta1 * PI / 180
        angle_3B = path[4].theta2 * PI / 180

        angle_3 = (angle_3B - angle_3A) % (2*PI)
        INCREMENT_C = increments(angle_3)
        angle_increment_3 = angle_3 / INCREMENT_C

        # discretizing points for arc 3
        for i in range(INCREMENT_C + 1):
            temp_x = path[6][0] + TURNRADIUS * math.cos(angle_3A + angle_increment_3 * i)
            temp_y = path[6][1] + TURNRADIUS * math.sin(angle_3A + angle_increment_3 * i)

            points.append([temp_x, temp_y])

    return points

# initialize gps utility
gps = GPS_utils()

# get position of drone
drone_lat = float(input("Enter drone latitude: "))
drone_long = float(input("Enter drone longitude: "))
drone_pos = [0, 0] # drone always positioned at origin

# set local xy-origin to drone position
gps.setENUorigin(drone_lat, drone_long, 0)

# set position of waypoint
point_lat = float(input("Enter waypoint latitude: "))
point_long = float(input("Enter waypoint longitude: "))

point_xy = gps.geo2enu(point_lat, point_long, 0)
point_pos = [point_xy.item(0), point_xy.item(1)] 

drone_angle = float(input("Enter drone bearing (in degrees): ")) % 360
point_angle = float(input("Enter waypoint bearing (in degrees): ")) % 360

drone_vec = [math.cos(drone_angle * PI/180), math.sin(drone_angle * PI/180)]
point_vec = [math.cos(point_angle * PI/180), math.sin(point_angle * PI/180)]

path_xy = find_path(drone_pos, point_pos, drone_vec, point_vec)
path_gps = []

for p in path_xy:
    geo = gps.enu2geo(p[0], p[1], 0)
    path_gps.append([geo.item(0), geo.item(1)])

print("\n")

for p in path_gps:
    print(f"{p[0]}, {p[1]}")