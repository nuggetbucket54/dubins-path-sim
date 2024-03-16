from matplotlib.patches import Arc

def test():
    print("HI")

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
