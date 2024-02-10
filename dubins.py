import random

width=100
height=80

def genCoords():
    """generates two random coordinates representing drone position and waypoint"""
    return [[random.randint(1,width-1),random.randint(1,height-1)],
            [random.randint(1,width-1),random.randint(1,height-1)]]