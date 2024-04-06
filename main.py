from fastapi import FastAPI
from pydantic import BaseModel
from dubins import dubin

app = FastAPI()

# waypoint object
class Waypoint(BaseModel):
    id: int
    name: str
    lat: float
    long: float
    alt: float

# object received from GCOM
class Item(BaseModel):
    current_waypoint: Waypoint
    desired_waypoint: Waypoint
    current_heading: float
    desired_heading: float

# converts calculated latitude/longitude into JSON format to match object
def create_item(lat, long, alt):
    return {
        "id": -1,
        "name": "",
        "lat": lat,
        "long": long,
        "alt": alt
    }

@app.post("/")
async def root(item: Item):
    # get drone and waypoint positions
    drone = item.current_waypoint
    destination = item.desired_waypoint
    alt = drone.alt
    waypoints = []

    drone_lat = drone.lat
    drone_long = drone. long
    drone_angle = item.current_heading

    point_lat = destination.lat
    point_long = destination.long
    point_angle = item.desired_heading

    # grab points of Dubins path
    points = dubin(drone_lat, drone_long, drone_angle, point_lat, point_long, point_angle)

    # turn all points into JSON objects
    for p in points:
        waypoints.append(create_item(p[0], p[1], alt))

    return {"waypoints": waypoints}