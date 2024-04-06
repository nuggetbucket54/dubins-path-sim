from fastapi import FastAPI
from pydantic import BaseModel
from dubins import dubin

app = FastAPI()

class Waypoint(BaseModel):
    id: int
    name: str
    lat: float
    long: float
    alt: float


class Item(BaseModel):
    current_waypoint: Waypoint
    desired_waypoint: Waypoint
    current_heading: float
    desired_heading: float


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

    points = dubin(drone_lat, drone_long, drone_angle, point_lat, point_long, point_angle)

    for p in points:
        waypoints.append(create_item(p[0], p[1], alt))

    print('\n'*10)
    for w in waypoints:
        print(str(w["lat"])+","+str(w["long"]))
    print('\n'*10)

    return {"waypoints": waypoints}