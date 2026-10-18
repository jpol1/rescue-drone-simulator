import math
import json
import time


def calculate_object_position(drone_x, drone_y, distance, heading, orientation):
    angle = math.radians(heading / 100 + orientation * 45)

    object_x = drone_x + distance * math.cos(angle)
    object_y = drone_y + distance * math.sin(angle)

    return object_x, object_y


def get_lidar_readings(connection):
    message = connection.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=5)

    if message is None:
        raise TimeoutError("No DISTANCE_SENSOR data received")

    distance = message.current_distance

    if not message.min_distance <= distance <= message.max_distance:
        return None

    return {"distance": distance / 100, "orientation": message.orientation}


def process_sensor_data(data, position, heading):
    if data.get_type() != "DISTANCE_SENSOR":
        return None

    if position is None or heading is None:
        return None

    if not (
        data.min_distance <= data.current_distance <= data.max_distance
        and 0 <= data.orientation <= 7
    ):
        return None

    return calculate_object_position(
        position[0], position[1], data.current_distance / 100, heading, data.orientation
    )
