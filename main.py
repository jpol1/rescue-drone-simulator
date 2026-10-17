import time

from pymavlink import mavutil

from drone_controller import (
    set_mode,
    arm_drone,
    takeoff,
    wait_for_position_estimate,
    land_drone,
    generate_search_route,
    fly_route
)


if __name__ == "__main__":
    connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

    print("Waiting for heartbeat...")
    heartbeat = connection.wait_heartbeat(timeout=30)

    if heartbeat is None:
        raise TimeoutError("No heartbeat received")

    print("Connected to ArduPilot!")

    set_mode(connection, "GUIDED")
    wait_for_position_estimate(connection)
    arm_drone(connection)
    takeoff(connection, 10)

    waypoints = generate_search_route(100, 300, 30)
    fly_route(connection, waypoints)

    time.sleep(1)
    land_drone(connection)