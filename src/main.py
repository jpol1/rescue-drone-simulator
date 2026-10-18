from pymavlink import mavutil

from drone_controller import (
    set_mode,
    arm_drone,
    takeoff,
    wait_for_position_estimate,
    generate_search_route,
    fly_route,
    get_drone_position,
)

from detection_management import read_saved_points
from simulation_management import read_objects, analyze_detected_points, generate_report

if __name__ == "__main__":
    connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

    print("Waiting for heartbeat...")
    heartbeat = connection.wait_heartbeat(timeout=45)

    if heartbeat is None:
        raise TimeoutError("No heartbeat received")

    print("Connected to ArduPilot!")

    set_mode(connection, "GUIDED")
    wait_for_position_estimate(connection)

    start_position = get_drone_position(connection)

    arm_drone(connection)
    takeoff(connection, 10)

    waypoints = generate_search_route(100, 300, 30)
    try:
        fly_route(connection, waypoints, start_position)

    except (Exception, KeyboardInterrupt) as error:
        print(f"FLY ERROR: {error}")
    finally:
        potential_people = read_objects()
        detected_objects = read_saved_points()
        confirmed_points = analyze_detected_points(
            detected_objects, potential_people
        )
        generate_report(confirmed_points)
