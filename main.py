from pymavlink import mavutil
from drone_controller import set_mode, arm_drone

if __name__ == "__main__":
    connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    print("Waiting for heartbeat...")
    heartbeat = connection.wait_heartbeat(timeout=15)

    if heartbeat is None:
        raise TimeoutError("No heartbeat received")

    print("Connected to ArduPilot!")
    print("System ID:", connection.target_system)
    print("Component ID:", connection.target_component)

    print("Waiting for position data...")

    message = connection.recv_match(
        type="GLOBAL_POSITION_INT", blocking=True, timeout=10
    )

    if message is None:
        raise TimeoutError("No position data received")

    print(message.to_dict())
    latitude = message.lat / 1e7
    longitude = message.lon / 1e7
    altitude = message.relative_alt / 1000

    print(f"Latitude: {latitude:.7f}")
    print(f"Longitude: {longitude:.7f}")
    print(f"Relative altitude: {altitude:.2f} m")

    set_mode(connection, "GUIDED")
    arm_drone(connection)
