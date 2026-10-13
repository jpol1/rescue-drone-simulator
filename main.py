import time

from pymavlink import mavutil
from drone_controller import set_mode, arm_drone, takeoff, wait_for_position_estimate, recurring_message_request

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

    wait_for_position_estimate(connection)
    # recurring_message_request(connection, command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, param1=mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, param2=1000000)
    # message = connection.recv_match(
    #     type="EXTENDED_SYS_STATE",
    #     blocking=True,
    #     timeout=5
    # )
    # if message is not None:
    #     print(message.to_dict())


    arm_drone(connection)
    takeoff(connection, 10)
    time.sleep(4)
    set_mode(connection, "LAND")

