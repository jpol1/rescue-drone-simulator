from pymavlink import mavutil
import time

def wait_for_position_estimate(connection):
    print("Waiting for position to be estimated...")
    time_start = time.monotonic()
    while True:
        time_elapsed = time.monotonic() - time_start
        if time_elapsed >= 60:
            raise TimeoutError("Position has not been estimated. Drone cannot be armed.")
        message = connection.recv_match(
            type="EKF_STATUS_REPORT",
            blocking=True,
            timeout=min(60 - time_elapsed, 2)
        )


        if message is None:
            continue

        if(message.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS):
            print("Position is estimated. Drone is ready for arming.")
            break
        else:
            continue

def recurring_message_request(connection, **params):
    default_params={
        "target_system": connection.target_system,
        "target_component": mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        "command": mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        "confirmation": 0,
        "param1": 0,
        "param2": 0,
        "param3": 0,
        "param4": 0,
        "param5": 0,
        "param6": 0,
        "param7": 0,
    }
    for key, value in params.items():
        if key in default_params:
            default_params[key] = value

    connection.mav.command_long_send(**default_params)

def wait_for_command_ack_answer(connection, expected_communicate, timeout=10):
    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= timeout:
            raise TimeoutError

        message = connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=min(timeout - elapsed_time, 2)
        )

        if message is None or message.command != expected_communicate:
            continue
        if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return {
                "success":True,
                "result": message.result
            }
        else:
            return {
                "success": False,
                "result": message.result
            }


def wait_for_heartbeat_answer(connection, condition, timeout=10):
    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= timeout:
            raise TimeoutError

        heartbeat_message = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(timeout - elapsed_time, 2)
        )

        if heartbeat_message is None:
            continue
        else:
            if condition(heartbeat_message):
                return

def set_mode(connection, mode):
    mode_mapping = connection.mode_mapping()

    if mode_mapping is None or mode not in mode_mapping:
        raise ValueError(f"Unknown flight mode: {mode}")

    mode_id = mode_mapping[mode]

    connection.mav.command_long_send(
        target_system=connection.target_system,
        target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        confirmation=0,
        param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        param2=mode_id,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )


    try:
        ack_success, result = wait_for_command_ack_answer(connection=connection, expected_communicate=mavutil.mavlink.MAV_CMD_DO_SET_MODE)
        if ack_success:
            print(f"Requested flight mode: {mode}")
        else:
            raise RuntimeError(f"Mode change rejected. MAV_RESULT: {result}")
    except TimeoutError:
        print(f"Flight mode: {mode} was not confirmed")

    try:
        wait_for_heartbeat_answer(connection=connection, condition=lambda msg: msg.custom_mode == mode_id)
        print(f"Actual flight mode: {mode}")
    except TimeoutError:
        print(f"Flight mode {mode} was not confirmed")



def arm_drone(connection):
    connection.mav.command_long_send(
        target_system=connection.target_system,
        target_component=1,
        confirmation=0,
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        param1=1,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )
    try:
        ack_success, result = wait_for_command_ack_answer(connection=connection,
                                                          expected_communicate=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack_success:
            print(f"Arming command accepted")
        else:
            raise RuntimeError(f"Drone has not been armed.MAV_RESULT: {result}")
    except TimeoutError:
        print(f"No suitable command data received")

    try:
        wait_for_heartbeat_answer(connection=connection, condition=lambda msg: msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"Drone has been ARMED")
    except TimeoutError:
        print(f"Drone has not been armed")

def takeoff(connection, altitude):
    if altitude <= 0:
        raise ValueError("Altitude must be above 0 meters")

    connection.mav.command_long_send(
        target_system=connection.target_system,
        target_component=1,
        confirmation=0,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=altitude,
    )

    try:
        ack_success, result = wait_for_command_ack_answer(connection=connection,
                                                          expected_communicate=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if ack_success:
            print(f"Arming command accepted")
        else:
            raise RuntimeError(f"Drone has not takeoff. MAV_RESULT: {result}")
    except TimeoutError:
        print(f"Drone has not takeoff")


    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if (elapsed_time >= 30):
            raise TimeoutError("Drone hasn't reached required altitude")

        drone_info = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=min(30 - elapsed_time,2))
        if drone_info is None:
            continue

        drone_altitude = drone_info.relative_alt / 1000
        if drone_altitude >= (altitude-0.5):
            print(f"Drone altitude: {drone_altitude}")
            print(f"Drone has reached required altitude!")
            break
        else:
            print(f"Drone altitude: {drone_altitude}")