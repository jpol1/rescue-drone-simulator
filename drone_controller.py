from pymavlink import mavutil
import time


def wait_for_position_estimate(connection):
    print("Waiting for position to be estimated...")
    time_start = time.monotonic()
    while True:
        time_elapsed = time.monotonic() - time_start
        if time_elapsed >= 60:
            raise TimeoutError(
                "Position has not been estimated. Drone cannot be armed."
            )
        message = connection.recv_match(
            type="EKF_STATUS_REPORT", blocking=True, timeout=min(60 - time_elapsed, 2)
        )

        if message is None:
            continue

        if message.flags & mavutil.mavlink.EKF_POS_HORIZ_ABS:
            print("Position is estimated. Drone is ready for arming.")
            break


def recurring_message_request(connection, **params):
    default_params = {
        "target_system": connection.target_system,
        "target_component": mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
        "command": mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
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
            return {"success": True, "result": message.result}
        else:
            return {"success": False, "result": message.result}


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


def get_drone_height(connection):
    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time

        if elapsed_time >= 5:
            raise TimeoutError("Cannot take the drone height!")
        drone_info = connection.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True, timeout=min(5 - elapsed_time, 1)
        )
        if drone_info is None:
            continue

        return drone_info.relative_alt / 1000


def drone_height_required(connection, required_altitude):
    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= 30:
            raise TimeoutError("Drone hasn't reached required altitude")

        drone_altitude = get_drone_height(connection)

        if drone_altitude >= (required_altitude - 0.5):
            print(f"Drone altitude: {drone_altitude}")
            print(f"Drone has reached required altitude!")
            break
        print(f"Drone altitude: {drone_altitude}")


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

    ack = wait_for_command_ack_answer(
        connection=connection, expected_communicate=mavutil.mavlink.MAV_CMD_DO_SET_MODE
    )
    if ack["success"]:
        print(f"Requested flight mode: {mode}")
    else:
        raise RuntimeError(f"Mode change rejected. MAV_RESULT: {ack['result']}")

    wait_for_heartbeat_answer(
        connection=connection, condition=lambda msg: msg.custom_mode == mode_id
    )
    print(f"Actual flight mode: {mode}")


def arm_drone(connection):
    connection.mav.command_long_send(
        target_system=connection.target_system,
        target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
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

    ack = wait_for_command_ack_answer(
        connection=connection,
        expected_communicate=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    )
    if ack["success"]:
        print(f"Arming command accepted")
    else:
        raise RuntimeError(f"Drone has not been armed.MAV_RESULT: {ack['result']}")


    wait_for_heartbeat_answer(
        connection=connection,
        condition=lambda msg: (
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ),
    )
    print(f"Drone has been ARMED")



def takeoff(connection, altitude):
    if altitude <= 0:
        raise ValueError("Altitude must be above 0 meters")

    connection.mav.command_long_send(
        target_system=connection.target_system,
        target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
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


    ack = wait_for_command_ack_answer(
        connection=connection,
        expected_communicate=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    )
    if ack["success"]:
        print("Takeoff command accepted")
        drone_height_required(connection, altitude)
    else:
        raise RuntimeError(f"Drone has not takeoff. MAV_RESULT: {ack['result']}")



def land_drone(connection):
    set_mode(connection, "LAND")
    recurring_message_request(
        connection,
        command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        param1=mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
        param2=1000000,
    )

    start_time = time.monotonic()

    while True:
        elapsed_time = time.monotonic() - start_time

        if elapsed_time >= 60:
            raise TimeoutError("Drone hasn't landed! ERROR")
        message = connection.recv_match(
            type="EXTENDED_SYS_STATE", blocking=True, timeout=min(60 - elapsed_time, 2)
        )
        if message is None:
            continue

        if message.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("Drone has landed!")
            break
        else:
            try:
                drone_altitude = get_drone_height(connection)
                print(f"Drone altitude: {drone_altitude:.2f} m")
            except TimeoutError:
                print("Altitude data temporarily unavailable")

    wait_for_heartbeat_answer(
        connection,
        condition=lambda msg: (
            not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        ),
    )
    print("Drone has been disarmed!")
