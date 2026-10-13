from pymavlink import mavutil
import time


def set_mode(connection, mode):
    mode_mapping = connection.mode_mapping()

    if mode_mapping is None or mode not in mode_mapping:
        raise ValueError(f"Unknown flight mode: {mode}")

    mode_id = mode_mapping[mode]

    connection.mav.command_long_send(
        target_system=connection.target_system,
        target_component=11,
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

    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= 10:
            raise TimeoutError("No suitable command data received")

        message = connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=min(10 - elapsed_time, 2)
        )

        if message is None or message.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
        if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Requested flight mode: {mode}")
            break
        else:
            raise RuntimeError(f"Mode change rejected. MAV_RESULT: {message.result}")

    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= 10:
            raise TimeoutError(f"Flight mode {mode} was not confirmed")

        heartbeat_message = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(10 - elapsed_time, 2)
        )

        if heartbeat_message is None:
            continue
        if heartbeat_message.custom_mode == mode_id:
            print(f"Actual flight mode: {mode}")
            break


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

    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= 10:
            raise TimeoutError("No suitable command data received")
        message = connection.recv_match(
            type="COMMAND_ACK", blocking=True, timeout=min(10 - elapsed_time, 2)
        )

        if message is None or message.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            continue

        if message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Arming command accepted")
            break
        else:
            raise RuntimeError(f"Drone has not been armed. MAV_RESULT: {message.result}")

    while True:
        elapsed_time = time.monotonic() - start_time
        if elapsed_time >= 10:
            raise TimeoutError("Drone has not been armed")

        heartbeat_message = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=min(10 - elapsed_time, 2)
        )

        if heartbeat_message is None:
            continue

        if heartbeat_message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print(f"Drone has been ARMED")
            break
        else:
            continue

