from pymavlink import mavutil
import time
def set_mode(connection, mode):
    mode_mapping = connection.mode_mapping()

    if mode_mapping is None or mode not in mode_mapping:
        raise ValueError(f"Unknown flight mode: {mode}")

    mode_id = mode_mapping[mode]

    connection.mav.command_long_send(
        connection.target_system,
        1,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
        0, 0, 0, 0, 0
    )

    start_time = time.monotonic()
    while True:
        elapsed_time = time.monotonic() - start_time
        if (elapsed_time >= 10) :
            raise TimeoutError("No suitable command data received")

        message = connection.recv_match(
            type="COMMAND_ACK",
            blocking=True,
            timeout=min(10 - elapsed_time, 2)
        )



        if message is None or message.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
        if (message.result == mavutil.mavlink.MAV_RESULT_ACCEPTED):
            print(f"Requested flight mode: {mode}")
            break
        else:
            raise RuntimeError(f"Mode change rejected. MAV_RESULT: {message.result}")

    while True:
        elapsed_time = time.monotonic() - start_time
        if (elapsed_time >= 10) :
            raise TimeoutError(f"Flight mode {mode} was not confirmed")

        heartbeat_message = connection.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=min(10 - elapsed_time, 2)
        )

        if heartbeat_message is None:
            continue
        if(heartbeat_message.custom_mode == mode_id):
            print(f"Actual flight mode: {mode}")
            break
