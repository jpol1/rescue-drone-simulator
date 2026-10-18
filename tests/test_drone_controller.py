import pytest
from drone_controller import (
    generate_search_route,
    check_distance_from_home,
    takeoff,
    wait_for_command_ack_answer,
    get_drone_direction,
    get_drone_position,
    get_drone_height,
    check_before_takeoff,
    arm_drone,
    move_drone,
    fly_route,
)
from unittest.mock import MagicMock, patch
from pymavlink import mavutil

from drone_controller import set_mode


def test_generate_search_route():
    route = generate_search_route(100, 300, 30)
    sum_y = sum(y for x, y, z in route)

    assert route[0] == (100, 0, 0)
    assert route[1] == (0, 30, 0)
    assert route[2] == (-100, 0, 0)
    assert sum_y == 300
    assert all(z == 0 for x, y, z in route)


def test_generate_exception_search_route():
    with pytest.raises(ValueError):
        generate_search_route(-100, 300, 30)


def test_generate_search_route_length_does_not_divide_by_spacing():
    route = generate_search_route(100, 273, 30)
    sum_y = sum(y for x, y, z in route)

    assert route[0] == (100, 0, 0)
    assert route[1] == (0, 30, 0)
    assert route[2] == (-100, 0, 0)
    assert route[-2] == (0, 3, 0)
    assert sum_y == 273
    assert all(z == 0 for x, y, z in route)


def test_generate_search_spacing_bigger_than_length():
    route = generate_search_route(100, 100, 150)
    sum_y = sum(y for x, y, z in route)

    assert route[0] == (100, 0, 0)
    assert route[1] == (0, 100, 0)
    assert sum_y == 100
    assert all(z == 0 for x, y, z in route)


def test_check_distance_from_home_bigger_than2():
    current_position = (10, 20, 0)
    start_position = (0, 0, 0)
    with pytest.raises(RuntimeError):
        check_distance_from_home(
            current_positon=current_position, start_position=start_position
        )


def test_check_distance_from_home():
    current_position = (1, 1, 0)
    start_position = (0, 0, 0)

    check_distance_from_home(
        current_positon=current_position, start_position=start_position
    )


def test_takeoff_with_altitude_below_0():
    with pytest.raises(ValueError):
        takeoff(None, -5)


def test_wait_for_command_ack_answer_accepted():
    connection = MagicMock()
    message = MagicMock()
    message.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    message.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
    connection.recv_match.return_value = message

    res = wait_for_command_ack_answer(connection, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    assert res["success"] == True
    assert res["result"] == message.result


def test_wait_for_command_ack_answer_failure():
    connection = MagicMock()
    message = MagicMock()
    message.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    message.result = mavutil.mavlink.MAV_RESULT_FAILED
    connection.recv_match.return_value = message

    res = wait_for_command_ack_answer(connection, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    assert res["success"] == False
    assert res["result"] == message.result


def test_wait_for_command_ack_wrong_command():
    connection = MagicMock()
    message = MagicMock()
    message.command = mavutil.mavlink.MAV_CMD_DO_SET_MODE
    message.result = mavutil.mavlink.MAV_RESULT_ACCEPTED
    connection.recv_match.return_value = message

    with pytest.raises(TimeoutError):
        wait_for_command_ack_answer(
            connection, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=0.1
        )


def test_get_drone_position_success():
    position = (10.1, 20.4, 9.84)
    connection = MagicMock()
    data = MagicMock()
    data.x = position[0]
    data.y = position[1]
    data.z = position[2]
    connection.recv_match.return_value = data

    res = get_drone_position(connection)
    assert res == position


def test_get_drone_position_failure():
    connection = MagicMock()
    data = None
    connection.recv_match.return_value = data

    with pytest.raises(TimeoutError):
        get_drone_position(connection)


def test_get_drone_height_success():
    altitude = 10000

    connection = MagicMock()
    drone_info = MagicMock()

    drone_info.relative_alt = altitude

    connection.recv_match.return_value = drone_info

    res = get_drone_height(connection)

    assert res == altitude / 1000


def test_get_drone_direction_success():
    heading = 30405
    connection = MagicMock()
    drone_info = MagicMock()

    drone_info.hdg = heading

    connection.recv_match.return_value = drone_info

    res = get_drone_direction(connection)

    assert res == heading


def test_get_drone_direction_unavailable():
    heading = 65535
    connection = MagicMock()
    drone_info = MagicMock()

    drone_info.hdg = heading

    connection.recv_match.return_value = drone_info

    res = get_drone_direction(connection)

    assert res == None


@pytest.mark.parametrize(
    "landed_state, expected_exception",
    [
        (mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, None),
        (mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, RuntimeError),
        (mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF, RuntimeError),
        (mavutil.mavlink.MAV_LANDED_STATE_LANDING, RuntimeError),
        (mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED, RuntimeError),
    ],
)
def test_check_before_takeoff(landed_state, expected_exception):
    connection = MagicMock()
    connection.recv_match.return_value.landed_state = landed_state

    if expected_exception is None:
        check_before_takeoff(connection)
    else:
        with pytest.raises(expected_exception):
            check_before_takeoff(connection)


def test_check_before_takeoff_message_is_None():
    connection = MagicMock()
    connection.recv_match.return_value = None

    with pytest.raises(TimeoutError):
        check_before_takeoff(connection)


def test_set_mode_success():
    connection = MagicMock()
    connection.mode_mapping.return_value = {"MODE": 123}

    with (
        patch("drone_controller.wait_for_command_ack_answer") as mock_ack,
        patch("drone_controller.wait_for_heartbeat_answer") as mock_heartbeat,
    ):
        mock_ack.return_value = {
            "success": True,
            "result": mavutil.mavlink.MAV_RESULT_ACCEPTED,
        }

        set_mode(connection, "MODE")

        connection.mav.command_long_send.assert_called_once_with(
            target_system=connection.target_system,
            target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=123,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )

        mock_ack.assert_called_once_with(
            connection=connection,
            expected_communicate=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        )

        mock_heartbeat.assert_called_once()


def test_set_mode_failure():
    connection = MagicMock()
    connection.mode_mapping.return_value = {"MODE": 123}

    with (
        patch("drone_controller.wait_for_command_ack_answer") as mock_ack,
    ):
        mock_ack.return_value = {
            "success": False,
            "result": mavutil.mavlink.MAV_RESULT_FAILED,
        }
        with pytest.raises(RuntimeError):
            set_mode(connection, "MODE")


def test_set_mode_unknown_mode():
    connection = MagicMock()
    connection.mode_mapping.return_value = {"MODE": 123}

    with pytest.raises(ValueError):
        set_mode(connection, "UNKNOWN_MODE")


def test_set_mode_none_mode_mapping():
    connection = MagicMock()
    connection.mode_mapping.return_value = None

    with pytest.raises(ValueError):
        set_mode(connection, "UNKNOWN_MODE")


def test_arm_drone_success():
    connection = MagicMock()

    with (
        patch("drone_controller.wait_for_command_ack_answer") as mock_ack,
        patch("drone_controller.wait_for_heartbeat_answer") as mock_heartbeat,
    ):
        mock_ack.return_value = {
            "success": True,
            "result": mavutil.mavlink.MAV_RESULT_ACCEPTED,
        }

        arm_drone(connection)

        connection.mav.command_long_send.assert_called_once_with(
            target_system=connection.target_system,
            target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )

        mock_ack.assert_called_once_with(
            connection=connection,
            expected_communicate=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        )

        mock_heartbeat.assert_called_once()


def test_arm_drone_failure():
    connection = MagicMock()

    with (
        patch("drone_controller.wait_for_command_ack_answer") as mock_ack,
    ):
        mock_ack.return_value = {
            "success": False,
            "result": mavutil.mavlink.MAV_RESULT_FAILED,
        }

        with pytest.raises(RuntimeError):
            arm_drone(connection)


def test_takeoff_success():
    connection = MagicMock()
    altitude = 10

    with (
        patch("drone_controller.wait_for_command_ack_answer") as mock_ack,
        patch("drone_controller.drone_height_required") as mock_height_required,
        patch("drone_controller.check_before_takeoff"),
    ):
        mock_ack.return_value = {
            "success": True,
            "result": mavutil.mavlink.MAV_RESULT_ACCEPTED,
        }

        takeoff(connection, altitude)

        connection.mav.command_long_send.assert_called_once_with(
            target_system=connection.target_system,
            target_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=altitude,
        )

        mock_ack.assert_called_once_with(
            connection=connection,
            expected_communicate=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        )

        mock_height_required.assert_called_once_with(
            connection=connection, required_altitude=altitude
        )


def test_takeoff_failure():
    connection = MagicMock()
    altitude = 10

    with (
        patch("drone_controller.wait_for_command_ack_answer") as mock_ack,
        patch("drone_controller.check_before_takeoff"),
    ):
        mock_ack.return_value = {
            "success": False,
            "result": mavutil.mavlink.MAV_RESULT_FAILED,
        }

        with pytest.raises(RuntimeError):
            takeoff(connection, altitude)


def test_move_drone_success():
    connection = MagicMock()
    saved_points = []

    position_message = MagicMock()
    position_message.get_type.return_value = "LOCAL_POSITION_NED"
    position_message.x = 10
    position_message.y = 20
    position_message.z = -10

    with patch("drone_controller.get_drone_position") as mock_position:
        mock_position.return_value = (0, 0, -10)
        connection.recv_match.return_value = position_message

        result = move_drone(connection, 10, 20, 0, saved_points)

        assert result is True

        connection.mav.set_position_target_local_ned_send.assert_called_once_with(
            0,
            connection.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            3576,
            10,
            20,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )


def test_move_drone_detect_object():
    connection = MagicMock()
    saved_points = []

    heading_message = MagicMock()
    heading_message.get_type.return_value = "GLOBAL_POSITION_INT"
    heading_message.hdg = 0

    sensor_message = MagicMock()
    sensor_message.get_type.return_value = "DISTANCE_SENSOR"

    target_message = MagicMock()
    target_message.get_type.return_value = "LOCAL_POSITION_NED"
    target_message.x = 10
    target_message.y = 0
    target_message.z = -10

    connection.recv_match.side_effect = [
        heading_message,
        sensor_message,
        target_message,
    ]

    with (
        patch("drone_controller.get_drone_position", return_value=(0, 0, -10)),
        patch(
            "drone_controller.process_sensor_data", return_value=(5, 0)
        ) as mock_sensor,
        patch("drone_controller.is_new_point") as mock_new_point,
    ):
        result = move_drone(connection, 10, 0, 0, saved_points)

        assert result is True

        mock_sensor.assert_called_once_with(sensor_message, (0, 0, -10), 0)

        mock_new_point.assert_called_once_with((5, 0), saved_points)


def test_move_drone_timeout():
    connection = MagicMock()
    connection.recv_match.return_value = None

    with (
        patch("drone_controller.get_drone_position", return_value=(0, 0, -10)),
        patch("drone_controller.time.monotonic") as mock_time,
    ):
        mock_time.side_effect = [0, 21]

        with pytest.raises(TimeoutError):
            move_drone(connection, 10, 0, 0, [])


def test_fly_route_success():
    connection = MagicMock()
    waypoints = [(10, 0, 0), (0, 20, 0), (-10, 0, 0)]
    start_position = (0, 0, 0)

    with (
        patch("drone_controller.move_drone", return_value=True) as mock_move,
        patch("drone_controller.land_drone") as mock_land,
        patch("drone_controller.save_points_to_json") as mock_save,
        patch("drone_controller.time.sleep"),
    ):
        fly_route(connection, waypoints, start_position)

        assert mock_move.call_count == 3

        saved_points = []

        mock_move.assert_any_call(connection, 10, 0, 0, saved_points)
        mock_move.assert_any_call(connection, 0, 20, 0, saved_points)
        mock_move.assert_any_call(connection, -10, 0, 0, saved_points)

        assert mock_save.call_count == 4

        mock_land.assert_called_once_with(
            connection, back_to_home=True, start_position=start_position
        )


def test_fly_route_error():
    connection = MagicMock()
    waypoints = [(10, 0, 0)]

    with (
        patch("drone_controller.move_drone") as mock_move,
        patch("drone_controller.land_drone") as mock_land,
        patch("drone_controller.save_points_to_json") as mock_save,
        patch("drone_controller.time.sleep"),
    ):
        mock_move.side_effect = TimeoutError("Drone stuck")

        with pytest.raises(TimeoutError):
            fly_route(connection, waypoints)

        mock_save.assert_called_once_with([])

        mock_land.assert_called_once_with(
            connection, back_to_home=True, start_position=None
        )


def test_fly_route_save_error():
    connection = MagicMock()
    waypoints = [(10, 0, 0)]

    with (
        patch("drone_controller.move_drone", return_value=True),
        patch("drone_controller.land_drone") as mock_land,
        patch("drone_controller.save_points_to_json") as mock_save,
        patch("drone_controller.time.sleep"),
    ):
        mock_save.side_effect = OSError("Cannot save file")

        with pytest.raises(OSError):
            fly_route(connection, waypoints)

        assert mock_save.call_count == 2

        mock_land.assert_called_once()


def test_get_drone_height_timeout_failure():
    connection = MagicMock()
    connection.recv_match.return_value = None

    with pytest.raises(TimeoutError):
        get_drone_height(connection)


def test_get_drone_direction_timeout_failure():
    connection = MagicMock()
    connection.recv_match.return_value = None

    with pytest.raises(TimeoutError):
        get_drone_direction(connection)
