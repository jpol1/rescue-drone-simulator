import pytest
from unittest.mock import MagicMock

from sensor_management import (
    calculate_object_position,
    process_sensor_data,
    get_lidar_readings,
)


@pytest.mark.parametrize(
    "drone_x, drone_y, distance, heading, orientation, expected",
    [
        (0, 0, 10, 0, 0, (10, 0)),
        (0, 0, 10, 9000, 0, (0, 10)),
        (0, 0, 10, 0, 2, (0, 10)),
        (20, 30, 10, 0, 0, (30, 30)),
    ],
)
def test_calculate_object_position(
    drone_x, drone_y, distance, heading, orientation, expected
):
    x, y = calculate_object_position(drone_x, drone_y, distance, heading, orientation)

    assert x == pytest.approx(expected[0])
    assert y == pytest.approx(expected[1])


def test_process_sensor_data_invalid_orientation():
    data = MagicMock()
    data.get_type.return_value = "DISTANCE_SENSOR"
    data.min_distance = 2
    data.max_distance = 1200
    data.current_distance = 500
    data.orientation = 9

    result = process_sensor_data(data, position=(0, 0, 0), heading=0)

    assert result is None


@pytest.mark.parametrize("distance", [1, 1300])
def test_process_sensor_data_invalid_distance(distance):
    data = MagicMock()
    data.get_type.return_value = "DISTANCE_SENSOR"
    data.min_distance = 2
    data.max_distance = 1200
    data.current_distance = distance
    data.orientation = 0

    result = process_sensor_data(data, position=(0, 0, 0), heading=0)

    assert result is None


def test_process_sensor_data_success():
    data = MagicMock()
    data.get_type.return_value = "DISTANCE_SENSOR"
    data.min_distance = 2
    data.max_distance = 1200
    data.current_distance = 500
    data.orientation = 0

    result = process_sensor_data(data, position=(10, 20, -10), heading=0)

    assert result == pytest.approx((15, 20))


@pytest.mark.parametrize(
    "position, heading",
    [
        (None, 0),
        ((10, 20, -10), None),
        (None, None),
    ],
)
def test_process_sensor_data_missing_position_or_heading(position, heading):
    data = MagicMock()
    data.get_type.return_value = "DISTANCE_SENSOR"
    data.min_distance = 2
    data.max_distance = 1200
    data.current_distance = 500
    data.orientation = 0

    result = process_sensor_data(data, position, heading)

    assert result is None


def test_process_sensor_data_wrong_message_type():
    data = MagicMock()
    data.get_type.return_value = "GLOBAL_POSITION_INT"

    result = process_sensor_data(data, (0, 0, -10), 0)

    assert result is None


def test_get_lidar_readings_success():
    connection = MagicMock()

    message = MagicMock()
    message.min_distance = 2
    message.max_distance = 1200
    message.current_distance = 500
    message.orientation = 2

    connection.recv_match.return_value = message

    result = get_lidar_readings(connection)

    assert result == {"distance": 5.0, "orientation": 2}

    connection.recv_match.assert_called_once_with(
        type="DISTANCE_SENSOR", blocking=True, timeout=5
    )


@pytest.mark.parametrize("distance", [1, 1300])
def test_get_lidar_readings_invalid_distance(distance):
    connection = MagicMock()

    message = MagicMock()
    message.min_distance = 2
    message.max_distance = 1200
    message.current_distance = distance
    message.orientation = 0

    connection.recv_match.return_value = message

    result = get_lidar_readings(connection)

    assert result is None


def test_get_lidar_readings_timeout():
    connection = MagicMock()
    connection.recv_match.return_value = None

    with pytest.raises(TimeoutError, match="No DISTANCE_SENSOR data received"):
        get_lidar_readings(connection)
