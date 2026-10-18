from detection_management import save_points_to_json, read_saved_points, is_new_point
import pytest


@pytest.fixture
def sample_points():
    return [
        (10.0, 20.0),
        (15.5, 25.5),
        (30.0, 40.0),
        (50.2, 60.8),
    ]


def test_save_and_read_points_to_json(tmp_path, sample_points):
    filename = tmp_path / "points.json"

    save_points_to_json(sample_points, filename)
    new_points = read_saved_points(filename)

    assert sample_points == new_points


def test_read_saved_points_with_not_existing_file():
    points = read_saved_points("path/does/not/exist")
    assert points == []


def test_is_new_point_success(sample_points):
    len_sample_points = len(sample_points)
    new_point = (60.1, 30.9)
    is_new_point(new_point, sample_points)
    assert len(sample_points) == len_sample_points + 1
    assert sample_points[-1] == new_point


def test_is_new_point_failure(sample_points):
    len_sample_points = len(sample_points)
    new_point = (10.5, 20.4)
    is_new_point(new_point, sample_points)
    assert len(sample_points) == len_sample_points
