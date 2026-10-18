from simulation_management import read_objects, match_object, analyze_detected_points
import json


def test_read_objects(tmp_path):
    filepath = tmp_path / "objects.json"
    objects = [
        {"type": "person", "x": 10.0, "y": 20.0},
        {"type": "unknown", "x": 70.0, "y": 80.0},
    ]

    with open(filepath, "w", encoding="utf-8") as file:
        json.dump({"objects": objects}, file)

    result = read_objects(filepath)

    assert result == objects


def test_read_objects_with_not_existing_file(tmp_path):
    filepath = tmp_path / "nonexistent.json"

    result = read_objects(filepath)

    assert result == []


def test_match_object_person_success():
    objects = [
        {"type": "person", "x": 10.0, "y": 20.0},
        {"type": "person", "x": 30.0, "y": 40.0},
    ]
    object_point = (10.2, 20.3)

    result = match_object(objects, object_point)

    assert result == objects[0]


def test_match_object_unknown_object_type():
    objects = [
        {"type": "unknown", "x": 70.0, "y": 80.0},
    ]
    object_point = (70.2, 80.3)

    result = match_object(objects, object_point)

    assert result is None


def test_match_object_person_too_far():
    objects = [
        {"type": "person", "x": 30.0, "y": 40.0},
    ]
    object_point = (31.0, 42.0)

    result = match_object(objects, object_point)

    assert result is None


def test_analyze_detected_points():
    objects = [
        {"type": "person", "x": 10.0, "y": 20.0},
        {"type": "person", "x": 30.0, "y": 40.0},
        {"type": "unknown", "x": 70.0, "y": 80.0},
    ]
    detected = [
        (10.2, 20.3),
        (10.5, 20.4),
        (30.1, 40.2),
        (70.0, 80.0),
        (150.0, 160.0),
    ]

    result = analyze_detected_points(detected, objects)

    assert len(result) == 2
    assert objects[0] in result
    assert objects[1] in result


def test_analyze_detected_points_duplicates():
    objects = [
        {"type": "person", "x": 10.0, "y": 20.0},
    ]
    detected = [
        (10.0, 20.0),
        (10.2, 20.3),
        (10.5, 20.4),
    ]

    result = analyze_detected_points(detected, objects)

    assert len(result) == 1
    assert result[0] == objects[0]


def test_analyze_detected_points_no_people():
    objects = [
        {"type": "person", "x": 10.0, "y": 20.0},
        {"type": "unknown", "x": 70.0, "y": 80.0},
        {"type": "unknown", "x": 90.0, "y": 100.0},
    ]
    detected = [
        (70.0, 80.0),
        (90.0, 100.0),
        (150.0, 160.0),
    ]

    result = analyze_detected_points(detected, objects)

    assert result == []


def test_analyze_detected_points_empty():
    objects = [
        {"type": "person", "x": 10.0, "y": 20.0},
    ]

    result = analyze_detected_points([], objects)

    assert result == []
