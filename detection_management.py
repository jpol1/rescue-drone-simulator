import math
import json

def save_points_to_json(points, filename="data/detected_points.json"):
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(points, f, ensure_ascii=False, indent=4)

def read_saved_points(filename="data/detected_points.json"):
    try:
        with open(filename, "r", encoding="utf-8") as file:
            json_data = json.load(file)
            return [tuple(data) for data in json_data]
    except FileNotFoundError:
        return []

def is_new_point(point, saved_points, min_distance=1.0):
    for saved_point in saved_points:
        if math.dist(point, saved_point) < min_distance:
            return saved_points
    saved_points.append(point)