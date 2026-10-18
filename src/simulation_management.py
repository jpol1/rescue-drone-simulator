import json
import math


def read_objects(filename="../data/objects.json"):
    try:
        with open(filename, "r", encoding="utf-8") as file:
            json_data = json.load(file)
            return json_data["objects"]

    except FileNotFoundError:
        return []


def match_object(people_coordinates, object_coordinates):
    for person in people_coordinates:
        if person["type"] != "person":
            continue

        person_coordinates = (person["x"], person["y"])

        if math.dist(person_coordinates, object_coordinates) < 1.5:
            return person
    return None


def analyze_detected_points(objects_coordinates, people_coordinates):
    confirmed_people_coordinates = []
    for point in objects_coordinates:
        person = match_object(people_coordinates, point)

        if person is not None and person not in confirmed_people_coordinates:
            confirmed_people_coordinates.append(person)

    return confirmed_people_coordinates


def generate_report(confirmed_coordinates):
    confirmed_coordinates_count = len(confirmed_coordinates)
    if confirmed_coordinates_count > 0:
        print(f"Drone found {len(confirmed_coordinates)}")
        for coordinates_n in range(confirmed_coordinates_count):
            print(
                f"{coordinates_n + 1}/{confirmed_coordinates_count} - "
                f"X: {confirmed_coordinates[coordinates_n]['x']:.2f} "
                f"Y: {confirmed_coordinates[coordinates_n]['y']:.2f}"
            )
    else:
        print(f"Drone has not found any people!")
