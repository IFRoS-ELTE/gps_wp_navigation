#!/usr/bin/env python3

import numpy as np
import json
import rospkg

rospack = rospkg.RosPack()
package_path = rospack.get_path("scout_robot__2dnav")


def get_waypoints(file_name):
    write_path = package_path + "/waypoints/lat_lon.txt"
    with open(file_name, "r") as file:
        geo_json = json.load(file)
    waypoints = []

    for feature in geo_json["features"]:
        if feature["geometry"]["type"] == "Point":
            waypoints.append(feature["geometry"]["coordinates"])
    # swap latitude and longitude to match the UTM coordinate system
    waypoints = [[point[1], point[0]] for point in waypoints]

    with open(write_path, "w") as file:
        for point in waypoints:
            file.write(str(point[0]) + " " + str(point[1]) + "\n")

    return np.array(waypoints)


if __name__ == "__main__":

    filename = package_path + "/waypoints/waypoints.geojson"
    waypoints = get_waypoints(filename)
