#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This program can help you to find suitable spawn points for your
scenarios. It provides a visualisation of each one in turn, with a
delay in-between, and outputs a list.

If you have a rough idea of where you want a spawn point, for example
by manually driving there, you can provide a location and radius to
limit the search.
"""

import argparse
import math
import random
import time

import carla

DEF_START_INDEX = 0
DEF_INTERVAL = 2.0
DEF_DISTANCE = 12.0
DEF_YAW_ANGLE = 15.0
DEF_PITCH_ANGLE = 30.0
DEF_RADIUS = 30.0


def get_view_transform(args, base_transform):
    d = args.distance
    br = base_transform.rotation
    yaw = math.radians(-(args.yaw + br.yaw))
    pitch = math.radians(args.pitch + br.pitch)
    location = (
        carla.Location(-d * math.cos(yaw), d * math.sin(yaw), d * math.sin(pitch))
        + base_transform.location
    )
    rotation = carla.Rotation(-math.degrees(pitch), -math.degrees(yaw))
    return carla.Transform(location, rotation)


def main():
    argparser = argparse.ArgumentParser(description="CARLA Spawn Point Display")
    argparser.add_argument(
        "-s",
        "--start",
        metavar="S",
        default=DEF_START_INDEX,
        type=int,
        help=f"starting index of spawn points (default: {DEF_START_INDEX})",
    )
    argparser.add_argument(
        "-i",
        "--interval",
        metavar="I",
        default=DEF_INTERVAL,
        type=float,
        help=f"interval between spawn points in seconds (default: {DEF_INTERVAL})",
    )
    argparser.add_argument(
        "-d",
        "--distance",
        metavar="D",
        default=DEF_DISTANCE,
        type=float,
        help=f"distance to view from (default: {DEF_DISTANCE})",
    )
    argparser.add_argument(
        "-y",
        "--yaw",
        metavar="Y",
        default=DEF_YAW_ANGLE,
        type=float,
        help=f"yaw angle to view from (default: {DEF_YAW_ANGLE})",
    )
    argparser.add_argument(
        "-p",
        "--pitch",
        metavar="P",
        default=DEF_PITCH_ANGLE,
        type=float,
        help=f"pitch angle to view from (default: {DEF_PITCH_ANGLE})",
    )
    argparser.add_argument(
        "-l",
        "--location",
        metavar="X,Y",
        default="",
        help="optional location to find spawn points around (default: not set)",
    )
    argparser.add_argument(
        "-r",
        "--radius",
        metavar="R",
        default=DEF_RADIUS,
        type=float,
        help=f"radius around search location to find spawn points (default: {DEF_RADIUS})",
    )
    args = argparser.parse_args()

    search_x = None
    search_y = None
    if args.location != "":
        search_x, search_y = [float(v) for v in args.location.split(",")]

    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    spectator = world.get_spectator()
    vehicle_blueprints = world.get_blueprint_library().filter("vehicle")
    blueprint = random.choice(vehicle_blueprints)
    spawn_points = world.get_map().get_spawn_points()

    count = len(spawn_points)
    for i in range(args.start, count):
        spawn_point = spawn_points[i]
        if search_x is not None and search_y is not None:
            dx = spawn_point.location.x - search_x
            dy = spawn_point.location.y - search_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > args.radius:
                continue

        print(f"Spawn Point {i} ({i+1} of {count}): {spawn_point}")
        spectator.set_transform(get_view_transform(args, spawn_point))
        vehicle = world.spawn_actor(blueprint, spawn_point)
        try:
            time.sleep(args.interval)
        finally:
            vehicle.destroy()


if __name__ == "__main__":
    main()
