#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This program takes a JSON perception input file and produces a modified
version introducing perturbations to the data. This is to simulate the
imprecise nature of perception algorithms. The input is normally
ground-truth. 
"""

import argparse
import json
import math
import os
import random

from enum import Enum


DEF_MAX_ANGLE = 8.0
DEF_MAX_XY_SHIFT = 0.4
DEF_MAX_SIZE_CHANGE = 0.1
DEF_NOMINAL_DISTANCE = 20.0
DEF_PERCEPTION_FAILURE_PROBABILITY = 0.05

MAX_SCALING_FACTOR = 2.0


class ObjectEnum(Enum):
    unknown = 0
    bicycle = 1
    motorbike = 2
    car = 3
    emergency = 4
    truck = 5
    campervan = 6
    pedestrian = 7


# Each entry is a list of objects that object could be classified as
# with their probabilities. When they don't add up to 1.0, the remaining
# probability is assigned to the "Unknown" object class.
CLASSIFICATION_MATRIX = [
    # 0: unknown
    [(1.00, ObjectEnum.unknown)],
    # 1: bicycle
    [
        (0.75, ObjectEnum.bicycle),
        (0.10, ObjectEnum.motorbike),
        (0.08, ObjectEnum.pedestrian),
    ],
    # 2: motorbike
    [(0.80, ObjectEnum.motorbike), (0.10, ObjectEnum.bicycle), (0.05, ObjectEnum.car)],
    # 3 : car
    [
        (0.75, ObjectEnum.car),
        (0.10, ObjectEnum.emergency),
        (0.05, ObjectEnum.motorbike),
        (0.04, ObjectEnum.campervan),
        (0.01, ObjectEnum.truck),
    ],
    # 4 : emergency
    [
        (0.50, ObjectEnum.emergency),
        (0.25, ObjectEnum.car),
        (0.10, ObjectEnum.truck),
        (0.03, ObjectEnum.motorbike),
    ],
    # 5 : truck
    [
        (0.80, ObjectEnum.truck),
        (0.10, ObjectEnum.campervan),
        (0.05, ObjectEnum.car),
        (0.02, ObjectEnum.emergency),
    ],
    # 6 : campervan
    [
        (0.70, ObjectEnum.campervan),
        (0.20, ObjectEnum.car),
        (0.05, ObjectEnum.emergency),
    ],
    # 7: pedestrian
    [(0.85, ObjectEnum.pedestrian), (0.05, ObjectEnum.bicycle)],
]


def get_object_index(type_name):
    for obj_type in ObjectEnum:
        if obj_type.name == type_name:
            return obj_type.value

    # Return 'unknown' type if there's no match
    return 0


def misclassify_object(obj, scale):
    original_type = obj.get("type")
    if original_type is None:
        return

    type_index = get_object_index(original_type)
    if type_index >= len(CLASSIFICATION_MATRIX):
        return

    types = CLASSIFICATION_MATRIX[type_index]
    new_type_enum = ObjectEnum.unknown
    p_type = random.random() * scale
    p_match = 0.0
    for t in types:
        p_match += t[0]
        if p_type <= p_match:
            new_type_enum = t[1]
            break

    new_type = new_type_enum.name
    obj["type"] = new_type


def process_file(args, input_pathname, output_pathname):
    input_filename = os.path.basename(input_pathname)
    output_filename = os.path.basename(output_pathname)
    output_dir = os.path.dirname(output_pathname)
    base_name = os.path.splitext(input_filename)[0]
    if output_filename == "":
        output_filename = base_name + ".json"
        output_pathname = os.path.join(output_dir, output_filename)

    os.makedirs(output_dir, exist_ok=True)

    print(input_filename)

    with open(input_pathname) as input_file:
        data = json.load(input_file)
        op_detections = []
        ip_detections = data["detections"]
        for obj in ip_detections:
            loc = obj.get("relative_location")
            if loc is not None:
                x = loc["x"]
                y = loc["y"]
                dist = math.sqrt(x**2 + y**2)
                scale = min(MAX_SCALING_FACTOR, dist / args.nominal_distance)

                p_detected = min(1.0, random.random() / scale) if scale > 0.0 else 0.0
                if args.perception_fail <= 0.0 or p_detected > args.perception_fail:
                    if args.misclassify:
                        misclassify_object(obj, scale)

                    delta_x = (2.0 * random.random() - 1.0) * args.max_shift * scale
                    delta_y = (2.0 * random.random() - 1.0) * args.max_shift * scale
                    loc["x"] = x + delta_x
                    loc["y"] = y + delta_y

                    rot = obj.get("relative_rotation")
                    if rot is not None:
                        delta_yaw = (
                            (2.0 * random.random() - 1.0) * args.max_angle * scale
                        )
                        rot["yaw"] = rot["yaw"] + delta_yaw

                    bb = obj.get("bounding_box")
                    if bb is not None:
                        ext = bb.get("extent")
                        if ext is not None:
                            delta_ex = (
                                (2.0 * random.random() - 1.0)
                                * args.max_size_change
                                * scale
                            )
                            delta_ey = (
                                (2.0 * random.random() - 1.0)
                                * args.max_size_change
                                * scale
                            )
                            delta_ez = (
                                (2.0 * random.random() - 1.0)
                                * args.max_size_change
                                * scale
                            )
                            ext["x"] = ext["x"] + delta_ex
                            ext["y"] = ext["y"] + delta_ey
                            ext["z"] = ext["z"] + delta_ez

                    op_detections.append(obj)

        data["detections"] = op_detections
        with open(output_pathname, "w") as output_file:
            json.dump(data, output_file, ensure_ascii=False, indent=4, sort_keys=True)


def check_args(args):
    if not os.path.exists(args.input):
        print("Please provide a valid input JSON file or folder")
        return False

    if args.output == "":
        print("Please provide a valid output JSON file or folder")
        return False

    input_filename = os.path.basename(args.input)
    output_filename = os.path.basename(args.output)
    if input_filename == "" and output_filename != "":
        print("Output must be a folder if input is a folder")
        return False

    input_dir = os.path.dirname(args.input)
    output_dir = os.path.dirname(args.output)
    if input_dir == output_dir:
        print("Input and output folders must be different")
        return False

    return True


def main():
    argparser = argparse.ArgumentParser(
        description="Manipulate perception sensor data to introduce noise, etc."
    )
    argparser.add_argument(
        "-i",
        "--input",
        metavar="I",
        default="",
        type=str,
        help=f"input JSON file or folder of files to process",
    )
    argparser.add_argument(
        "-o",
        "--output",
        metavar="O",
        default="",
        type=str,
        help=f"output JSON file to generate or folder to generate into",
    )
    argparser.add_argument(
        "-a",
        "--max_angle",
        metavar="A",
        default=DEF_MAX_ANGLE,
        type=float,
        help=f"maximum +/- deviation of yaw rotation to apply, in degrees (default: {DEF_MAX_ANGLE})",
    )
    argparser.add_argument(
        "-s",
        "--max_shift",
        metavar="S",
        default=DEF_MAX_XY_SHIFT,
        type=float,
        help=f"maximum +/- deviation of X any Y to apply, in metres (default: {DEF_MAX_XY_SHIFT})",
    )
    argparser.add_argument(
        "-z",
        "--max_size_change",
        metavar="Z",
        default=DEF_MAX_SIZE_CHANGE,
        type=float,
        help=f"maximum +/- deviation of bounding box extent, in metres (default: {DEF_MAX_SIZE_CHANGE})",
    )
    argparser.add_argument(
        "-n",
        "--nominal_distance",
        metavar="N",
        default=DEF_NOMINAL_DISTANCE,
        type=float,
        help=f"nominal distance for maximum deviation, in metres (default: {DEF_NOMINAL_DISTANCE})",
    )
    argparser.add_argument(
        "-p",
        "--perception_fail",
        metavar="P",
        default=DEF_PERCEPTION_FAILURE_PROBABILITY,
        type=float,
        help=f"probability of perception failure for each object, 0.0 to 1.0  (default: {DEF_PERCEPTION_FAILURE_PROBABILITY})",
    )
    argparser.add_argument(
        "-m",
        "--misclassify",
        action="store_true",
        help="allow objects to be misclassified sometimes",
    )

    print("Perception Noise Generator")
    args = argparser.parse_args()
    if not check_args(args):
        return

    input_filename = os.path.basename(args.input)
    if input_filename == "":
        # Process entire folder of input JSON files
        input_dir = os.path.dirname(args.input)
        for file in os.listdir(input_dir):
            if file.endswith(".json"):
                pathname = os.path.join(input_dir, file)
                process_file(args, pathname, args.output)
    else:
        # Convert single file
        process_file(args, args.input, args.output)

    print("Done")


if __name__ == "__main__":
    main()
