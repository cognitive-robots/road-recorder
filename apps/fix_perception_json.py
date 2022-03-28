#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
A simple program to replace the bounding boxes of all bicycles in a
perception JSON file with a fixed 'reasonable' version. This is
necessary because CARLA sometimes reports incorrect bounding boxes
for some bicycles.

In addition, this program allows clipping of all detections until a
given relative timestamp which is useful for some scenarios.
"""

import argparse
import json
import os


DEF_TIMESTAMP = -100.0


def fix_detections(args, input_pathname, output_pathname):
    input_filename = os.path.basename(input_pathname)
    output_filename = os.path.basename(output_pathname)
    output_dir = os.path.dirname(output_pathname)
    base_name = os.path.splitext(input_filename)[0]
    time_offset = float(base_name.split("_")[-1])
    if output_filename == "":
        output_filename = input_filename
        output_pathname = os.path.join(output_dir, output_filename)

    os.makedirs(output_dir, exist_ok=True)

    print(input_filename)

    with open(input_pathname) as input_file:
        data = json.load(input_file)

        if time_offset < args.timestamp:
            data["detections"] = []
        else:
            detections = data["detections"]
            for det in detections:
                actor_type = det["type"]
                if actor_type == "bicycle":
                    # Fix bicycle bounding box extents
                    bb = det["bounding_box"]
                    bb["extent"] = {
                        "x": 0.9177202582359314,
                        "y": 0.26446444392204285,
                        "z": 0.8786712288856506,
                    }

        with open(output_pathname, "w") as output_file:
            json.dump(data, output_file, indent=4)


def check_args(args):
    if not os.path.exists(args.input):
        print("Please provide a valid input JSON file or folder")
        return False

    if args.output == "":
        print("Please provide a valid output image file or folder")
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
        description="Fix detections in perception sensor JSON output"
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
        help=f"output image file to generate or folder to generate into",
    )
    argparser.add_argument(
        "-t",
        "--timestamp",
        metavar="T",
        default=DEF_TIMESTAMP,
        type=float,
        help=f"timestamp threshold to clip detections before (default: {DEF_TIMESTAMP})",
    )

    print("Fix Perception Detections")
    args = argparser.parse_args()
    if not check_args(args):
        return

    input_filename = os.path.basename(args.input)
    if input_filename == "":
        # Convert entire folder of input JSON files
        input_dir = os.path.dirname(args.input)
        for file in os.listdir(input_dir):
            if file.endswith(".json"):
                pathname = os.path.join(input_dir, file)
                fix_detections(args, pathname, args.output)
    else:
        # Convert single file
        fix_detections(args, args.input, args.output)

    print("Done")


if __name__ == "__main__":
    main()
