#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This program takes a JSON perception file and an optional set of PLY
LiDAR scans and creates a top-down reconstruction image of the scene
centred on the ego vehicle and showing bounding boxes of all objects.

It can also take a folder full of such perecption files for batch
processing.
"""

import argparse
import json
import os

import carla

from PIL import Image, ImageDraw, ImageFont

from carlasim.core.utilities import *
from carlasim.utilities.ply_reader import PLYManager

DEF_RANGE = 25.0
DEF_PIXELS_PER_METRE = 16

QUALITY_SCALE = 8
OUTLINE_WIDTH = 2
GRID_LINE_WIDTH = 1
VELOCITY_LINE_WIDTH = 2
POINT_SIZE = 3

MAJOR_GRID_SPACING = 10
MINOR_GRID_SPACING = 1

BG_COLOUR = (64, 64, 64)
MAJOR_LINE_COLOUR = (128, 128, 128)
MINOR_LINE_COLOUR = (96, 96, 96)
TEXT_COLOUR = (255, 255, 255)
PROXIMITY_COLOUR = (255, 224, 0)
VELOCITY_COLOUR = (255, 128, 25)
GT_VELOCITY_COLOUR = (255, 64, 25)
GT_OUTLINE_COLOUR = (255, 64, 58)
POINT_CLOUD_COLOUR = (0xB5, 0x84, 0x63)

FONT_TYPEFACE = "/usr/share/fonts/truetype/ubuntu/UbuntuMono-B.ttf"
FONT_SIZE = 30

TEXT_TL_OFFSET_X = 41
TEXT_TL_OFFSET_Y = 31
TEXT_BR_OFFSET_X = 263
TEXT_BR_OFFSET_Y = 65

GROUND_THRESHOLD = -1.2


def sub_point(p1, p2, factor=0.5):
    """
    Returns a point on the line between p1 and p2 according to factor.
    """
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x = x1 + factor * (x2 - x1)
    y = y1 + factor * (y2 - y1)
    return (int(x), int(y))


def vector_to_point(vector, mid, ppm):
    """
    Returns image co-ordinates given a vector from the mid point and
    pixels per metre (ppm).
    """
    return (mid + vector.y * ppm, mid - vector.x * ppm)


def bounding_box_vertices(extent, loc, rot):
    """
    Returns world coordinates of a bounding box whose extent, loc(ation)
    and rot(ation) are provided as dictionary objects.
    """
    ex = extent["x"]
    ey = extent["y"]
    ez = extent["z"]
    transform = carla.Transform(
        carla.Location(x=loc["x"], y=loc["y"], z=loc["z"]),
        carla.Rotation(roll=rot["roll"], pitch=rot["pitch"], yaw=rot["yaw"]),
    )

    v0 = get_world_coordinate(transform, carla.Vector3D(-ex, -ey, -ez))
    v1 = get_world_coordinate(transform, carla.Vector3D(-ex, -ey, ez))
    v2 = get_world_coordinate(transform, carla.Vector3D(-ex, ey, -ez))
    v3 = get_world_coordinate(transform, carla.Vector3D(-ex, ey, ez))
    v4 = get_world_coordinate(transform, carla.Vector3D(ex, -ey, -ez))
    v5 = get_world_coordinate(transform, carla.Vector3D(ex, -ey, ez))
    v6 = get_world_coordinate(transform, carla.Vector3D(ex, ey, -ez))
    v7 = get_world_coordinate(transform, carla.Vector3D(ex, ey, ez))
    return [v0, v1, v2, v3, v4, v5, v6, v7]


def velocity_vector(velocity, loc, rot):
    """
    Returns a vector in world coordinates from a velocity vector, origin
    loc(ation) and rot(ation), provided as dictionary objects.
    """
    vel = carla.Vector3D(velocity["x"], velocity["y"], velocity["z"])
    transform = carla.Transform(
        carla.Location(x=loc["x"], y=loc["y"], z=loc["z"]),
        carla.Rotation(roll=rot["roll"], pitch=rot["pitch"], yaw=rot["yaw"]),
    )
    return get_world_vector(transform, vel)


def draw_grid(ppm, mid, full_range, spacing, colour, draw):
    """
    Draws a background grid for the reconstrcution image to help
    associate scale.
    """
    g = 0
    fr = full_range * ppm
    line_width = QUALITY_SCALE * GRID_LINE_WIDTH
    while g <= full_range:
        v = g * ppm
        draw.line(
            ((mid - fr, mid + v), (mid + fr, mid + v)), fill=colour, width=line_width
        )
        draw.line(
            ((mid + v, mid - fr), (mid + v, mid + fr)), fill=colour, width=line_width
        )
        if g > 0:
            draw.line(
                ((mid - fr, mid - v), (mid + fr, mid - v)),
                fill=colour,
                width=line_width,
            )
            draw.line(
                ((mid - v, mid - fr), (mid - v, mid + fr)),
                fill=colour,
                width=line_width,
            )

        g += spacing


def draw_point(ppm, mid, x, y, colour, draw):
    """
    Plots a single (liDAR scan) point on the image.
    """
    sx = mid + y * ppm
    sy = mid - x * ppm
    offset = POINT_SIZE * QUALITY_SCALE * 0.5
    draw.ellipse((sx - offset, sy - offset, sx + offset, sy + offset), fill=colour)


def add_object_to_image(args, ppm, mid, draw, obj, gt=False):
    """
    Draws everything associated with a single object onto the image,
    including a bounding box, proximity box, direction arrow and
    velocity vector.

    If the ground-truth flag (gt) is True then an appropriate overlay
    bounding box is also drawn, for comparison.
    """
    loc = obj.get("relative_location")
    if loc is None:
        loc = {"x": 0, "y": 0, "z": 0}

    rot = obj.get("relative_rotation")
    if rot is None:
        rot = {"yaw": 0, "pitch": 0, "roll": 0}

    bb = obj.get("bounding_box")
    if bb is None:
        print("No bounding box!")
        return

    extent = bb.get("extent")
    if extent is None:
        print("No bounding box extent!")
        return

    velocity = obj.get("velocity")

    fill_colour = (192, 192, 192)
    outline_colour = (255, 255, 255)
    proximity_colour = PROXIMITY_COLOUR
    velocity_color = VELOCITY_COLOUR
    obj_type = obj.get("type")
    if gt:
        fill_colour = None
        outline_colour = GT_OUTLINE_COLOUR
        velocity_color = GT_VELOCITY_COLOUR
    elif obj_type is not None:
        if obj_type == "car":
            fill_colour = (0x27, 0x7D, 0xA1)
            outline_colour = (0x30, 0x98, 0xC5)
        elif obj_type == "motorbike":
            fill_colour = (0x90, 0xBE, 0x6D)
            outline_colour = (0xA9, 0xCC, 0x8E)
        elif obj_type == "campervan":
            fill_colour = (0x43, 0xAA, 0x8B)
            outline_colour = (0x5F, 0xBF, 0xA2)
        elif obj_type == "truck":
            fill_colour = (0x57, 0x75, 0x90)
            outline_colour = (0x73, 0x91, 0xAB)
        elif obj_type == "emergency":
            fill_colour = (0xF9, 0x41, 0x44)
            outline_colour = (0xFB, 0x74, 0x77)
        elif obj_type == "bicycle":
            fill_colour = (0xF9, 0x84, 0x4A)
            outline_colour = (0xFB, 0xA1, 0x74)
        elif obj_type == "pedestrian":
            fill_colour = (0xF9, 0xC7, 0x4F)
            outline_colour = (0xFC, 0xE5, 0xB0)

    line_width = QUALITY_SCALE * OUTLINE_WIDTH

    # proximity threshold box
    proximity = obj.get("proximity_threshold")
    if proximity is not None and proximity > 0.0:
        prox_extent = {
            "x": extent["x"] + proximity,
            "y": extent["y"] + proximity,
            "z": extent["z"],
        }
        prox_vertices = bounding_box_vertices(prox_extent, loc, rot)
        pp1 = vector_to_point(prox_vertices[0], mid, ppm)
        pp2 = vector_to_point(prox_vertices[2], mid, ppm)
        pp3 = vector_to_point(prox_vertices[6], mid, ppm)
        pp4 = vector_to_point(prox_vertices[4], mid, ppm)
        draw.line(
            (pp1, pp2, pp3, pp4, pp1, pp2), fill=proximity_colour, width=line_width
        )

    # bounding box
    vertices = bounding_box_vertices(extent, loc, rot)
    p1 = vector_to_point(vertices[0], mid, ppm)
    p2 = vector_to_point(vertices[2], mid, ppm)
    p3 = vector_to_point(vertices[6], mid, ppm)
    p4 = vector_to_point(vertices[4], mid, ppm)
    if not gt:
        draw.polygon((p1, p2, p3, p4), fill=fill_colour)
    draw.line((p1, p2, p3, p4, p1, p2), fill=outline_colour, width=line_width)

    # direction arrow
    p5 = sub_point(p1, p4, 0.75)
    p6 = sub_point(p3, p4, 0.5)
    p7 = sub_point(p2, p3, 0.75)
    draw.line((p5, p6, p7), fill=outline_colour, width=line_width)

    # velocity vector
    if args.velocities and velocity is not None:
        vel = velocity_vector(velocity, loc, rot)
        velocity_width = QUALITY_SCALE * VELOCITY_LINE_WIDTH
        p8 = sub_point(p1, p3, 0.5)
        p9 = (p8[0] + vel.y * ppm, p8[1] - vel.x * ppm)
        draw.line((p8, p9), fill=velocity_color, width=velocity_width)


def generate_image_from_file(args, ply_manager, json_pathname, image_pathname):
    """
    Processes a single JSON file to produce a reconstruction image file.
    """
    json_filename = os.path.basename(json_pathname)
    json_dir = os.path.dirname(json_pathname)
    image_filename = os.path.basename(image_pathname)
    image_dir = os.path.dirname(image_pathname)
    base_name = os.path.splitext(json_filename)[0]
    time_offset = base_name.split("_")[-1] + " sec"
    if image_filename == "":
        image_filename = base_name + ".png"
        image_pathname = os.path.join(image_dir, image_filename)

    os.makedirs(image_dir, exist_ok=True)

    print(json_filename)

    with open(json_pathname) as json_file:
        data = json.load(json_file)

        half_size = int(args.range * args.ppm)
        size = 2 * half_size

        working_size = QUALITY_SCALE * size
        mid = QUALITY_SCALE * half_size
        ppm = QUALITY_SCALE * args.ppm

        image = Image.new("RGB", (working_size, working_size), BG_COLOUR)
        draw = ImageDraw.Draw(image)

        draw_grid(ppm, mid, args.range, MINOR_GRID_SPACING, MINOR_LINE_COLOUR, draw)
        draw_grid(ppm, mid, args.range, MAJOR_GRID_SPACING, MAJOR_LINE_COLOUR, draw)

        ego_vehicle = data["ego_vehicle"]
        add_object_to_image(args, ppm, mid, draw, ego_vehicle)

        detections = data["detections"]
        for det in detections:
            add_object_to_image(args, ppm, mid, draw, det)

        if ply_manager is not None:
            points = ply_manager.get_points(json_filename)
            if points is not None:
                # TODO: extrinsics transform
                for pt in points:
                    x = pt[0]
                    y = pt[1]
                    z = pt[2]
                    # if math.fabs(x) > 1.0 and math.fabs(y) > 1.0 and math.fabs(x) < args.range and math.fabs(y) < args.range:
                    if (
                        z >= GROUND_THRESHOLD
                        and math.fabs(x) <= args.range
                        and math.fabs(y) <= args.range
                    ):
                        draw_point(ppm, mid, x, y, POINT_CLOUD_COLOUR, draw)

        if args.gt != "":
            gt_pathname = os.path.join(args.gt, json_filename)
            with open(gt_pathname) as gt_file:
                gt_data = json.load(gt_file)
                gt_detections = gt_data["detections"]
                for gt_det in gt_detections:
                    add_object_to_image(args, ppm, mid, draw, gt_det, gt=True)

        tx = QUALITY_SCALE * TEXT_TL_OFFSET_X
        ty = QUALITY_SCALE * TEXT_TL_OFFSET_Y
        if args.br:
            tx = working_size - QUALITY_SCALE * TEXT_BR_OFFSET_X
            ty = working_size - QUALITY_SCALE * TEXT_BR_OFFSET_Y

        font_size = QUALITY_SCALE * FONT_SIZE
        font = ImageFont.truetype(FONT_TYPEFACE, font_size)
        draw.text((tx, ty), time_offset, font=font, fill=TEXT_COLOUR)

        output_image = image.resize((size, size), Image.BILINEAR)
        output_image.save(image_pathname)


def check_args(args):
    """
    Returns True if all important command line arguments are valid.
    """
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

    if args.gt != "":
        gt_filename = os.path.basename(args.gt)
        if gt_filename != "":
            print("Ground-truth input must be a folder when provided")
            return False

    return True


def main():
    argparser = argparse.ArgumentParser(
        description="Generate images from perception sensor JSON output"
    )
    argparser.add_argument(
        "-g",
        "--gt",
        metavar="G",
        default="",
        type=str,
        help=f"input folder containing corresponding ground-truth JSON files",
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
        "-l",
        "--lidar",
        metavar="L",
        default="",
        type=str,
        help=f"input folder full of LiDAR PLY files (optional)",
    )
    argparser.add_argument(
        "-r",
        "--range",
        metavar="R",
        default=DEF_RANGE,
        type=float,
        help=f"range of image from ego vehicle, metres (default: {DEF_RANGE})",
    )
    argparser.add_argument(
        "-p",
        "--ppm",
        metavar="P",
        default=DEF_PIXELS_PER_METRE,
        type=int,
        help=f"pixels per metre (default: {DEF_PIXELS_PER_METRE})",
    )
    argparser.add_argument(
        "-v",
        "--velocities",
        action="store_true",
        help="include velocity vectors in images",
    )
    argparser.add_argument(
        "-b",
        "--br",
        action="store_true",
        help="put timestamp text in the bottom-right instead of the top-left",
    )

    print("Perception Reconstruction Image Generator")
    args = argparser.parse_args()
    if not check_args(args):
        return

    ply_manager = None
    if args.lidar != "":
        ply_manager = PLYManager(args.lidar)

    input_filename = os.path.basename(args.input)
    if input_filename == "":
        # Convert entire folder of input JSON files
        input_dir = os.path.dirname(args.input)
        for file in os.listdir(input_dir):
            if file.endswith(".json"):
                pathname = os.path.join(input_dir, file)
                generate_image_from_file(args, ply_manager, pathname, args.output)
    else:
        # Convert single file
        generate_image_from_file(args, ply_manager, args.input, args.output)

    print("Done")


if __name__ == "__main__":
    main()
