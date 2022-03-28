#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import numpy as np
import re
import transforms3d

import carla


# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")
    name = lambda x: " ".join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = " ".join(actor.type_id.replace("_", ".").title().split(".")[1:])
    return (name[: truncate - 1] + "\u2026") if len(name) > truncate else name


def get_actor_type(actor):
    bp_parts = actor.type_id.split(".")
    actor_type = bp_parts[0]
    if actor_type == "vehicle":
        name = bp_parts[-1]
        if name == "crossbike" or name == "omafiets" or name == "century":
            actor_type = "bicycle"
        elif name == "ninja" or name == "yzf" or name == "low_rider":
            actor_type = "motorbike"
        elif name == "police" or name == "chargercop2020":
            actor_type = "emergency"
        elif name == "carlacola":
            actor_type = "truck"
        elif name == "t2":
            actor_type = "campervan"
        else:
            actor_type = "car"

    elif actor_type == "walker":
        actor_type = "pedestrian"

    return actor_type


def get_proximity_threshold(actor_type):
    if actor_type == "pedestrian":
        return 0.75

    if actor_type == "bicycle":
        return 1.0

    return 0.0


def get_local_coordinate(transform, world_coordinate):
    mat = np.array(transform.get_inverse_matrix())
    wc = np.array([world_coordinate.x, world_coordinate.y, world_coordinate.z, 1.0])
    lc = mat.dot(wc)
    return carla.Vector3D(lc[0], lc[1], lc[2])


def get_world_coordinate(transform, local_coordinate):
    mat = np.array(transform.get_matrix())
    lc = np.array([local_coordinate.x, local_coordinate.y, local_coordinate.z, 1.0])
    wc = mat.dot(lc)
    return carla.Vector3D(wc[0], wc[1], wc[2])


def get_local_vector(transform, world_vector):
    mat = np.array(transform.get_inverse_matrix())
    rot_mat = mat[:3, :3]
    wv = np.array([world_vector.x, world_vector.y, world_vector.z])
    lv = rot_mat.dot(wv)
    return carla.Vector3D(lv[0], lv[1], lv[2])


def get_world_vector(transform, local_vector):
    mat = np.array(transform.get_matrix())
    rot_mat = mat[:3, :3]
    lv = np.array([local_vector.x, local_vector.y, local_vector.z])
    wv = rot_mat.dot(lv)
    return carla.Vector3D(wv[0], wv[1], wv[2])


def get_vector_norm(vector):
    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


def distance_between(vector1, vector2):
    return get_vector_norm(vector2 - vector1)


def mat2transform(M):
    yaw, pitch, roll = transforms3d.taitbryan.mat2euler(M[0:3, 0:3])
    yaw = np.rad2deg(yaw)
    pitch = np.rad2deg(pitch)
    roll = np.rad2deg(roll)

    T = carla.Transform(
        carla.Location(x=M[0, 3], y=M[1, 3], z=M[2, 3]),
        carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
    )

    return T


def relative_transform(source, target):
    """Return Transform of target with reference to source frame
    (right-handed co-ordinate frame: X=forward, Y=left, Z=down)
    """
    MaI = np.array(source.get_inverse_matrix())
    Mb = np.array(target.get_matrix())
    Mab = np.dot(MaI, Mb)
    Tab = mat2transform(Mab)
    return Tab
