#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import json
import os

from .edr_camera import EDRCamera
from .edr_lidar3d import EDRLidar3D

DEF_CAMERA_WIDTH = 720
DEF_CAMERA_HEIGHT = 480
DEF_CAMERA_FOV = 110
DEF_CAMERA_RATE = 10.0


# ==============================================================================
# -- EDRLoader -----------------------------------------------------------------
# ==============================================================================


class EDRLoader(object):
    """
    Loads EDR perception sensors from a JSON file with the following
    general format:

        {
            "lidars3d": [
                {
                    "label": "<lidar-id>",
                    "x": <+x-is-forward = 0.0>,
                    "y": <+y-is-right = 0.0>,
                    "z": <+z-is-up = 0.0>,
                    "roll": <+roll-is-clockwise = 0.0>,
                    "pitch": <+pitch-is-up = 0.0>,
                    "yaw": <+yaw-is-right = 0.0>
                },
                {
                    <next-lidar-configuration>
                }
            ],
            "cameras": [
                {
                    "label": "<camera-id>",
                    "x": <+x-is-forward = 0.0>,
                    "y": <+y-is-right = 0.0>,
                    "z": <+z-is-up = 0.0>,
                    "roll": <+roll-is-clockwise = 0.0>,
                    "pitch": <+pitch-is-up = 0.0>,
                    "yaw": <+yaw-is-right = 0.0>
                    "width": <width-in-pixels = 720>,
                    "height": <height-in-pixels = 480>,
                    "fov": <degrees = 110>,
                    "rate": <hz = 10.0>
                },
                {
                    <next-camera-configuration>
                }
            ]

        }
    """

    def __init__(self, parent_actor, preevent_time, postevent_time):
        self.parent_actor = parent_actor
        self.preevent_time = preevent_time
        self.postevent_time = postevent_time

    def load_sensors(self, pathname, edr):
        """
        Loads sensor definitions from a JSON file, creates the sensors
        and adds them to the EDR.
        """
        print("EDR Sensor Configuration:", os.path.basename(pathname))
        try:
            with open(pathname) as file:
                data = json.load(file)

                lidars = data.get("lidars3d", [])
                print("Lidars loaded:", len(lidars))
                for lidar in lidars:
                    label = lidar.get("label", "")
                    if label != "":
                        transform = self._get_transform(lidar)
                        sensor = EDRLidar3D(
                            self.parent_actor,
                            self.preevent_time,
                            self.postevent_time,
                            label,
                            transform,
                        )
                        edr.add_sensor(sensor)

                cameras = data.get("cameras", [])
                print("Cameras loaded:", len(cameras))
                for camera in cameras:
                    label = camera.get("label", "")
                    if label != "":
                        transform = self._get_transform(camera)
                        width = camera.get("width", DEF_CAMERA_WIDTH)
                        height = camera.get("height", DEF_CAMERA_HEIGHT)
                        fov = camera.get("fov", DEF_CAMERA_FOV)
                        rate = camera.get("rate", DEF_CAMERA_RATE)
                        sensor = EDRCamera(
                            self.parent_actor,
                            self.preevent_time,
                            self.postevent_time,
                            label,
                            transform,
                            width,
                            height,
                            fov,
                            rate,
                        )
                        edr.add_sensor(sensor)

            return True

        except:
            return False

    def _get_transform(self, info):
        """
        PRIVATE: Returns a CARLA Transform created from the given
        information dictionary object.
        """
        x = info.get("x", 0.0)
        y = info.get("y", 0.0)
        z = info.get("z", 0.0)
        roll = info.get("roll", 0.0)
        pitch = info.get("pitch", 0.0)
        yaw = info.get("yaw", 0.0)
        return carla.Transform(
            carla.Location(x, y, z), carla.Rotation(pitch, yaw, roll)
        )
