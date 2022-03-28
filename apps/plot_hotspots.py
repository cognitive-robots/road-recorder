#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Given a Vulnerable Road User (VRU) hotspot log file, this program can
produce a top-down view of the map (town) showing just the road network
with all of the hotspots plotted.

The program can enter interactive mode in which case the following
controls can be used to drive a car around:

    W            : throttle
    S            : brake
    AD           : steer
    Space        : hand-brake

    ESC          : quit
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import argparse
import glob
import os
import sys
import time

try:
    sys.path.append(
        glob.glob(
            "../carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

import weakref
import random

from carlasim.core.utilities import *

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame package is installed")

try:
    import numpy as np
except ImportError:
    raise RuntimeError("cannot import numpy, make sure numpy package is installed")

VIEW_FOV = 90

DEF_VIEW_WIDTH = 720
DEF_VIEW_HEIGHT = DEF_VIEW_WIDTH

DEF_CAMERA_X = 0
DEF_CAMERA_Y = 0
DEF_CAMERA_Z = 200

DEF_MARKER_RADIUS = 12
DEF_MARKER_OPACITY = 30

DEF_DENSITY_THRESHOLD = 1.0

MARKER_COLOUR = (248, 64, 24)


# ==============================================================================
# -- ClientSideMarkers ---------------------------------------------------------
# ==============================================================================


class ClientSideMarkers(object):
    """
    This is a module responsible for drawing markers at vehicle locations
    client-side on a pygame surface. A marker is an (x,y) coordinate.
    """

    @staticmethod
    def get_location_markers(locations, camera):
        """
        Creates markers based on locations list and camera.
        """
        markers = [
            ClientSideMarkers.get_location_marker(location, camera)
            for location in locations
        ]
        return markers

    @staticmethod
    def get_location_marker(location, camera):
        """
        Returns a marker for a vehicle based on the camera view.
        location is a carla.Vector3D
        """
        world_cord = np.transpose([[location.x, location.y, location.z, 1.0]])
        cords_x_y_z = ClientSideMarkers._world_to_sensor(world_cord, camera)
        cords_y_minus_z_x = np.concatenate(
            [cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]]
        )
        mkr = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_marker = np.concatenate(
            [mkr[:, 0] / mkr[:, 2], mkr[:, 1] / mkr[:, 2], mkr[:, 2]], axis=1
        )
        marker = (camera_marker[0, 0], camera_marker[0, 1])
        return marker

    @staticmethod
    def get_vehicle_markers(vehicles, camera):
        """
        Creates markers based on CARLA vehicle list and camera.
        """
        markers = [
            ClientSideMarkers.get_vehicle_marker(vehicle, camera)
            for vehicle in vehicles
        ]
        return markers

    @staticmethod
    def get_vehicle_marker(vehicle, camera):
        """
        Returns a marker for a vehicle based on the camera view.
        """
        cords_x_y_z = ClientSideMarkers._vehicle_to_sensor(vehicle, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate(
            [cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]]
        )
        mkr = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_marker = np.concatenate(
            [mkr[:, 0] / mkr[:, 2], mkr[:, 1] / mkr[:, 2], mkr[:, 2]], axis=1
        )
        marker = (camera_marker[0, 0], camera_marker[0, 1])
        return marker

    @staticmethod
    def draw_markers(args, display, markers):
        """
        Draws markers on pygame display.
        """
        colour = (
            MARKER_COLOUR[0],
            MARKER_COLOUR[1],
            MARKER_COLOUR[2],
            args.marker_opacity,
        )
        for mkr in markers:
            surface = pygame.Surface(
                (args.view_width, args.view_height), flags=pygame.SRCALPHA
            )
            pygame.draw.circle(surface, colour, mkr, args.marker_radius)
            display.blit(surface, (0, 0))

    # -------
    # PRIVATE
    # -------

    @staticmethod
    def _vehicle_to_sensor(vehicle, sensor):
        """
        Transforms coordinates of a vehicle marker to sensor.
        """
        world_cord = ClientSideMarkers._vehicle_to_world(vehicle)
        sensor_cord = ClientSideMarkers._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """
        cords = np.zeros((1, 4))
        cords[0, :] = np.array([0, 0, 0, 1])

        vehicle_world_matrix = ClientSideMarkers._get_matrix(vehicle.get_transform())
        world_cords = np.dot(vehicle_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """
        sensor_world_matrix = ClientSideMarkers._get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def _get_matrix(transform):
        """
        Creates matrix from CARLA transform.
        """
        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self, args):
        self.args = args

        self.client = None
        self.world = None
        self.camera = None
        self.car = None

        self.display = None
        self.image = None
        self.capture = True

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """
        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self.args.view_width))
        camera_bp.set_attribute("image_size_y", str(self.args.view_height))
        camera_bp.set_attribute("fov", str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """
        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controlled.
        """
        car_bp = self.world.get_blueprint_library().filter("vehicle.*")[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """
        camera_transform = carla.Transform(
            carla.Location(self.args.camera_x, self.args.camera_y, self.args.camera_z),
            carla.Rotation(pitch=-90),
        )
        self.camera = self.world.spawn_actor(
            self.camera_blueprint(), camera_transform, attach_to=None
        )
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = self.args.view_width / 2.0
        calibration[1, 2] = self.args.view_height / 2.0
        calibration[0, 0] = calibration[1, 1] = self.args.view_width / (
            2.0 * np.tan(VIEW_FOV * np.pi / 360.0)
        )
        self.camera.calibration = calibration

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True if ESCAPE is hit to end the main loop, otherwise False.
        """
        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True

        if car is None:
            return False

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1.0, min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1.0, max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets the image coming from the overhead camera sensor.
        self.capture is used for synchronization; when set, next image is stored.
        """
        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    def render(self, display):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """
        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

    def load_log_file(self, args, client):
        """
        Loads the log file containing the hotspots to be plotted.
        """
        log_pathname = args.input
        if not os.path.exists(log_pathname):
            return None

        locations = []
        total = 0
        count = 0
        with open(log_pathname, mode="r") as file:
            map_name = file.readline().split(",")[0].strip() + "_Opt"
            print("Loading map:", map_name)
            client.load_world(map_name)

            last_loc = None
            lines = file.readlines()
            for line in lines:
                line.strip()
                if line[0] == "#":
                    continue

                items = line.split(",")
                if len(items) >= 11:
                    total += 1
                    x = float(items[8])  # EGO X
                    y = float(items[9])  # EGO Y
                    z = float(items[10])  # EGO Z
                    loc = carla.Vector3D(x, y, z)
                    if (
                        last_loc is None
                        or distance_between(loc, last_loc) > args.density
                    ):
                        locations.append(loc)
                        last_loc = loc
                        count += 1

        print(f"Locations used: {count} of {total}")
        return locations

    def game_loop(self, args):
        """
        Main program loop.
        """
        try:
            pygame.init()

            self.client = carla.Client("127.0.0.1", 2000)
            self.client.set_timeout(9.0)

            locations = self.load_log_file(args, self.client)

            self.world = self.client.get_world()

            self.world.load_map_layer(carla.MapLayer.All)
            time.sleep(2)
            self.world.unload_map_layer(
                carla.MapLayer(
                    carla.MapLayer.Buildings
                    | carla.MapLayer.Decals
                    | carla.MapLayer.Foliage
                    | carla.MapLayer.ParkedVehicles
                    | carla.MapLayer.Particles
                    | carla.MapLayer.Props
                    | carla.MapLayer.StreetLights
                    | carla.MapLayer.Walls
                )
            )
            time.sleep(2)

            if locations is None:
                self.setup_car()

            self.setup_camera()

            self.display = pygame.display.set_mode(
                (args.view_width, args.view_height), pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)
            vehicles = self.world.get_actors().filter("vehicle.*")

            while True:
                self.world.tick()

                self.capture = True
                pygame_clock.tick_busy_loop(20)

                self.render(self.display)

                if locations is None:
                    markers = ClientSideMarkers.get_vehicle_markers(
                        vehicles, self.camera
                    )
                else:
                    markers = ClientSideMarkers.get_location_markers(
                        locations, self.camera
                    )

                ClientSideMarkers.draw_markers(args, self.display, markers)

                pygame.display.flip()
                pygame.event.pump()

                # if locations is not None or self.control(self.car):
                if self.control(self.car):
                    return

        finally:
            if args.output is not None and args.output != "":
                print("Saving image:", args.output)
                output_dir = os.path.dirname(args.output)
                os.makedirs(output_dir, exist_ok=True)
                pygame.image.save(self.display, args.output)

            self.set_synchronous_mode(False)
            self.camera.destroy()
            if self.car is not None:
                self.car.destroy()
            pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the hotspot plotting application.
    """
    argparser = argparse.ArgumentParser(description="CARLA Hotspot Plotter")
    argparser.add_argument(
        "--input", default="", help="input hotspots log filename (default: not set)"
    )
    argparser.add_argument(
        "--output", default="", help="optional output image filename (default: not set)"
    )
    argparser.add_argument(
        "--view",
        metavar="WIDTH,HEIGHT",
        default=f"{DEF_VIEW_WIDTH},{DEF_VIEW_HEIGHT}",
        help=f"top-down camera view size (default: {DEF_VIEW_WIDTH},{DEF_VIEW_HEIGHT})",
    )
    argparser.add_argument(
        "--camera",
        metavar="X,Y,Z",
        default=f"{DEF_CAMERA_X},{DEF_CAMERA_Y},{DEF_CAMERA_Z}",
        help=f"top-down camera position (default: {DEF_CAMERA_X},{DEF_CAMERA_Y},{DEF_CAMERA_Z})",
    )
    argparser.add_argument(
        "--marker_radius",
        default=DEF_MARKER_RADIUS,
        type=float,
        help=f"size to draw marker circles (default: {DEF_MARKER_RADIUS})",
    )
    argparser.add_argument(
        "--marker_opacity",
        default=DEF_MARKER_OPACITY,
        type=int,
        help=f"opacity to draw marker circles, 0 - 255 (default: {DEF_MARKER_OPACITY})",
    )
    argparser.add_argument(
        "--density",
        default=DEF_DENSITY_THRESHOLD,
        type=float,
        help=f"minimum distance between successive log locations (default: {DEF_DENSITY_THRESHOLD})",
    )

    args = argparser.parse_args()
    args.view_width, args.view_height = [int(v) for v in args.view.split(",")]
    args.camera_x, args.camera_y, args.camera_z = [
        float(v) for v in args.camera.split(",")
    ]

    try:
        client = BasicSynchronousClient(args)
        client.game_loop(args)
    finally:
        print("EXIT")


if __name__ == "__main__":
    main()
