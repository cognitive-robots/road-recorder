#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import os
import random
import sys
import time

import carla

from edr.edr import EDR
from edr.near_miss_logger import NearMissLogger

from ..edrsensors.edr_loader import EDRLoader
from ..edrsensors.edr_perception import EDRPerceptionSensor
from ..edrsensors.edr_vehicle_state import EDRVehicleStateSensor

from ..sensors.collision_sensor import CollisionSensor
from ..sensors.gnss_sensor import GnssSensor
from ..sensors.lane_sensor import LaneInvasionSensor

from ..utilities.separating_axis_theorem import has_collided

from .camera_manager import CameraManager
from .utilities import *


PERCEPTION_RANGE = 50.0
EDR_SAVE_DELAY = 2.0


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, args, carla_world, hud, data_path):
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print("RuntimeError: {}".format(error))
            print("  The server could not send the OpenDRIVE (.xodr) file:")
            print(
                "  Make sure it exists, has the same name of your town, and is correct."
            )
            sys.exit(1)

        edr_config_exists = os.path.exists(args.edr_sensors)
        if args.edr and not edr_config_exists:
            print(
                "WARNING: EDR configuration file not found, EDR disabled:",
                args.edr_sensors,
            )

        self.hud = hud
        self.player = None
        self.edr = EDR()
        self.edr_sensor_config = args.edr_sensors if edr_config_exists else None
        self.edr_enabled = args.edr and edr_config_exists
        self.edr_sensor = None
        self.edr_perception = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self.data_path = data_path
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._start_index = args.start_index
        self._edr_autosave = args.edr_autosave
        self._edr_savetime = None
        self._edr_pretime = args.edr_pretime
        self._edr_posttime = args.edr_posttime
        self._near_miss_velocity = args.near_miss_velocity
        self.restart()
        self.world.on_tick(hud.on_world_tick)

        self.near_miss_logger = None
        if args.near_miss_log_file is not None and args.near_miss_log_file != "":
            self.near_miss_logger = NearMissLogger(
                args.near_miss_log_file, self.map.name, args.near_miss_sound
            )

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = (
            self.camera_manager.transform_index
            if self.camera_manager is not None
            else 0
        )

        # Get the player vehicle blueprint (use the Tesla rather than
        # random so the 1st-person view works correctly for control).
        # blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint = self.world.get_blueprint_library().find("vehicle.tesla.model3")
        blueprint.set_attribute("role_name", "hero")
        if blueprint.has_attribute("color"):
            color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = carla.Transform()
            if spawn_points:
                spawn_count = len(spawn_points)
                spawn_index = (
                    self._start_index
                    if self._start_index >= 0 and self._start_index < spawn_count
                    else random.randrange(spawn_count)
                )
                spawn_point = spawn_points[spawn_index]
                print(f"Player spawn index: {spawn_index} of {spawn_count}")
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            
        print(f"Player Actor ID: {self.player.id}")

        # Set up the EDR sensors.
        self.edr.clear()
        if self.edr_enabled:
            # Vehicle State
            self.edr_sensor = EDRVehicleStateSensor(
                self.player, self._edr_pretime, self._edr_posttime
            )
            self.edr.add_sensor(self.edr_sensor)
            # Perception
            self.edr_perception = EDRPerceptionSensor(
                self.player, self._edr_pretime, self._edr_posttime,
                self.world, PERCEPTION_RANGE
            )
            self.edr.add_sensor(self.edr_perception)
        
        if self.edr_sensor_config:
            # Load EDR Sensors (whether or not EDR is enabled)
            loader = EDRLoader(self.player, self._edr_pretime, self._edr_posttime)
            loader.load_sensors(self.edr_sensor_config, self.edr)

        # Set up other sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud, self)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def set_simulation_fps(self, fps):
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = (1.0 / fps) if fps > 0.0 else 0.0
        self.world.apply_settings(settings)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification("Weather: %s" % preset[1])
        self.player.get_world().set_weather(preset[0])

    def generate_vehicle_state_data(self):
        if self.edr_enabled and self.edr_sensor is not None:
            self.edr_sensor.generate_data(self.player, self.gnss_sensor)

    def generate_perception_data(self):
        if self.edr_enabled and self.edr_perception is not None:
            self.edr_perception.generate_data(self.player)

    def check_for_near_misses(self):
        if not self.near_miss_logger and (
            not self.edr_enabled or self.edr.has_triggered()
        ):
            return

        player_speed = get_vector_norm(self.player.get_velocity())
        if player_speed < self._near_miss_velocity:
            return

        player_transform = self.player.get_transform()
        player_bb = self.player.bounding_box
        player_bb_vertices = player_bb.get_world_vertices(player_transform)
        player_vertices = []
        for pv in player_bb_vertices:
            player_vertices.append((pv.x, pv.y))

        for actor in self.world.get_actors():
            if not self._is_vru(actor.type_id):
                continue

            actor_type = get_actor_type(actor)
            near_miss_threshold = get_proximity_threshold(actor_type)
            rough_threshold = near_miss_threshold * 8

            vru_transform = actor.get_transform()
            rough_distance = math.sqrt(
                (vru_transform.location.x - player_transform.location.x) ** 2
                + (vru_transform.location.y - player_transform.location.y) ** 2
            )
            if rough_distance > rough_threshold:
                continue

            vru_bb = actor.bounding_box
            vru_bb.extent.x += near_miss_threshold
            vru_bb.extent.y += near_miss_threshold
            vru_bb.extent.z += near_miss_threshold
            vru_bb_vertices = vru_bb.get_world_vertices(vru_transform)

            vru_vertices = []
            for vv in vru_bb_vertices:
                vru_vertices.append((vv.x, vv.y))

            try:
                # NOTE: There's currently no distinction between a near
                #       miss and an actual collision.
                if has_collided(player_vertices, vru_vertices):
                    self.trigger_edr_event("Too close to Vulnerable Road User (VRU)")
                    if self.near_miss_logger is not None:
                        self.near_miss_logger.log_event(
                            near_miss_threshold, near_miss_threshold, actor, self.player
                        )
                    else:
                        return
            except ZeroDivisionError:
                print("separating_axis_theorem failed!")

    def trigger_edr_event(self, reason):
        if self.edr_enabled:
            self.edr.on_event_trigger(reason)
            self._edr_savetime = time.time() + self._edr_posttime + EDR_SAVE_DELAY

    def check_edr_autosave(self):
        if self.edr_enabled and self._edr_savetime is not None and time.time() > self._edr_savetime:
            self._edr_savetime = None
            if self._edr_autosave:
                self.save_edr_data()
            else:
                print("EDR data ready. Press 'J' to save or 'G' to clear.")

    def save_edr_data(self):
        if self.edr_enabled:
            self.edr.save_data(self.data_path)

    def clear_edr_event(self):
        if self.edr_enabled:
            self.edr.clear_event()

    def tick(self, clock):
        self.generate_vehicle_state_data()
        self.generate_perception_data()
        self.check_for_near_misses()
        self.check_edr_autosave()
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
        ]

        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

        if self.player is not None:
            self.player.destroy()

    def _is_vru(self, actor_type):
        # Vulnerable Road Users include cyclists and pedestrians
        return (
            actor_type == "vehicle.bh.crossbike"
            or actor_type == "vehicle.gazelle.omafiets"
            or actor_type == "vehicle.diamondback.century"
            or actor_type.startswith("walker.")
        )
