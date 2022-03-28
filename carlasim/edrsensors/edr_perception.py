#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import json
import time

import carla

from edr.edr_sensor import EDRSensor
from ..core.utilities import *

MAX_SAMPLE_RATE_HZ = 20.0
# SELF_DIST_THRESHOLD = 0.2


# ==============================================================================
# -- _EDRPerceptionData ---------------------------------------- PRIVATE -----
# ==============================================================================


class _EDRPerceptionData(object):
    """
    Since CARLA doesn't provide a native "perception" sensor in the same
    way it provides cameras and lidars, this simple class holds such
    data and knows how to write it to disk as a JSON file.
    """

    def __init__(self, perception_data):
        # perception_data is a dictionary object containing:
        #   timestamp:
        #   ego_vehicle:
        #   detections: []
        self.perception_data = perception_data

    def save_to_disk(self, frame_path):
        with open(frame_path, "w") as write_file:
            json.dump(self.perception_data, write_file, indent=4)


# ==============================================================================
# -- EDRPerceptionSensor -----------------------------------------------------
# ==============================================================================


class EDRPerceptionSensor(EDRSensor):
    """
    Unlike cameras and lidars, where CARLA has a native sensor and data
    type, we can't create an automatic callback. So this sensor relies
    on being called explicitly whenever it needs to generate data.
    """

    def __init__(
        self, parent_actor, preevent_time, postevent_time, world, perception_range
    ):
        super().__init__(
            parent_actor, preevent_time, postevent_time, 100.0, "perception", "logs"
        )
        self.ext = ".json"
        self.world = world
        self.perception_range = perception_range
        self.next_timestamp = 0

    @staticmethod
    def _get_vector_data(vector):
        return {"x": vector.x, "y": vector.y, "z": vector.z}

    @staticmethod
    def _get_rotation_data(rotation):
        return {"yaw": rotation.yaw, "pitch": rotation.pitch, "roll": rotation.roll}

    # @staticmethod
    # def _get_vertices_data(vertices):
    #     vertices_data = []
    #     for vertex in vertices:
    #         vertices_data.append(EDRPerceptionSensor._get_vector_data(vertex))
    #
    #     return vertices_data

    @staticmethod
    def _get_bb_data(bb, world_transform=None, actor_type="car"):
        # if world_transform is None:
        #     world_transform = carla.Transform(carla.Location(0,0,0), carla.Rotation(0,0,0))

        # There seems to be a CARLA bug whereby bicycles return bad bounding boxes,
        # so use a plausible fixed bounding box instead
        if actor_type == "bicycle":
            return {
                "extent": {
                    "x": 0.9177202582359314,
                    "y": 0.16446444392204285,
                    "z": 0.8786712288856506,
                }
            }

        return {
            "extent": EDRPerceptionSensor._get_vector_data(bb.extent),
            # 'location': EDRPerceptionSensor._get_vector_data(bb.location),
            # 'rotation': EDRPerceptionSensor._get_rotation_data(bb.rotation),
            # 'vertices': EDRPerceptionSensor._get_vertices_data(bb.get_world_vertices(world_transform))
        }

    def get_detections(self, player_id, player_transform, all_actors, filter):
        """
        Returns a list of all actors from the given list and of the
        given filter type that are within preception range of the
        player actor.

        Each returned list entry is an object containing bounding box
        and other useful information about the perceived actor.

        NOTE: This is ground-truth information from CARLA's inherent
        knowledge of the world and not related in any way to information
        gathered via any sensor perception algorithms.
        """
        detections = []
        actors = all_actors.filter(filter)
        for actor in actors:
            actor_transform = actor.get_transform()
            rel_loc = actor_transform.location - player_transform.location
            dist = get_vector_norm(rel_loc)
            if actor.id != player_id and dist <= self.perception_range:
                rel_transform = relative_transform(player_transform, actor_transform)
                velocity = get_local_vector(actor_transform, actor.get_velocity())

                # print(f'DEBUG: rel_trans: {rel_transform}, distance: {dist}')
                actor_type = get_actor_type(actor)
                data = {
                    "id": actor.id,
                    "type": actor_type,
                    "bounding_box": EDRPerceptionSensor._get_bb_data(
                        actor.bounding_box, rel_transform, actor_type
                    ),
                    "proximity_threshold": get_proximity_threshold(actor_type),
                    "velocity": EDRPerceptionSensor._get_vector_data(velocity),
                    "relative_location": EDRPerceptionSensor._get_vector_data(
                        rel_transform.location
                    ),
                    "relative_rotation": EDRPerceptionSensor._get_rotation_data(
                        rel_transform.rotation
                    ),
                    # "location": EDRPerceptionSensor._get_vector_data(actor_transform.location),
                    # "rotation": EDRPerceptionSensor._get_rotation_data(actor_transform.rotation),
                }
                detections.append(data)

        # if len(detections) > 0:
        #     print(f'Found {len(detections)} detection(s) for {filter}')

        return detections

    def generate_data(self, player):
        """
        Generates an EDR "perception" data entry throttled to a
        reasonable maximum sample rate.
        """
        timestamp = time.time()
        if timestamp < self.next_timestamp:
            return

        # Limit samlpe rate
        self.next_timestamp = timestamp + 1.0 / MAX_SAMPLE_RATE_HZ

        # actors = world.get_actors()
        # vehicles = actors.filter()

        # TODO:
        #   world.get_level_bbs(carla.Vehicles)
        #   world.get_level_bbs(carla.Pedestrians)
        #   world.get_level_bbs(carla.Poles)
        #   world.get_level_bbs(carla.TrafficSigns)
        #   world.get_level_bbs(carla.TrafficLight)
        #   world.get_level_bbs(carla.Other) ?

        player_transform = player.get_transform()
        player_bb = player.bounding_box
        player_location = player_bb.location
        player_velocity = get_local_vector(player_transform, player.get_velocity())

        detections = []
        actors = self.world.get_actors()

        # NOTE: actors do NOT include static objects (e.g. vehicles) which are part of the map
        #       whereas world.get_level_bbs() does, but has no actor reference

        detections.extend(
            self.get_detections(player.id, player_transform, actors, "vehicle.*")
        )
        detections.extend(
            self.get_detections(player.id, player_transform, actors, "walker.*")
        )

        perception_data = {
            "timestamp": timestamp,
            "ego_vehicle": {
                "velocity": EDRPerceptionSensor._get_vector_data(player_velocity),
                # "location": EDRPerceptionSensor._get_vector_data(player_transform.location),
                # "rotation": EDRPerceptionSensor._get_rotation_data(player_transform.rotation),
                "bounding_box": EDRPerceptionSensor._get_bb_data(player_bb),
            },
            "detections": detections,
        }
        self.on_data(timestamp, perception_data)

    def on_data(self, timestamp, perception_data):
        """
        Sends perception data to the EDR buffer.
        """
        data = _EDRPerceptionData(perception_data)
        self.edr_buffer.on_data(timestamp, data)
