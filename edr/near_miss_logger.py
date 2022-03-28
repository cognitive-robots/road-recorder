#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import pygame
import os
import time

from carlasim.core.utilities import *


MIN_EVENT_INTERVAL = 1.0


# ==============================================================================
# -- NearMissLogger ------------------------------------------------------------
# ==============================================================================


class NearMissLogger(object):
    """
    The NearMissLogger operates independently of the EDR logger. It is
    used to keep track of all near misses with Vulnerable Road Users
    (VRUs) irrespective of the EDR. The resulting CSV log file can be
    used to plot hotspots on a map.

    A single near miss log file can be shared among multiple runs of
    the simulator to build up a broader picture of events than could
    be captured in a single run.
    """

    def __init__(self, pathname, map_name, sound_file):
        self.log_file = None
        self.last_vru_id = -1
        self.last_event_time = time.time()
        if not os.path.exists(pathname):
            self._create_log_file(pathname, map_name)

        print("Opening Near Miss Log File:", pathname)
        self._check_log_file(pathname, map_name)
        self.log_file = open(pathname, mode="a")

        self.sound = None
        if sound_file is not None and sound_file != "":
            print("Near Miss Sound:", sound_file)
            self.sound = pygame.mixer.Sound(sound_file)
            pygame.mixer.Sound.play(self.sound)

    def __del__(self):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None

    def log_event(self, threshold, distance, vru, player):
        """
        Writes a new line entry directly to the CSV log file. An attempt
        is made to avoid repeated triggers relating to the same event
        with the same VRU.
        """
        now = time.time()
        interval = now - self.last_event_time
        if self.last_vru_id == vru.id and interval < MIN_EVENT_INTERVAL:
            # print('Skipping near miss event with:', vru.id)
            return

        self.last_event_time = now
        self.last_vru_id = vru.id
        if self.sound is not None:
            pygame.mixer.Sound.play(self.sound)

        vru_type = get_actor_type(vru)
        vru_speed = get_vector_norm(vru.get_velocity())
        vru_loc = vru.get_location()
        player_speed = get_vector_norm(player.get_velocity())
        player_loc = player.get_location()
        line = f"{vru_type}, {threshold}, {distance}, {vru_speed}, {vru_loc.x}, {vru_loc.y}, {vru_loc.z}, {player_speed}, {player_loc.x}, {player_loc.y}, {player_loc.z}\n"
        # print('Near Miss:', line)
        self.log_file.write(line)

    def _get_base_map_name(self, map_name):
        """
        Removes any trailing '_Opt' from the town map name since the
        two are essentially the same map but with different layer
        optimizations, so are interchangeable.
        """
        return map_name.split("_")[0]

    def _create_log_file(self, pathname, map_name):
        """
        The first time a CSV log file is created, its header rows must
        be inserted. The first line is the map name and the second is
        the field list.
        """
        print("Creating Near Miss Log File:", pathname)
        base_map_name = self._get_base_map_name(map_name)
        folder = os.path.dirname(pathname)
        os.makedirs(folder, exist_ok=True)
        with open(pathname, mode="w") as file:
            file.write(f"{base_map_name}\n")
            file.write(
                "# VRU Type, Threshold, Distance, VRU Speed, VRU X, VRU Y, VRU Z, EGO Speed, EGO X, EGO Y, EGO Z\n"
            )

    def _check_log_file(self, pathname, map_name):
        """
        Throws an exception if we're attempting to use a log file for
        a different map. Locations only make sense for the same map.
        """
        base_map_name = self._get_base_map_name(map_name)
        with open(pathname, mode="r") as file:
            file_map_name = file.readline().split(",")[0].strip()
            if base_map_name != file_map_name:
                raise ValueError(
                    "Map names don't match in NearMissLogger",
                    file_map_name,
                    base_map_name,
                )
