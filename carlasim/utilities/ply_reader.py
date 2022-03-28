#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import os

import numpy as np
import open3d as o3d


# ==============================================================================
# -- PLYReader -----------------------------------------------------------------
# ==============================================================================


class PLYReader(object):
    """Reads points from an individual PLY file as an numpy array"""

    def __init__(self, pathname):
        try:
            self.pcd = o3d.io.read_point_cloud(pathname)
        except RuntimeError as error:
            print("Error opening PLY file: {}".format(error))
            self.pcd = None

    def get_points(self):
        return np.asarray(self.pcd.points)


# ==============================================================================
# -- PLYManager ----------------------------------------------------------------
# ==============================================================================


class PLYManager(object):
    """
    Each PLY file represents a snapshot in time from a LiDAR scan.
    These are not synchronised with the other sensors and perception
    data. The PLYManager finds the closest PLY file matching the
    timestamp from another file and returns its points.
    """

    def __init__(self, folder, max_delta=0.25):
        """
        Builds a timestamp map of all PLY files in a folder
        """
        self.max_delta = max_delta
        self.files = []
        if folder == "":
            return

        ply_dir = os.path.dirname(folder)
        for filename in os.listdir(ply_dir):
            if filename.endswith(".ply"):
                timestamp = self.extract_timestamp(filename)
                pathname = os.path.join(ply_dir, filename)
                self.files.append({"timestamp": timestamp, "pathname": pathname})

    def extract_timestamp(self, filename):
        """
        The whole scheme assumes the following file naming convention@:
                <content-type>_<timestamp>_<time-offset>.<extension>

        This function extracts the <timestamp> part of the filename.
        """
        return float(filename.split("_")[1])

    def get_points(self, related_filename):
        """
        Returns the point from the closest timestamp matching PLY file.
        Returns None if no file is within 'max_delta' seconds.
        """
        best_delta = 0.0
        best_file = None

        related_timestamp = self.extract_timestamp(related_filename)
        for file in self.files:
            delta = math.fabs(related_timestamp - file["timestamp"])
            if delta <= self.max_delta and (best_file is None or delta < best_delta):
                best_file = file
                best_delta = delta

        if best_file is None:
            return None

        reader = PLYReader(best_file["pathname"])
        return reader.get_points()
