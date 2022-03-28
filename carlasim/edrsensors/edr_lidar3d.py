#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
import weakref

from edr.edr_sensor import EDRSensor


# ==============================================================================
# -- EDRLidar3D ----------------------------------------------------------------
# ==============================================================================


class EDRLidar3D(EDRSensor):
    def __init__(
        self, parent_actor, preevent_time, postevent_time, sensor_id, transform
    ):
        super().__init__(
            parent_actor, preevent_time, postevent_time, 10.0, "lidar3d", sensor_id
        )
        self.ext = ".ply"

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
        bp.set_attribute("horizontal_fov", "360.0")
        bp.set_attribute("upper_fov", "22.5")
        bp.set_attribute("lower_fov", "-22.5")
        bp.set_attribute("channels", "64")
        bp.set_attribute("range", "100.0")
        bp.set_attribute("rotation_frequency", "20.0")
        bp.set_attribute("sensor_tick", "0.1")
        bp.set_attribute("points_per_second", "655360")  # 1024 pts x 64 ch x 10 Hz
        # bp.set_attribute('points_per_second', '327680')  # 512 pts x 64 ch x 10 Hz

        self.sensor = world.spawn_actor(bp, transform, attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: EDRLidar3D._on_data_event(weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        self = weak_self()
        if not self:
            return

        timestamp = time.time()
        self.edr_buffer.on_data(timestamp, event)
