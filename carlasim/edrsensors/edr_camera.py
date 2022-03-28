#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
import weakref

from edr.edr_sensor import EDRSensor


# ==============================================================================
# -- EDRCamera -----------------------------------------------------------------
# ==============================================================================


class EDRCamera(EDRSensor):
    def __init__(
        self,
        parent_actor,
        preevent_time,
        postevent_time,
        sensor_id,
        transform,
        width=720,
        height=480,
        fov=110,
        rate=10.0,
    ):
        super().__init__(
            parent_actor, preevent_time, postevent_time, rate, "images", sensor_id
        )
        self.ext = ".png"

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(width))
        bp.set_attribute("image_size_y", str(height))
        bp.set_attribute("fov", str(fov))
        bp.set_attribute("sensor_tick", str(1.0 / rate))

        self.sensor = world.spawn_actor(bp, transform, attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: EDRCamera._on_data_event(weak_self, event))

    @staticmethod
    def _on_data_event(weak_self, event):
        self = weak_self()
        if not self:
            return

        timestamp = time.time()
        self.edr_buffer.on_data(timestamp, event)
