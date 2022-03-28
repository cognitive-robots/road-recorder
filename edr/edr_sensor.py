#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from .edr_buffer import EDRBuffer


# ==============================================================================
# -- EDRSensor -----------------------------------------------------------------
# ==============================================================================


class EDRSensor(object):
    """
    EDRSensor is a base class for all EDR sensors which will capture and
    store data for saving. Not to be used by itself.
    """

    def __init__(
        self,
        parent_actor,
        preevent_time,
        postevent_time,
        max_sample_rate,
        sensor_type,
        sensor_id="",
    ):
        self._parent = parent_actor
        self.sensor_type = sensor_type
        self.sensor_id = sensor_id
        self.edr_buffer = EDRBuffer(
            sensor_type, sensor_id, preevent_time, postevent_time, max_sample_rate
        )
        self.sensor = None
        self.ext = ""
        print(
            "created:",
            self.__class__.__name__,
            f"[{self.sensor_id}]" if self.sensor_id != "" else "",
        )

    def __del__(self):
        print(self.__class__.__name__, "destroyed")
        if self.sensor is not None:
            self.sensor.stop()

    def clear_event(self):
        self.edr_buffer.clear()

    def on_event_trigger(self, timestamp):
        self.edr_buffer.on_event_trigger(timestamp)

    def save_data(self, path):
        print(
            "Saving",
            self.__class__.__name__,
            "data",
            "for " + self.sensor_id if self.sensor_id != "" else "",
        )
        self.edr_buffer.save_data(path, self.ext)
