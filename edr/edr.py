#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import datetime
import os
import time


# ==============================================================================
# -- EDR -----------------------------------------------------------------------
# ==============================================================================


class EDR(object):
    """
    The Event Data Recorder (EDR) class handles the events themselves
    and notifies all the EDR sensors so they can maintain the correct
    buffer history and save to disk. Only one event may be active at
    a time.
    """

    def __init__(self):
        self.sensors = []
        self.triggered = False
        self.event_timestamp = None
        self.reason = ""

    def has_triggered(self):
        """
        Returns True if we are in a triggered state.
        """
        return self.triggered

    def add_sensor(self, sensor):
        """
        Adds an EDR sensor to the system.
        """
        self.sensors.append(sensor)

    def clear(self):
        """
        Clear all sensors and events from the EDR.
        """
        self.sensors.clear()
        self.clear_event()

    def clear_event(self):
        """
        Resets the EDR after an event, ready for a new event.
        """
        if not self.has_triggered():
            return

        print("Resetting EDR")
        for sensor in self.sensors:
            sensor.clear_event()
        self.event_timestamp = None
        self.triggered = False
        self.reason = ""

    def on_event_trigger(self, reason):
        """
        Called when an event occurs. The reason string is stored so
        it can be saved to disk for reference. Only one event may be
        active at a time.
        """
        if self.has_triggered():
            return

        print(f"EDR Triggered - {reason}!")
        self.triggered = True
        self.event_timestamp = time.time()
        self.reason = reason
        for sensor in self.sensors:
            sensor.on_event_trigger(self.event_timestamp)

    def save_data(self, base_path):
        """
        Saves the entire EDR state to disk and resets the EDR ready
        for the next event. A unique date-time sub-folder is created
        off the given base_path for storing all the state for the
        event.
        """
        dt = datetime.datetime.fromtimestamp(self.event_timestamp)
        path = os.path.join(base_path, dt.strftime("%Y-%m-%d-%H-%M-%S"))
        os.makedirs(path, exist_ok=True)
        print("Saving EDR data to:", path)
        reason_filepath = os.path.join(path, "reason.txt")
        with open(reason_filepath, "w") as reason_file:
            reason_file.write(self.reason)
        for sensor in self.sensors:
            sensor.save_data(path)
        print("EDR data saved")
        self.clear_event()
