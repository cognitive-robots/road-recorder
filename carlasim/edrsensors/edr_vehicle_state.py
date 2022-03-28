#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import datetime
import os
import time

from ..core.utilities import get_local_vector
from ..core.utilities import get_vector_norm

from edr.edr_sensor import EDRSensor

from . import edr_states


# ==============================================================================
# -- _EDRVehicleStateFile ---------------------------------------- PRIVATE -----
# ==============================================================================


class _EDRVehicleStateFile(object):
    """
    Represents a write-only CSV file for storing a time series of
    vehicle state information.
    """

    def __init__(self, filename, fields):
        self.filename = filename
        self.fields = fields  # array object
        self.file = None
        self.event_timestamp = None

    def open(self, path, event_timestamp):
        os.makedirs(path, exist_ok=True)
        filepath = os.path.join(path, self.filename)
        self.event_timestamp = event_timestamp
        self.file = open(filepath, "wt")
        self.file.write("Date-Time,Timestamp,Offset,Event Trigger")
        for field in self.fields:
            self.file.write(f",{field}")
            units = edr_states.STATE_UNITS.get(field, "")
            if units != "":
                self.file.write(f" ({units})")
        self.file.write("\n")

    def close(self):
        self.file.close()

    def write(self, timestamp, states):
        offset = timestamp - self.event_timestamp
        dt = datetime.datetime.fromtimestamp(timestamp)
        self.file.write(dt.strftime("%Y-%m-%d-%H-%M-%S.%f"))
        self.file.write(",")
        self.file.write(str(timestamp))
        self.file.write(",")
        self.file.write(str(offset))
        self.file.write(",")
        self.file.write("0" if offset < 0.0 else "100")
        for field in self.fields:
            self.file.write(",")
            self.file.write(str(states.get(field, "")))
        self.file.write("\n")


# ==============================================================================
# -- _EDRVehicleStateData ---------------------------------------- PRIVATE -----
# ==============================================================================


class _EDRVehicleStateData(object):
    """
    Internal storage of vehicle state information for a single time
    frame (analogous to a single camera image).
    """

    def __init__(self, file, timestamp, states):
        self.file = file  # _EDRVehicleStateFile
        self.timestamp = timestamp  # time.time()
        self.states = states  # dictionary object

    def save_to_disk(self, frame_path):
        # Ignore frame_path as file is already open for writing
        self.file.write(self.timestamp, self.states)


# ==============================================================================
# -- EDRVehicleStateSensor -----------------------------------------------------
# ==============================================================================


class EDRVehicleStateSensor(EDRSensor):
    """
    An EDR sensor that captures the kind of vehicle state information
    that might be obtained from the vehicle CAN bus and GPS sensor,
    including speed, acceleration and location, etc.
    """

    def __init__(self, parent_actor, preevent_time, postevent_time):
        super().__init__(
            parent_actor, preevent_time, postevent_time, 100.0, "vehicle-state"
        )
        self.file = _EDRVehicleStateFile("vehicle_state.csv", edr_states.ALL_STATES)

    def generate_data(self, player, gnss_sensor):
        """
        Extracts vehicle state information from the player actor state
        and sends it to the buffer. There's no sample rate throttling
        so we'll save every sample we can get.
        """
        t = player.get_transform()
        a = player.get_acceleration()
        v = player.get_velocity() * 3.6  # m/s => km/h
        c = player.get_control()
        la = get_local_vector(t, a)
        lv = get_local_vector(t, v)
        speed = get_vector_norm(v)
        states = {
            edr_states.SPEED: speed,
            edr_states.LONGITUDINAL_VELOCITY: lv.x,
            edr_states.LATERAL_VELOCITY: lv.y,
            edr_states.NORMAL_VELOCITY: lv.z,
            edr_states.LONGITUDINAL_ACCELERATION: la.x,
            edr_states.LATERAL_ACCELERATION: la.y,
            edr_states.NORMAL_ACCELERATION: la.z,
            edr_states.ACCELERATOR_PERCENT: c.throttle * 100.0,
            edr_states.BRAKE_PERCENT: c.brake * 100.0,
            edr_states.SERVICE_BRAKE: c.hand_brake,  # c.brake > 0.0?
            edr_states.STEERING_INPUT_PERCENT: c.steer * 100.0,
            edr_states.GNSS_LATITUDE: gnss_sensor.lat,
            edr_states.GNSS_LONGITUDE: gnss_sensor.lon,
            edr_states.GNSS_ALTITUDE: gnss_sensor.alt,
        }
        self.on_data(states)

    def on_data(self, states):
        """
        Sends vehicle state data to the EDR buffer.
        """
        timestamp = time.time()
        data = _EDRVehicleStateData(self.file, timestamp, states)
        self.edr_buffer.on_data(timestamp, data)

    # Override
    def save_data(self, path):
        """
        A regular EDR sensor, such as a camera or lidar, will write each
        frame to a separate file. In this case, we override the save
        operation to open a single CSV file for each custom data item
        to append instead.
        """
        data_path = os.path.join(path, self.sensor_type)
        self.file.open(data_path, self.edr_buffer.event_timestamp)
        super().save_data(path)  # path gets ignored here
        self.file.close()
