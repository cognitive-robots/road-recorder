#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import collections
import os


# ==============================================================================
# -- EDRBuffer -----------------------------------------------------------------
# ==============================================================================


class EDRBuffer(object):
    """
    The EDRBuffer actually contains two buffers:

    1. A pre-event circular buffer which continuously updates with the
       most recent data prior to the event.

    2. A post-event buffer which fills once the event has occurred.

    Note that the size of the circular buffer is determined by the
    pre-event duration and the expected sample rate. Since the actual
    sample rate can vary, the timespan of samples in the buffer may
    be more or less than the desired pre-event time.

    Each EDRSensor has its own EDRBuffer.

    Each item in each of the buffers is a tuple containing the timestamp
    and the data itself.
    """

    def __init__(
        self, sensor_type, sensor_id, preevent_time, postevent_time, max_sample_rate
    ):
        preevent_samples = int(preevent_time * max_sample_rate)

        self.sensor_type = sensor_type
        self.sensor_id = sensor_id

        self.preevent_time = preevent_time
        self.postevent_time = postevent_time
        self.max_sample_rate = max_sample_rate
        self.sample_interval = 1.0 if max_sample_rate <= 0.0 else 1.0 / max_sample_rate

        self.preevent_buffer = collections.deque(maxlen=preevent_samples)
        self.postevent_buffer = collections.deque()
        self.event_timestamp = None
        self.start_timestamp = None
        self.end_timestamp = None
        self.next_timestamp = None
        self.saving = False

    def clear(self):
        """
        Clear all data from the buffer ready for a new event.
        """
        self.preevent_buffer.clear()
        self.postevent_buffer.clear()
        self.event_timestamp = None
        self.start_timestamp = None
        self.end_timestamp = None
        self.next_timestamp = None

    def on_data(self, timestamp, data):
        """
        Called when new data arrives from the parent sensor for storing
        in the appropriate buffer according to the current state.
        """
        if self.saving:
            # Prevent buffer mutations
            return

        if self.end_timestamp is not None and timestamp > self.end_timestamp:
            # Event storage has finished
            return

        if self.next_timestamp is not None and timestamp < self.next_timestamp:
            # Too soon - exceeding max_sample_rate
            return

        self.next_timestamp = timestamp + self.sample_interval

        item = (timestamp, data)
        if self.event_timestamp is None:
            # Event hasn't happened yet so add to circular buffer
            self.preevent_buffer.append(item)
        else:
            # Store data in the post-event buffer
            self.postevent_buffer.append(item)

    def on_event_trigger(self, timestamp):
        """
        Called when an event triggers to lock the pre-event buffer and
        start filling the post-event buffer. Assumes that a higher
        power is responsible for correct re-trigger prevention.
        """
        assert self.event_timestamp is None
        self.event_timestamp = timestamp
        self.start_timestamp = timestamp - self.preevent_time
        self.end_timestamp = timestamp + self.postevent_time

    def save_data(self, base_path, ext):
        """
        Saves all the buffered data to disk in a new subfolder named
        according to the parent sensor type and id, as provided.
        """
        self.saving = True
        path = os.path.join(base_path, self.sensor_type, self.sensor_id)
        os.makedirs(path, exist_ok=True)

        # Save data before the event
        for item in self.preevent_buffer:
            if item[0] >= self.start_timestamp:
                self._save_data_item(item, path, ext)

        # Save data after the event
        for item in self.postevent_buffer:
            if item[0] <= self.end_timestamp:
                self._save_data_item(item, path, ext)

        self.saving = False

    def _save_data_item(self, item, path, ext):
        """
        Saves an individual item to disk using a filename following
        this general scheme:
           <sensor-id>_<timestamp>_<offset>.<ext>

        The offset represents the amount of time from the event itself,
        preceded by a +/- indicator.

        Data objects are expected to be able to handle actually saving
        themselves to disk.
        """
        offset = item[0] - self.event_timestamp
        filename = self.sensor_id + "_" if self.sensor_id != "" else ""
        filename += "{:19.8f}".format(item[0]) + "_" + "{:+010.8f}".format(offset) + ext
        frame_path = os.path.join(path, filename)
        item[1].save_to_disk(frame_path)
