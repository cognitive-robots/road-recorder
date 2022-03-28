#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

ACCELERATOR_PERCENT = "Accelerator Pedal"
BRAKE_PERCENT = "Brake Pedal"
DELTA_V_LATERAL = "Delta-V Lateral"
DELTA_V_LONGITUDINAL = "Delta-V Longitudinal"
ENGINE_RPM = "Engine RPM"
GNSS_ALTITUDE = "Altitude"
GNSS_LATITUDE = "Latitude"
GNSS_LONGITUDE = "Longitude"
LATERAL_ACCELERATION = "Lateral Acceleration"
LATERAL_VELOCITY = "Lateral Velocity"
LONGITUDINAL_ACCELERATION = "Longitudinal Acceleration"
LONGITUDINAL_VELOCITY = "Longitudinal Velocity"
NORMAL_ACCELERATION = "Normal Acceleration"
NORMAL_VELOCITY = "Normal Velocity"
SERVICE_BRAKE = "Service Brake"
SPEED = "Speed"
STEERING_INPUT_PERCENT = "Steering Input"

STATE_UNITS = {
    ACCELERATOR_PERCENT: "%",
    BRAKE_PERCENT: "%",
    DELTA_V_LATERAL: "m/s",
    DELTA_V_LONGITUDINAL: "m/s",
    ENGINE_RPM: "rpm",
    GNSS_ALTITUDE: "m",
    GNSS_LATITUDE: "",
    GNSS_LONGITUDE: "",
    LATERAL_ACCELERATION: "m/s^2",
    LATERAL_VELOCITY: "km/h",
    LONGITUDINAL_ACCELERATION: "m/s^2",
    LONGITUDINAL_VELOCITY: "km/h",
    NORMAL_ACCELERATION: "m/s^2",
    NORMAL_VELOCITY: "km/h",
    SERVICE_BRAKE: "",
    SPEED: "km/h",
    STEERING_INPUT_PERCENT: "%",
}

ALL_STATES = [
    ACCELERATOR_PERCENT,
    BRAKE_PERCENT,
    DELTA_V_LATERAL,
    DELTA_V_LONGITUDINAL,
    ENGINE_RPM,
    GNSS_ALTITUDE,
    GNSS_LATITUDE,
    GNSS_LONGITUDE,
    LATERAL_ACCELERATION,
    LATERAL_VELOCITY,
    LONGITUDINAL_ACCELERATION,
    LONGITUDINAL_VELOCITY,
    NORMAL_ACCELERATION,
    NORMAL_VELOCITY,
    SERVICE_BRAKE,
    SPEED,
    STEERING_INPUT_PERCENT,
]
