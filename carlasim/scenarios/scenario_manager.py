#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import json
import math
import random
import time

import carla

from dataclasses import dataclass


# ==============================================================================
# -- PRESETS -------------------------------------------------------------------
# ==============================================================================

SUN_PRESETS = {
    # (altitude{-90..90}, azimuth{0..360})
    "day": (60.0, 0.0),
    "night": (-90.0, 0.0),
    "sunrise": (0.5, 0.0),
    "sunset": (0.5, 180.0),
}

WEATHER_PRESETS = {
    # [cloudiness{0..100}, rain{0..100}, puddles{0..100}, wind{0..100},
    #  fog_density{0..100}, fog_distance{0m..inf}, fog_falloff{0..5+},
    #  wetness{0..100}]
    "clear": [10.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.2, 0.0],
    "fog": [20.0, 0.0, 0.0, 0.0, 70.0, 0.0, 3.0, 5.0],
    "overcast": [80.0, 0.0, 0.0, 50.0, 2.0, 0.0, 0.9, 10.0],
    "rain": [70.0, 100.0, 20.0, 0.0, 0.0, 0.0, 0.2, 20.0],
    "storm": [100.0, 100.0, 90.0, 100.0, 20.0, 0.0, 0.9, 100.0],
}


# ==============================================================================
# -- ActorDefinition -----------------------------------------------------------
# ==============================================================================


@dataclass
class ActorDefinition:
    """
    Defines an actor (vehicle) to add to the scene.
    If spawn_index is negative, the spawn_x/y/z/yaw are used.
    """

    label: str
    bp_index: int
    spawn_index: int
    spawn_x: float
    spawn_y: float
    spawn_z: float
    spawn_yaw: float


# ==============================================================================
# -- ScenarioTrigger -----------------------------------------------------------
# ==============================================================================


@dataclass
class ScenarioTrigger:
    """
    Defines a velocity to apply to an actor when the player reaches the
    vicinity of the specified trigger location.
    """

    x: float
    y: float
    radius: float
    action_velocity: carla.Vector3D


# ==============================================================================
# -- TriggeredActorDefinition --------------------------------------------------
# ==============================================================================


@dataclass
class TriggeredActorDefinition:
    """
    A triggered actor (vehicle) as defined in a scenarion file.
    """

    actor_def: ActorDefinition
    trigger: ScenarioTrigger


# ==============================================================================
# -- TriggeredActor -----------------------------------------------------------
# ==============================================================================


@dataclass
class TriggeredActor:
    """
    Internal representation of a triggered actor (vehicle) once loaded.
    """

    label: str
    actor: carla.Actor
    trigger: ScenarioTrigger
    triggered: bool


# ==============================================================================
# -- Scenario ------------------------------------------------------------------
# ==============================================================================


class Scenario(object):
    """
    Scenarios hold all the information to set up a scene, including the
    map, environmental conditions and actors (vehicles) to place in it.

    Scenarios are normally loaded from a JSON definition but it is also
    possible to create a specialised subclass which hard codes aspects
    of the scenario.

    The JSON file has the following general structure (see README.md >
    MANUAL_DRIVING.md for full explanations):

        {
            "label": "<scenario-name>",
            "description": "<scenario-description>",
            "map": "<CARLA-town-name>",
            "environment": <CARLA-weather-preset>
            "hour": <decimal-hour-of-day>,
            "spawn_index": <driver-spawn-point-index>,
            "cctv_viewpoint": {
                "x": <focus-location-x>,
                "y": <focus-location-y>,
                "z": <height-offset>,
                "distance": <distance-of-camera-from-focus-location>,
                "pitch": <camera-pitch-angle-degrees>,
                "yaw": <camera-yaw-angle-degrees>
            },
            "managed_actor_bp_indexes": [<list-of-actor-blueprint-indexes>],
            "managed_actor_spawn_indexes": [<list-of-actor-spawn-indexes>]
            "triggered_actor_definitions": [
                {
                    "actor": {
                        "label": "<actor-label>",
                        "bp_index": <blueprint-index>,
                        "spawn_location": {
                            "x": <location-x>,
                            "y": <location-y>,
                            "z": <location-z>,
                            "yaw": <rotation-angle-degrees>
                        }
                    },
                    "trigger": {
                        "x": <driver-location-x>,
                        "y": <driver-location-y>,
                        "radius": <trigger-radius>,
                        "action_velocity": {
                            "x": <actor-velocity-x>,
                            "y": <actor-velocity-y>,
                            "z": <actor-velocity-z>
                        }
                    }
                },
                {
                    <next-triggered-actor-definition>
                }
            ]
        }
    """

    def __init__(self):
        self.label = ""
        self.description = ""
        self.map = None
        self.environment_preset = None
        self.hour = None
        self.sun_preset = None
        self.weather_preset = None
        self.cctv_viewpoint = None
        self.player_spawn_index = -1
        self.managed_actor_bp_indexes = []
        self.managed_actor_spawn_indexes = []
        self.triggered_actor_definitions = []
        self.triggered_actors = []

    def load(self, json_pathname):
        """
        Loads a  scenario from a JSON definition file.
        """
        try:
            with open(json_pathname) as json_file:
                data = json.load(json_file)
                self.label = data.get("label", "")
                self.description = data.get("description", "")
                self.map = data.get("map")
                self.environment_preset = data.get("environment")
                self.hour = data.get("hour")
                self.sun_preset = data.get("sun_preset")
                self.weather_preset = data.get("weather_preset")
                self.cctv_viewpoint = data.get("cctv_viewpoint")
                self.player_spawn_index = data.get("spawn_index", -1)
                self.managed_actor_bp_indexes = data.get("managed_actor_bp_indexes", [])
                self.managed_actor_spawn_indexes = data.get(
                    "managed_actor_spawn_indexes", []
                )
                triggered_actor_defs = data.get("triggered_actor_definitions")
                if triggered_actor_defs is not None and len(triggered_actor_defs) > 0:
                    for tad in triggered_actor_defs:
                        spawn_index = -1
                        spawn_x = 0.0
                        spawn_y = 0.0
                        spawn_z = 0.0
                        spawn_yaw = 0.0

                        ad = tad["actor"]
                        label = ad["label"]
                        bp_index = ad["bp_index"]
                        loc = ad.get("spawn_location")
                        if loc is None:
                            spawn_index = ad["spawn_index"]
                        else:
                            spawn_x = loc["x"]
                            spawn_y = loc["y"]
                            spawn_z = loc["z"]
                            spawn_yaw = loc["yaw"]

                        actor_def = ActorDefinition(
                            label,
                            bp_index,
                            spawn_index,
                            spawn_x,
                            spawn_y,
                            spawn_z,
                            spawn_yaw,
                        )

                        trig = tad["trigger"]
                        tx = trig["x"]
                        ty = trig["y"]
                        tr = trig["radius"]
                        vel = trig["action_velocity"]
                        action_velocity = carla.Vector3D(vel["x"], vel["y"], vel["z"])

                        scenario_trigger = ScenarioTrigger(tx, ty, tr, action_velocity)

                        self.triggered_actor_definitions.append(
                            TriggeredActorDefinition(actor_def, scenario_trigger)
                        )

            return True

        except:
            return False

    def get_map(self):
        """
        Returns the name of the CARLA map to load.
        At the time of writing, this should be one of the following:
            Town01, Town01_Opt, Town02, Town02_Opt, Town03, Town03_Opt,
            Town04, Town04_Opt, Town05, Town05_Opt, Town06, Town06_Opt,
            Town07, Town07_Opt, Town10HD, Town10HD_Opt
        """
        return self.map

    def get_environment_preset(self):
        """
        Returns the name of the CARLA environment preset to use.
        At the time of writing, this should be one of the following:
            ClearNoon, ClearSunset, CloudyNoon, CloudySunset, Default,
            HardRainNoon, HardRainSunset, MidRainSunset, MidRainyNoon,
            SoftRainNoon, SoftRainSunset, WetCloudyNoon, WetCloudySunset,
            WetNoon, WetSunset
        """
        return self.environment_preset

    def get_sun_values(self):
        """
        Returns the set of environmental values related to sun position.
        This can be based on one of the SUN_PRESETS defined above, or
        the time of day as a fractional hour value (e.g. 8.5 = 08:30).
        """
        values = None
        if self.sun_preset is not None:
            print(f"Sun preset: {self.sun_preset}")
            values = SUN_PRESETS.get(self.sun_preset)

        if values is None and self.hour is not None:
            hour = self.hour % 24
            # altitude = (70 * math.sin(hour * math.pi / 12.0)) - 20
            altitude = (-70.0 * math.cos(hour * math.pi / 12.0)) - 20.0
            azimuth = hour * 15.0
            values = (altitude, azimuth)
            print(f"Time of day: {int(hour)}:{int(60 * (hour % 1.0)):02d}")

        return values

    def get_weather_preset(self):
        """
        Returns the name of the preset in use from WEATHER_PRESETS,
        defined above. Use of these presets is not recommended as
        they're not particularly effective and need more work!
        """
        return self.weather_preset

    def get_cctv_viewpoint(self):
        """
        Returns an object defining the position and orientation of a
        "CCTV" camera in terms of a focus location, a distance from
        that location and pitch and yaw. This is used to set the main
        CARLA simulation server viewpoint and is useful for creating
        demonstration videos containing a combination of the 1st-person
        player view and the CCTV view of a scene event.
        """
        return self.cctv_viewpoint

    def get_player_spawn_index(self):
        """
        Returns the index of the spawn location where the player starts.
        """
        return self.player_spawn_index

    def setup_actors(self, sm):
        """
        Uses the given ScenarioManager to set up all the actors
        (vehicles) in the scenario.
        """
        if len(self.managed_actor_bp_indexes) > 0:
            sm.spawn_sequenced_vehicles_at(
                self.managed_actor_bp_indexes, self.managed_actor_spawn_indexes
            )

        for tad in self.triggered_actor_definitions:
            actor = None
            ad = tad.actor_def
            if ad.spawn_index < 0:
                actor = sm.spawn_vehicle_at_location(
                    ad.bp_index,
                    carla.Transform(
                        carla.Location(ad.spawn_x, ad.spawn_y, ad.spawn_z),
                        carla.Rotation(yaw=ad.spawn_yaw),
                    ),
                    False,
                )
            else:
                actor = sm.spawn_vehicle_at(ad.bp_index, ad.spawn_index, False)

            actor.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.triggered_actors.append(
                TriggeredActor(ad.label, actor, tad.trigger, False)
            )

    def get_actors(self):
        """
        Returns a list of all the (triggered) actors which are not managed
        automatically by CARLA.
        """
        actors = []
        for ta in self.triggered_actors:
            actors.append(ta.actor)
        return actors

    def tick(self, player_transform):
        """
        Checks whether to trigger actors at each simulation time step.
        """
        for ta in self.triggered_actors:
            if not ta.triggered:
                dx = player_transform.location.x - ta.trigger.x
                dy = player_transform.location.y - ta.trigger.y
                r2 = ta.trigger.radius**2
                if dx**2 + dy**2 < r2:
                    print("Triggered:", ta.label)
                    ta.triggered = True
                    ta.actor.set_target_velocity(ta.trigger.action_velocity)


# ==============================================================================
# -- ScenarioManager -----------------------------------------------------------
# ==============================================================================


class ScenarioManager(object):
    """
    The ScenarioManager handles the running of a Scenario and
    interfacing with the CARLA server. The class split is partially
    one of historical development and they could feasibly be
    combined into one simpler class in future.
    """

    def __init__(self, client, tm_port, scenario, seed=0):
        rseed = seed if seed > 0 else int(time.time())
        print("Using random seed:", rseed)
        random.seed(rseed)

        self.actor_list = []
        self.scenario = scenario

        print("Running Scenario:", self.scenario.label)
        if self.scenario.description is not None and self.scenario.description != "":
            print(f'"{self.scenario.description}"')

        self.client = client

        scenario_map = self.scenario.get_map()
        if scenario_map is not None:
            print("Loading map:", scenario_map)
            self.client.load_world(scenario_map)

        self.world = self.client.get_world()
        self.world_map = self.world.get_map()
        self.spawn_points = self.world_map.get_spawn_points()

        # self.tm_port = args.tm_port
        self.tm_port = tm_port
        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.traffic_manager.set_random_device_seed(rseed)
        self.traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        self.traffic_manager.global_percentage_speed_difference(0.0)

        blueprint_library = self.world.get_blueprint_library()
        self.vehicle_blueprints = blueprint_library.filter("vehicle")

        env = self.scenario.get_environment_preset()
        print("Environment preset:", env)
        weather = (
            self.world.get_weather()
            if env is None
            else getattr(carla.WeatherParameters, env)
        )
        self._apply_sun_presets(self.scenario.get_sun_values(), weather)
        self._apply_weather_presets(self.scenario.get_weather_preset(), weather)
        print("Setting weather values:", weather)
        self.world.set_weather(weather)

    def on_shutdown(self):
        """
        Called when the application closes to perform cleanup tasks.
        """
        print("Destroying actors")
        unmanaged_actors = self.scenario.get_actors()
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in unmanaged_actors]
        )
        self.client.apply_batch(
            [carla.command.DestroyActor(x) for x in self.actor_list]
        )

    def tick(self, player_transform):
        """
        Called at each simulation time step.
        """
        if player_transform is not None:
            self.scenario.tick(player_transform)
        # self.world.wait_for_tick()

    def get_vehicle_blueprint_count(self):
        """
        Returns the number of "vehicle" actor blueprints available.
        """
        return len(self.vehicle_blueprints)

    def get_spawn_point_count(self):
        """
        Returns the number of "vehicle" actor spawn points available.
        """
        return len(self.spawn_points)

    def spawn_vehicle_at_location(self, bp_index, transform, manage=True):
        """
        Creates an actor in the scene at the given location.
        If manage is True, the ScenarioManager will look after its
        destruction and automation state.
        """
        bp = self._get_actor_blueprint(bp_index)
        vehicle = self.world.try_spawn_actor(bp, transform)
        if vehicle is None:
            print(f"Failed to create vehicle at {transform}")
        else:
            print(f"Created {vehicle.type_id} at {transform}")
            if manage:
                self.actor_list.append(vehicle)

        return vehicle

    def spawn_vehicle_at(self, bp_index, spawn_index, manage=True):
        """
        Creates an actor in the scene at the given spawn point.
        If manage is True, the ScenarioManager will look after its
        destruction and automation state.
        """
        spawn_count = self.get_spawn_point_count()
        if spawn_index < 0 or spawn_index >= spawn_count:
            spawn_index = random.randrange(spawn_count)

        transform = self.spawn_points[spawn_index]
        bp = self._get_actor_blueprint(bp_index)
        vehicle = self.world.try_spawn_actor(bp, transform)
        if vehicle is None:
            print(f"Failed to create vehicle at spawn point {spawn_index}")
        else:
            print(f"Created {vehicle.type_id} at spawn point {spawn_index}")
            if manage:
                self.actor_list.append(vehicle)

        return vehicle

    def spawn_vehicles_at(self, bp_index, spawn_indexes):
        """
        Creates a number of managed actors in the scene at the given
        spawn points, all of the same type.
        """
        for i in spawn_indexes:
            self.spawn_vehicle_at(bp_index, i)

    def spawn_select_vehicle_at(self, bp_indexes, spawn_index):
        """
        Creates a managed actor in the scene at the given spawn point,
        using a random type from the given set of blueprints.
        """
        bp_index = random.choice(bp_indexes)
        self.spawn_vehicle_at(bp_index, spawn_index)

    def spawn_select_vehicles_at(self, bp_indexes, spawn_indexes):
        """
        Creates a number of managed actors in the scene at the given
        spawn points, using a random type for each from the given set
        of blueprints.
        """
        for i in spawn_indexes:
            self.spawn_select_vehicle_at(bp_indexes, i)

    def spawn_sequenced_vehicles_at(self, bp_indexes, spawn_indexes):
        """
        Creates a number of managed actors in the scene at the given
        spawn points, using the given, repeating sequence of blueprints.
        """
        j = 0
        for i in spawn_indexes:
            self.spawn_vehicle_at(bp_indexes[j], i)
            j = (j + 1) % len(bp_indexes)

    def set_cctv_viewpoint(self, viewpoint):
        """
        Sets the main CARLA simulation server viewpoint. Useful for
        creating demonstration videos containing a combination of the
        1st-person player view and the "CCTV" view of a scene event.

        Takes an object defining the position and orientation of a
        CCTV camera in terms of a focus location, a distance from
        that location and pitch and yaw.
        """
        if viewpoint is None:
            return

        x = viewpoint["x"]
        y = viewpoint["y"]
        z = viewpoint["z"]
        d = viewpoint["distance"]
        yaw = math.radians(viewpoint["yaw"])
        pitch = math.radians(viewpoint["pitch"])

        location = carla.Location(
            x + -d * math.cos(yaw), y + d * math.sin(yaw), z + d * math.sin(pitch)
        )
        rotation = carla.Rotation(-math.degrees(pitch), -math.degrees(yaw), 0.0)
        transform = carla.Transform(location, rotation)

        spectator = self.client.get_world().get_spectator()
        spectator.set_transform(transform)

    def start(self, delay=0.0):
        """
        Starts the simulation running by setting up actors and getting
        them moving.
        """
        print("Starting...")
        self.set_cctv_viewpoint(self.scenario.get_cctv_viewpoint())
        self.scenario.setup_actors(self)
        if delay > 0.0:
            time.sleep(delay)

        self._action_actors(True)
        print("Started")

    def pause(self):
        """
        Pauses the simulation. In practice, this just turns off the
        autonomous behaviour of actors and is not a proper pause,
        so maybe not so useful.
        """
        print("Pausing...")
        self._action_actors(False)
        print("Paused")

    def resume(self):
        """
        Resumes a paused simulation.
        """
        print("Resuming...")
        self._action_actors(True)
        print("Resumed")

    def _get_actor_blueprint(self, bp_index):
        """
        PRIVATE: Gets the CARLA blueprint for the given index and
        assigns it a random colour. If the index is invalid, it's
        replaced by a random choice of blueprint.
        """
        bp_count = self.get_vehicle_blueprint_count()
        if bp_index < 0 or bp_index >= bp_count:
            bp_index = random.randrange(bp_count)

        bp = self.vehicle_blueprints[bp_index]
        if bp.has_attribute("color"):
            color = random.choice(bp.get_attribute("color").recommended_values)
            bp.set_attribute("color", color)

        return bp

    def _action_actors(self, autopilot):
        """
        PRIVATE: Activates or deactivates the autopilot state of
        managed actors.
        """
        for vehicle in self.actor_list:
            vehicle.set_autopilot(autopilot, self.tm_port)

    def _apply_sun_presets(self, values, weather):
        """
        PRIVATE: Modifies the weather definition using the given sun
        preset values.
        """
        if values is not None:
            print("Setting the Sun to", values)
            weather.sun_altitude_angle = values[0]
            weather.sun_azimuth_angle = values[1]

    def _apply_weather_presets(self, label, weather):
        """
        PRIVATE: Modifies the weather definition using the given
        weather preset values.
        """
        preset = WEATHER_PRESETS.get(label)
        if preset is not None:
            print("Setting the Weather to", preset)
            weather.cloudiness = preset[0]
            weather.precipitation = preset[1]
            weather.precipitation_deposits = preset[2]
            weather.wind_intensity = preset[3]
            weather.fog_density = preset[4]
            weather.fog_distance = preset[5]
            weather.fog_falloff = preset[6]
            weather.wetness = preset[7]
