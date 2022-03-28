#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This is the main application program for the RoAD project. It sets up
a scenario then lest you manually drive the ego vehicle from a 1st or
3rd-person perspective to execute that scenario and save Event Data
Recorder (EDR) data to disk.

NOTE: Most of the key functionality can be found in the World and
DualControl sub-modules.
"""

import argparse
import logging
import math
import os

import carla
import pygame

from carlasim.core.hud import HUD
from carlasim.core.world import World

from carlasim.control.dual_control import DualControl

from carlasim.scenarios.scenario_manager import ScenarioManager
from carlasim.scenarios.scenario_manager import Scenario


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def get_view_transform(view_str):
    """
    Extracts a transform from an argumemt string. This is mainly used to
    set the main CARLA simulation view. However, that will be overridden
    by any such setting in a loaded scenario.
    """
    x, y, z, d, yaw_deg, pitch_deg = [float(v) for v in view_str.split(",")]
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    location = carla.Location(
        x + -d * math.cos(yaw), y + d * math.sin(yaw), z + d * math.sin(pitch)
    )
    rotation = carla.Rotation(-math.degrees(pitch), -math.degrees(yaw), 0.0)
    return carla.Transform(location, rotation)


def get_scenario(pathname):
    """
    Returns a Scenario object built from the JSON descriptor file given
    by pathname or None if this fails for any reason.
    """
    scenario = None
    if pathname is not None and pathname != "":
        scenario = Scenario()
        if not scenario.load(pathname):
            scenario = None

    return scenario


def game_loop(args):
    """
    The main pygame loop.
    """
    if args.wpos != "":
        os.environ["SDL_VIDEO_WINDOW_POS"] = args.wpos

    pygame.init()
    pygame.font.init()

    world = None
    scenario_manager = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        lights_on = False
        scenario = get_scenario(args.scenario)
        if scenario is not None:
            if scenario.hour is not None and (
                scenario.hour < 7.5 or scenario.hour > 5.5
            ):
                lights_on = True

            scenario_manager = ScenarioManager(client, args.tm_port, scenario)
            if args.start_index < 0:
                args.start_index = scenario.get_player_spawn_index()
        else:
            if args.scenario != "":
                print("Unknown scenario:", args.scenario)
            if args.map != "":
                client.load_world(args.map)

        if args.view != "":
            vt = get_view_transform(args.view)
            spectator = client.get_world().get_spectator()
            spectator.set_transform(vt)

        display = pygame.display.set_mode(
            (args.width, args.height), pygame.HWSURFACE | pygame.DOUBLEBUF
        )

        print("Lights On:", lights_on)
        hud = HUD(args.width, args.height)
        data_path = os.path.expanduser("~/data/edr/")
        world = World(args, client.get_world(), hud, data_path)
        controller = DualControl(world, args.autopilot, lights_on, args.controller)

        world.set_simulation_fps(20)

        if scenario_manager is not None:
            scenario_manager.start(1.0)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(world, clock):
                return

            if scenario_manager is not None:
                scenario_manager.tick(world.player.get_transform())

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if scenario_manager is not None:
            scenario_manager.on_shutdown()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(description="CARLA Manual Control Client")
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="print debug information",
    )
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    argparser.add_argument(
        "--tm-port",
        metavar="P",
        default=8000,
        type=int,
        help="port to communicate with Traffic Manager (default: 8000)",
    )
    argparser.add_argument(
        "-a",
        "--autopilot",
        action="store_true",
        help="enable autopilot"
    )
    argparser.add_argument(
        "-e",
        "--edr",
        action="store_true",
        help="enable EDR"
    )
    argparser.add_argument(
        "--edr_autosave",
        action="store_true",
        help="autosave EDR data to disk as soon as the post-event time expires"
    )
    argparser.add_argument(
        "--edr_sensors",
        default="",
        help="name of EDR sensors configuration file (default: not set)",
    )
    argparser.add_argument(
        "--edr_pretime",
        default=5.0,
        type=float,
        help="duration of EDR log before the event, in seconds (default: 5.0)",
    )
    argparser.add_argument(
        "--edr_posttime",
        default=2.0,
        type=float,
        help="duration of EDR log after the event, in seconds (default: 2.0)",
    )
    argparser.add_argument(
        "-s",
        "--start_index",
        default=-1,
        type=int,
        help="starting spawn index for player vehicle (default: -1)",
    )
    argparser.add_argument(
        "--map",
        default="",
        help="map to load if not using a scenario (default: not set)",
    )
    argparser.add_argument(
        "--scenario",
        default="",
        help="name of scenario to run (default: not set)"
    )
    argparser.add_argument(
        "--near_miss_velocity",
        default=1.0,
        type=float,
        help="Trigger velocity (m/s) testing near misses with vulnerable road users (VRUs) (default: 1.0)",
    )
    argparser.add_argument(
        "--near_miss_log_file",
        default="",
        help="File to log vulnerable road user (VRU) near misses to (default: not set)",
    )
    argparser.add_argument(
        "--near_miss_sound",
        default="",
        help="WAV file to play when logging a near miss event (default: not set)",
    )
    argparser.add_argument(
        "--wpos",
        metavar="X,Y",
        default="",
        help="optional window position (default: not set)",
    )
    argparser.add_argument(
        "--view",
        metavar="X,Y,Z,DISTANCE,YAW,PITCH",
        default="",
        help="optional CARLA simulation view (default: not set)",
    )
    argparser.add_argument(
        "--res",
        metavar="WIDTHxHEIGHT",
        default="1280x720",
        help="window resolution (default: 1280x720)",
    )
    argparser.add_argument(
        "--filter",
        metavar="PATTERN",
        default="vehicle.*",
        help='actor filter (default: "vehicle.*")',
    )
    argparser.add_argument(
        "--controller",
        default="G29 Racing Wheel",
        help='name of input controller configuration to use from INI file (default: "G29 Racing Wheel")',
    )

    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split("x")]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("listening to server %s:%s", args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")


if __name__ == "__main__":
    main()
