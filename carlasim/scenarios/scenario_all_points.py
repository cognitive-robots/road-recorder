#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Scenario All Points: Create a vehicle at every spawn point and start them driving

This example is run separately as an alternative to using a JSON scenario file. 
"""

import argparse
import random

import carla

from scenario_manager import ScenarioManager
from scenario_manager import Scenario

RANDOM_SEED = 163842


class ScenarioAllPoints(Scenario):
    def __init__(self):
        super().__init__()
        self.label = "All Points"
        self.description = "All the spawn points!"
        self.managed_actor_bp_indexes = [-1]

    def setup_actors(self, sm):
        count = sm.get_spawn_point_count()
        for i in range(count):
            self.managed_actor_spawn_indexes.append(i)

        super().setup_actors(sm)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
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
        help="port to communicate with TM (default: 8000)",
    )

    args = argparser.parse_args()
    print(__doc__)

    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)

    sm = ScenarioManager(client, args.tm_port, ScenarioAllPoints(), RANDOM_SEED)
    try:
        sm.start(1.0)
        while True:
            sm.tick(None)

    finally:
        sm.on_shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print("Done")
