#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# Based on Example CARLA scripts:
#
#    Copyright (c) 2019 Computer Vision Center (CVC) at the
#                       Universitat Autonoma de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import collections
import weakref

import carla

from ..core.utilities import get_actor_display_name
from ..core.utilities import get_vector_norm

# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud, world):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        self.world = world
        carlaworld = self._parent.get_world()
        bp = carlaworld.get_blueprint_library().find("sensor.other.collision")
        self.sensor = carlaworld.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent
        )
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event)
        )

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification("Collision with %r" % actor_type)
        impulse = event.normal_impulse
        intensity = get_vector_norm(impulse)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)

        self.world.trigger_edr_event("Collision")
