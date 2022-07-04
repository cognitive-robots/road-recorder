#!/usr/bin/env python

# Copyright (c) 2022 Oxford Robotics Institute (ORI), University of Oxford
#
# Based on Example CARLA scripts:
#
#    Copyright (c) 2019 Intel Labs
#    Copyright (c) 2019 Computer Vision Center (CVC) at the
#                       Universitat Autonoma de Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import sys

import carla

if sys.version_info >= (3, 0):
    from configparser import ConfigParser
else:
    from ConfigParser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_g
    from pygame.locals import K_h
    from pygame.locals import K_i
    from pygame.locals import K_j
    from pygame.locals import K_k
    from pygame.locals import K_l
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_z
except ImportError:
    raise RuntimeError("cannot import pygame, make sure pygame package is installed")


# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    """
    Handles control of the player actor (vehicle or pedestrian) using a
    keyboard, wheel or joystick. Some keys are also used to change the
    environment and view settings.
    """

    def __init__(self, world, start_in_autopilot, lights_on, controller_id):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = (
                carla.VehicleLightState.LowBeam
                if lights_on
                else carla.VehicleLightState.NONE
            )
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel/joystick
        print("Using controller:", controller_id)
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        if joystick_count == 0:
            self._joystick = None
        else:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read("wheel_config.ini")
        self._steer_scale = (
            float(self._parser.get(controller_id, "steering_scale"))
            if self._parser.has_option(controller_id, "steering_scale")
            else 1.0
        )
        self._steer_offset = (
            float(self._parser.get(controller_id, "steering_offset"))
            if self._parser.has_option(controller_id, "steering_offset")
            else 0.0
        )
        self._steer_deadband = (
            float(self._parser.get(controller_id, "steering_deadband"))
            if self._parser.has_option(controller_id, "steering_deadband")
            else 0.0
        )
        self._throttle_deadband = (
            float(self._parser.get(controller_id, "throttle_deadband"))
            if self._parser.has_option(controller_id, "throttle_deadband")
            else 0.0
        )
        self._steer_idx = int(self._parser.get(controller_id, "steering_wheel"))
        self._throttle_idx = int(self._parser.get(controller_id, "throttle"))
        self._brake_idx = int(self._parser.get(controller_id, "brake"))
        self._reverse_idx = int(self._parser.get(controller_id, "reverse"))
        self._handbrake_idx = int(self._parser.get(controller_id, "handbrake"))
        self._handbrake_on = False

    def parse_events(self, world, clock):
        """
        Handles pygame events such as keypresses, joystick buttons
        and quit events. Returns True to exit the application.
        """
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

            elif event.type == pygame.JOYBUTTONDOWN:
                # if event.button == 0:
                #     world.restart()
                # elif event.button == 1:
                #     world.hud.toggle_info()
                # elif event.button == 2:
                #     world.camera_manager.toggle_camera()
                # elif event.button == 3:
                #     world.next_weather()
                # elif event.button == self._reverse_idx:
                if event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                # elif event.button == self._handbrake_idx:
                #     self._handbrake_on = not self._handbrake_on
                # elif event.button == 23:
                #     world.camera_manager.next_sensor()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_F1:
                    world.hud.toggle_info()
                elif event.key == K_h or (
                    event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT
                ):
                    world.hud.help.toggle()
                elif event.key == K_TAB:
                    world.camera_manager.toggle_camera()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                elif event.key == K_k:
                    world.trigger_edr_event("Manual Trigger")
                elif event.key == K_j:
                    world.save_edr_data()
                elif event.key == K_g:
                    world.clear_edr_event()
                elif event.key == K_BACKQUOTE:
                    world.camera_manager.next_sensor()
                elif event.key > K_0 and event.key <= K_9:
                    world.camera_manager.set_sensor(event.key - 1 - K_0)
                elif event.key == K_r:
                    world.camera_manager.toggle_recording()
                elif event.key == K_l and pygame.key.get_mods() & KMOD_CTRL:
                    current_lights ^= carla.VehicleLightState.Special1
                elif event.key == K_l and pygame.key.get_mods() & KMOD_SHIFT:
                    current_lights ^= carla.VehicleLightState.HighBeam
                elif event.key == K_l:
                    # Use 'L' key to switch between lights:
                    # closed -> position -> low beam -> fog
                    if not self._lights & carla.VehicleLightState.Position:
                        world.hud.notification("Position lights")
                        current_lights |= carla.VehicleLightState.Position
                    else:
                        world.hud.notification("Low beam lights")
                        current_lights |= carla.VehicleLightState.LowBeam
                    if self._lights & carla.VehicleLightState.LowBeam:
                        world.hud.notification("Fog lights")
                        current_lights |= carla.VehicleLightState.Fog
                    if self._lights & carla.VehicleLightState.Fog:
                        world.hud.notification("Lights off")
                        current_lights ^= carla.VehicleLightState.Position
                        current_lights ^= carla.VehicleLightState.LowBeam
                        current_lights ^= carla.VehicleLightState.Fog
                elif event.key == K_i:
                    current_lights ^= carla.VehicleLightState.Interior
                elif event.key == K_z:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                elif event.key == K_x:
                    current_lights ^= carla.VehicleLightState.RightBlinker

                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = (
                            not self._control.manual_gear_shift
                        )
                        self._control.gear = world.player.get_control().gear
                        world.hud.notification(
                            "%s Transmission"
                            % (
                                "Manual"
                                if self._control.manual_gear_shift
                                else "Automatic"
                            )
                        )
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p:
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
                        world.hud.notification(
                            "Autopilot %s"
                            % ("On" if self._autopilot_enabled else "Off")
                        )

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else:  # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else:  # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if (
                    current_lights != self._lights
                ):  # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))

            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())

            world.player.apply_control(self._control)

        return False

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        Sets player vehicle steering, throttle and brake when not
        in autopilot mode, using keyboard controls.
        """
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        # self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        """
        Sets player vehicle steering, throttle and brake when not
        in autopilot mode, using joystick or steering wheel controls.
        """
        if self._joystick is None:
            return

        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [
            float(self._joystick.get_button(i))
            for i in range(self._joystick.get_numbuttons())
        ]
        # print (jsButtons)

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        steer_input = max(
            -1.0, min(1.0, jsInputs[self._steer_idx] + self._steer_offset)
        )
        steer_value = (
            (steer_input * self._steer_scale)
            if math.fabs(steer_input) >= self._steer_deadband
            else 0.0
        )

        if self._throttle_idx == self._brake_idx:
            # Combined throttle/brake axis => joystick
            steer_value = steer_value**3
            throttle_input = jsInputs[self._throttle_idx]
            throttle_value = (
                (2.0 * throttle_input + 1.0)
                if throttle_input <= self._throttle_deadband
                else 1.0
            )
            brake_value = (
                (-2.0 * throttle_input + 1.0)
                if throttle_input >= -self._throttle_deadband
                else 1.0
            )
        else:
            # Separate throttle/brake axes/pedals => wheel
            throttle_input = jsInputs[self._throttle_idx]
            throttle_value = (
                throttle_input
                if math.fabs(throttle_input) >= self._throttle_deadband
                else 1.0
            )

            brake_input = jsInputs[self._brake_idx]
            brake_value = (
                brake_input
                if math.fabs(brake_input) >= self._throttle_deadband
                else 1.0
            )

        # print(steer_input, steer_value)
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * steer_value)

        # print(f'Throttle: {throttle_value}, Brake: {brake_value}')
        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(-0.7 * throttle_value + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = K2 + (2.05 * math.log10(-0.7 * brake_value + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        # toggle = jsButtons[self._reverse_idx]

        # self._control.hand_brake = bool(jsButtons[self._handbrake_idx])
        self._control.hand_brake = self._handbrake_on

    def _parse_walker_keys(self, keys, milliseconds):
        """
        Sets player pedestrian movement when not in autopilot mode,
        using keyboard controls.
        """
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = 0.01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = 0.01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
