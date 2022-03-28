# Manual Driving, Scenarios and EDR Execution

## Overview

The core part of this project is being able to manually drive around and generate data from the black-box Event Data Recorder (EDR), as if from a real vehicle (autonomous or otherwise).

Scenarios allow for a specific set of initial conditions, including:

- the (town) map to use
- weather conditions and time of day
- the starting location for the vehicle being driven
- other vehicles in the scene
- vehicles with trigger conditions

Since CARLA operates a client/server model, the driving application containing the EDR is always run as a separate process to the main CARLA simulation.

![Manual driving rear-end collision](./scenario3_view_clip1.gif "Manual driving rear-end collision")

## Recording Event Data

To record event data, run the CARLA simulator then the driving application. A scenario can be provided, if desired. However, the EDR will still operate without one.

This requires a couple of command line terminals.

### Terminal #1 : Simulator

Run the CARLA simulator:

    cd /opt/carla-simulator
    ./CarlaUE4.sh

When experimenting on a laptop use this instead:

    ./CarlaUE4.sh -quality-level=Low

**Note:** Since the simulator rendering window isn't actually useful (unless making demo videos), it can also help with performance to aim this view deep into the ground so it has nothing much to render. However, using a scenario with a "CCTV" definition will override this and reset the view.


### Terminal #2 : Manual Driving

Run the manual driving application:

    cd ~/code/road/road-sim/apps
    python3 drive.py

The vehicle will appear at a random spawn location or the spawn location provided by the optional `--start_index` parameter. No other vehicles will appear by default but other client applications may be run to populate the town.

##### Scenario

Use the `--scenario` flag to execute a scenario (see the "Scenario Definition Files" section below):

    python3 drive.py --scenario ../carlasim/scenarios/<scenario_xxx>.json

##### EDR

To activate the EDR, set the `--edr` flag and specify `--edr_sensors` (see the "EDR Sensor Configuration Files" section below):

    python3 drive.py --edr --edr_sensors ../carlasim/edrsensors/<edr_xxx>.json

**Note 1:** EDR uses a lot of resources and dramatically hurts performance. To practice driving around without the EDR, exclude this flag.

**Note 2:** The `--edr` flag is practically redundant but is used for backwards compatibility of scripts and also for convenience to allow the EDR to be turned on/off easily without massive command line edits.

##### EDR Buffer Size

The EDR will store data for a period of time before and after the event. These can be specified using the `--edr_pretime` and `--edr_posttime` command line options, which default to `5.0` and `2.0` seconds respectively, e.g.


    python3 drive.py --edr_pretime 10.0 --edr_posttime 5.0 ...


##### Window Size

Unfortunately, the PyGame window is not resizable once created so you can also set the window size here, e.g.

    python3 drive.py --res=1920x990


### Execute the Scenario

Once everything has started, check the output of **Terminal #2** for events. When a collision event has been automatically detected, the following message will appear:

     EDR Triggered - Collision!

Or you might see this instead:

     EDR Triggered - Too close to Vulnerable Road User (VRU)!

If this was an unintentional event, press the `G` key to reset the EDR and check the following message is displayed:

     Resetting EDR

### Saving EDR Data

Once an event has triggered, you must wait for the post-event buffer to fill with data (the post-event time which defaults to 2 seconds).

If the `--edr_autosave` flag was provided on the command line, saving will then begin automatically.

Otherwise, an `"EDR data ready"` message will appear on the terminal and you can choose to press the `J` key to save the data to disk or press the `G` key to clear the event and discard the data.

When saving, you will then see messages such as the following:

    Saving EDR data to: /home/{username}/data/edr/{date-time}
    Saving EDRVehicleStateSensor data 
    Saving EDRPerceptionSensor data for logs
    Saving EDRLidar3D data for lidar
    Saving EDRCamera data for front-camera
    Saving EDRCamera data for rear-camera
    Saving EDRCamera data for left-camera
    Saving EDRCamera data for right-camera
    EDR data saved
    Resetting EDR

Note that the EDR is automatically reset afterwards, ready for another event to be captured and saved.

**IMPORTANT:** Saving the data to disk is a time-consuming process (especially for virtual camera images). The driving application is single-threaded so will freeze while this is being performed. Depending on the virtual sensor suite, it could take a couple of minutes.


## Saved EDR Data

The EDR data will be saved according to the following structure, depending on the actual virtual sensor suite in use:

    ~/data/edr/
    └── {date-time}/
        ├── images/
        │   ├── {camera1-id}/
        │   │   ├── {camera1-id}_{timestamp}_{offset}.png
        │   │   ├── {camera1-id}_{timestamp}_{offset}.png
        │   │   ├── :
        │   │   └── {camera1-id}_{timestamp}_{offset}.png
        │   :
        │   └── {cameraN-id}/
        │       ├── {cameraN-id}_{timestamp}_{offset}.png
        │       ├── {cameraN-id}_{timestamp}_{offset}.png
        │       ├── :
        │       └── {cameraN-id}_{timestamp}_{offset}.png
        │
        ├── lidar3d/
        │   ├── {lidar1-id}/
        │   │   ├── {lidar1-id}_{timestamp}_{offset}.ply
        │   │   ├── {lidar1-id}_{timestamp}_{offset}.ply
        │   │   ├── :
        │   │   └── {lidar1-id}_{timestamp}_{offset}.ply
        │   :
        │   └── {lidarN-id}/
        │       ├── {lidarN-id}_{timestamp}_{offset}.ply
        │       ├── {lidarN-id}_{timestamp}_{offset}.ply
        │       ├── :
        │       └── {lidarN-id}_{timestamp}_{offset}.ply
        │
        ├── perception/
        │   └── logs/
        │       ├── logs_{timestamp}_{offset}.json
        │       ├── logs_{timestamp}_{offset}.json
        │       ├── :
        │       └── logs_{timestamp}_{offset}.json
        │
        └── vehicle-state/
            └── vehicle_state.csv

## EDR Sensor Configuration Files

To specify the location and attributes of cameras and LiDARs attached to the vehicle, use a JSON configuration file with the following simple structure.

Note, all positions are relative to the centre of the vehicle bounding box. If attributes are not provided, defaults will be used as shown after '=':

    {
        "lidars3d": [
            {
                "label": "<lidar-id>",
                "x": <+x-is-forward = 0.0>,
                "y": <+y-is-right = 0.0>,
                "z": <+z-is-up = 0.0>,
                "roll": <+roll-is-clockwise = 0.0>,
                "pitch": <+pitch-is-up = 0.0>,
                "yaw": <+yaw-is-right = 0.0>
            },
            {
                <next-lidar-configuration>
            }
        ],
        "cameras": [
            {
                "label": "<camera-id>",
                "x": <+x-is-forward = 0.0>,
                "y": <+y-is-right = 0.0>,
                "z": <+z-is-up = 0.0>,
                "roll": <+roll-is-clockwise = 0.0>,
                "pitch": <+pitch-is-up = 0.0>,
                "yaw": <+yaw-is-right = 0.0>
                "width": <width-in-pixels = 720>,
                "height": <height-in-pixels = 480>,
                "fov": <degrees = 110>,
                "rate": <hz = 10.0>
            },
            {
                <next-camera-configuration>
            }
        ]
    
    }

## Scenario Definition Files

Scenarios are defined in JSON files with the following structure:

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

### Entry Descriptions

#### Map

The CARLA town name must be provided to load up the correct `map` for the scenario. At the time of writing, these are:

    Town01, Town02, Town03, Town04, Town05, Town06, Town07, Town10HD

Note that the `_Opt` suffix versions are the same but have split layers for control of visible elements in the scene. This feature is not used so it doesn't matter which version is used.

#### Environment

The optional `environment` setting controls the time of day and weather. If provided, it should be one of the CARLA-defined presets:

    ClearNoon, ClearSunset, CloudyNoon, CloudySunset, Default,
    HardRainNoon, HardRainSunset, MidRainSunset, MidRainyNoon,
    SoftRainNoon, SoftRainSunset, WetCloudyNoon, WetCloudySunset,
    WetNoon, WetSunset

#### Sun Position

Whether or not the `environment` option is used, the Sun position may be set with an optional `sun_preset` value:

     day, night, sunrise, sunset

If not provided, an alternative is to set the `hour` of day. This is a value between `0.0` and `24.0` and is also optional.

Note that the decimal represents a proportion of the hour, not the number of minutes. So 5:30 pm would be represented by the value `17.5`.

You might need to experiment with this value to get the effect you want.

##### Weather

Whether or not the `environment` option is used, the weather condition may be set with an optional `weather_preset` value:

     clear, fog, overcast, rain, storm

#### CCTV Viewpoint

This is optional and sets the display viewpoint of the CARLA server window. This is normally used to act as a virtual "CCTV camera" of the scene event with the window placed on top of the driver window so it can all be screen recorded.

#### Managed Actors

These are actors (vehicles) which are placed in the scene at the given spawn points then set in motion. The program will produce one actor at each of the given spawn points but cycle through the list of allowed blueprints to determine what actor to spawn.

There must be at least one actor blueprint provided in the list but there may be more or fewer than the number of actors spawned. If -1 is given as a blueprint index (or a number higher than the number of CARLA blueprints available) then it will be substituted with a random actor. Therefore, a list with a single -1 entry will produce random actors at all the specified spawn points.

Managed actors are optional.

#### Triggered Actors

The idea of a triggered actor is to place it as a stationary object in the scene then trigger it to move as soon as the driven EGO vehicle gets within the specified radius of the trigger location. For example, this allows for vehicles to move into the path of the EGO vehicle in a relatively predictable way.

Triggered actors are set to have zero velocity when placed but, depending on the location (a slope for example), you may find that the physics engine causes it to move anyway and not be where you want it to be when the trigger point is reached. The only solution to this is to pick the spawn location carefully!

Triggered actors are optional.

#### Useful blueprints, as of CARLA 0.9.11

Select from these sets of blueprints for actors:

Decent vehicles: [0, 1, 3, 4, 5, 6, 8, 10, 11, 15, 16, 18, 19, 20, 21, 23, 25, 26, 28, 29, 30]
Stupid vehicles: [9, 22]
Police vehicles: [12, 24]
Motorbikes: [7, 14, 17]
Bicycles: [2, 13, 27]

## VRU Hotspots

If you provide a `--near_miss_log_file` argument, every time you drive too close to a Vulnerable Road User (VRU) at excessive speed, an entry will be logged to this file.

The file contains the map name (town) and each log entry includes:

- VRU Type
- Distance Trigger Threshold
- Distance
- VRU Speed
- VRU Location
- EGO Vehicle Speed
- EGO Vehicle Location

## Controller Configuration File

The manual driving application can be controlled using a steering wheel and pedals or joystick. If the chosen device is not found then the keyboard can be used instead, though steering is very difficult in that case.

The input device is specified using the `--controller` command line parameter and configured in a file called `wheel_config.ini`, which should be edited to add additional controllers:

    [G29 Racing Wheel]
    steering_wheel = 0
    clutch = 5
    throttle = 1
    brake = 2
    handbrake = 4
    reverse = 5
    
    [Phantom Hawk Joystick]
    steering_scale = 0.75
    steering_offset = -0.18
    steering_deadband = 0.05
    throttle_deadband = 0.25
    steering_wheel = 0
    clutch = 3
    throttle = 1
    brake = 1
    handbrake = 6
    reverse = 4

**Note:** If `--controller` is not specified, it defaults to "**G29 Racing Wheel**".

The settings define which input channel to use for each control. `steering_wheel`, `clutch`, `throttle` and `brake` are analog inputs whereas `handbrake` and `reverse` are button channels.

For joystick control, the `throttle` and `brake` can share the same channel and the `_deadband` options will likely be necessary. Also, steering sensitivity can be adjusted with `steering_scale` and a `steering_offset` can be used to account for sticks which don't centre very well.
