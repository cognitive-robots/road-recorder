# Fixing Detections

The perception model currently being used is very simple. It extracts all of the vehicle objects from CARLA (essentially, the 'ground-truth') and assumes the vehicle is aware of all of them precisely.

There are two problems that `fix_perception_json.py` can help with:

1. Sometimes, CARLA reports bogus bounding box extent values for bicycles (possibly for others too). If this program finds a bicycle, it replaces the bound box extent with a good one.
2. Some scenarios are intended to demonstrate that the vehicle would not have been able to detect a vehicle in time to avoid a collision. For example, it might emerge suddenly from behind a fence. However, the ground-truth representation will suggest it was always visible. To help with this, a `--timestamp` argument can be provided. Any detections before this time (whether visible or not) will be removed.

### Command Line

    cd ~/code/road/road-sim/apps
    python3 fix_perception_json.py --input <logs_xxx.json-or-folder> \
           --output <output-folder> [--timestamp <clip-timestamp>]

The output folder must be different to the input folder.

If `--input` specifies a folder, all JSON files in that folder will be processed. This is the typical use case.