# Manipulating Ground-Truth Perception

The perception logs generated for the EDR are taken from the ground-truth bounding boxes of objects around the vehicle, extracted from CARLA.

For greater realism, the logs can be manipulated to add noise and allow for perception drop-outs.

    cd ~/code/road/road-sim/apps
    python3 manipulate_perception.py --input <path-to-input-logs> --output <path-to-output-logs>

The input and output paths must be different. The input will be something like this:

    --input ~/data/edr/<date-time>/perception/logs/

The output should be similar but with a '**variant**' suffix, e.g. 'noisy' in this case:

    --output ~/data/edr/<date-time>/perception/logs-noisy/

The default parameters should get decent results but there are various options to change how it works. See:

    python3 manipulate_perception.py --help

### Convenience Script

To make things easier, there's a convenient shell script which will create a set of perception variations in a suitably-named sibling `logs` folder using a 'standard' EDR data location:

    ./manipulate.sh <date-time-folder> <variant> [<options>]

The input folder will be:

    ~/data/edr/<date-time-folder>/perception/logs/

The output folder will be:

    ~/data/edr/<date-time-folder>/perception/logs-<variant>/
