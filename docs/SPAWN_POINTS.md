# Viewing Spawn Points

When devising scenarios, it's useful to know where all the spawn points are. While you can use any location you like, the spawn points have been carefully selected by the CARLA team for each particular world.

`view_spawn_points.py` will visit every point and display it in the simulation window for a few seconds while logging it to the terminal. Make a note of the interesting ones you want to use:

    cd ~/code/road/road-sim/apps
    python3 view_spawn_points.py

There are command line options for setting the start index, the time interval between successive points and some view parameters:

    python3 view_spawn_points.py --help

### Targeted Search

One very useful technique is to manually drive to a desired location and note down the coordinates. Then specify these in the `--location` argument along with a search `--radius`. This will limit the output to show and report only nearby spawn points.