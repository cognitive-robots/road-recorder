# Making Videos

## Overview

The EDR will produce sequences of individual image files from each virtual camera sensor. In addition, the sequence of perception logs can be turned into top-down reconstruction images.

It's useful to turn these into an MP4 video for ease of viewing. To do this, we use `ffmpeg`.

Script files take care of all the correct parameters. There's a separate script for each type of image data.

## Frame Rate

Unfortunately, we can't take advantage of timestamp information to obtain correct timing in the video. Instead, we have to estimate a fixed frame rate by looking at the timestamp intervals.

For example, if the interval is approximately 0.06 seconds, the frame rate will be approximately 1/0.06 = 17 fps.

## Camera Videos

    cd ~/code/road/road-sim/apps
    ./makecamvideo.sh <date-time-folder> <fps> <camera-id>

This will look for images in this folder:

    ~/data/edr/<date-time-folder>/images/<camera-id>-camera/

And create the video in this folder:

    ~/data/edr/<date-time-folder>/videos/

## Reconstruction Videos

    cd ~/code/road/road-sim/apps
    ./makereconvideo.sh <date-time-folder> <fps> [<variant> [<ground-truth>]]

This will look for reconstruction image files in variant-specific subfolder of this:

    ~/data/edr/<date-time-folder>/perception/

And create the video in this folder:

    ~/data/edr/<date-time-folder>/videos/

#### Examples:

    ./makereconvideo.sh 2021-10-12-17-18-22 16
    ./makereconvideo.sh 2021-10-12-17-18-22 16 1 gt
