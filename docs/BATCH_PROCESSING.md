# Batch Processing

Some of the other scripts allow sections of a log to be processed individually, such as reconstructions and video production. On top of those are some higher level scripts to make post-processing of an EDR log set really simple.

## process.sh

    ./process.sh <date-time-folder>

This script creates a variation by manipulating the ground-truth perceptions using default parameters and misclassification turned on. It then creates 3 reconstructions, all with velocity vectors turned on:

1. Ground-truth
2. Manipulated variant
3. Manipulated variant with ground-truth overlays

**Uses:** [manipulate.sh](MANIPULATE_PERCEPTION.md), [reconstruct.sh](SCENE_RECONSTRUCTION.md)

## videos.sh

    ./videos.sh <date-time-folder> <camera-fps> <perception-fps>

This script assumes that `process.sh` has already been run and that four virtual cameras were used for the EDR: front, rear, left and right

It creates a video for each of the cameras using the same `<camera-fps> `for each. It then creates a video for each of the 3 reconstructions (see above) using the `<perception-fps>`.

**Uses:** makecamvideo.sh, makereconvideo.sh

## do_everything.sh

    ./do_everything.sh <date-time-folder> <camera-fps> <perception-fps>

This script simply combines the two above and should be the only script needed to be run to process an EDR trial data set unless further variations or different parameters are required.

**Uses:** process.sh, videos.sh
