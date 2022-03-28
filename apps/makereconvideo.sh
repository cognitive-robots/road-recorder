#!/bin/bash
FOLDER=$1
FPS=$2
VARIANT=$3
GT=$4
TITLE="Event Reconstruction"
AUTHOR="Oxford Robotics Institute"
COPYRIGHT="(C) 2021 Oxford Robotics Institute"

if [ -z "$FOLDER" ]; then
  echo "Usage: makereconvideo {date-time-folder} {fps} [{variant} [{gt}]]"
  exit 1
fi

BASE_FOLDER=~/data/edr/$FOLDER
PERCEPTION_FOLDER=$BASE_FOLDER/perception
VIDEOS_FOLDER=$BASE_FOLDER/videos

mkdir -p $VIDEOS_FOLDER

if [ -z "$VARIANT" ]; then
  ffmpeg -r $FPS -pattern_type glob -i "$PERCEPTION_FOLDER/reconstruction/logs_*.png" -c:v libx264 -pix_fmt yuv420p -metadata title="$TITLE" -metadata artist="$AUTHOR" -metadata copyright="$COPYRIGHT" -metadata comment="$COPYRIGHT" $VIDEOS_FOLDER/reconstruction.mp4
else
  echo "Using variant: $VARIANT"
  if [ -z "$GT" ]; then
    # No ground-truth overlay
    ffmpeg -r $FPS -pattern_type glob -i "$PERCEPTION_FOLDER/reconstruction-$VARIANT/logs_*.png" -c:v libx264 -pix_fmt yuv420p -metadata title="$TITLE" -metadata artist="$AUTHOR" -metadata copyright="$COPYRIGHT" -metadata comment="$COPYRIGHT" $VIDEOS_FOLDER/reconstruction-$VARIANT.mp4
  else
    # Overlay given variant as ground-truth
    echo "Using variant as ground truth: $GT"
    ffmpeg -r $FPS -pattern_type glob -i "$PERCEPTION_FOLDER/reconstruction-$VARIANT-$GT/logs_*.png" -c:v libx264 -pix_fmt yuv420p -metadata title="$TITLE" -metadata artist="$AUTHOR" -metadata copyright="$COPYRIGHT" -metadata comment="$COPYRIGHT" $VIDEOS_FOLDER/reconstruction-$VARIANT-$GT.mp4
  fi
fi
