#!/bin/bash
FOLDER=$1
FPS=$2
CAMID=$3
TITLE="EDR Camera $CAMID"
AUTHOR="Oxford Robotics Institute"
COPYRIGHT="(C) 2021 Oxford Robotics Institute"

if [ -z "$CAMID" ]; then
  echo "Usage: makecamvideo {date-time-folder} {fps} {camera-id} [mpeg]"
  exit 1
fi

BASE_FOLDER=~/data/edr/$FOLDER
CAMERA_FOLDER=$BASE_FOLDER/images/$CAMID-camera
VIDEOS_FOLDER=$BASE_FOLDER/videos
if [ "$4" == "mpeg" ]; then
  echo "Creating MPEG-4 file"
  FLAGS="-c:v mpeg4"
else
  FLAGS=""
fi

mkdir -p $VIDEOS_FOLDER

ffmpeg -r $FPS -pattern_type glob -i "$CAMERA_FOLDER/$CAMID-camera_*.png" -c:v libx264 -pix_fmt yuv420p $FLAGS -metadata title="$TITLE" -metadata artist="$AUTHOR" -metadata copyright="$COPYRIGHT" -metadata comment="$COPYRIGHT" $VIDEOS_FOLDER/$CAMID-camera.mp4
