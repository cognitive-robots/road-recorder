#!/bin/bash
FOLDER=$1
CAMERA_FPS=$2
PERCEPTION_FPS=$3
if [ -z "$PERCEPTION_FPS" ]; then
  echo "Usage: videos {date-time-folder} {camera-fps} {perception-fps}"
  exit 1
fi

echo "-----------------------------------"
echo "PROCESSING EDR: $FOLDER"
echo "-----------------------------------"
echo 

echo "-----------------------------------"
echo "MAKING VIDEOS..."
echo "-----------------------------------"
./makecamvideo.sh $FOLDER $CAMERA_FPS front
echo "-----------------------------------"
./makecamvideo.sh $FOLDER $CAMERA_FPS rear
echo "-----------------------------------"
./makecamvideo.sh $FOLDER $CAMERA_FPS left
echo "-----------------------------------"
./makecamvideo.sh $FOLDER $CAMERA_FPS right

echo "-----------------------------------"
./makereconvideo.sh $FOLDER $PERCEPTION_FPS
echo "-----------------------------------"
./makereconvideo.sh $FOLDER $PERCEPTION_FPS 1
echo "-----------------------------------"
./makereconvideo.sh $FOLDER $PERCEPTION_FPS 1 gt
echo "-----------------------------------"
