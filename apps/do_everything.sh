#!/bin/bash
CAMERA_FPS=$1
PERCEPTION_FPS=$2
FOLDER=$3
if [ -z "$FOLDER" ]; then
  echo "Usage: do_everything {camera-fps} {perception-fps} {date-time-folder}"
  exit 1
fi

echo "-----------------------------------"
echo "PROCESSING EDR: $FOLDER"
echo "-----------------------------------"

./process.sh $FOLDER
./videos.sh $FOLDER $CAMERA_FPS $PERCEPTION_FPS

echo 
echo "DONE!"
echo 
