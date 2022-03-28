#!/bin/bash
FOLDER=$1
VARIANT=$2
GT=$3
if [ -z "$FOLDER" ]; then
  echo "Usage: reconstruct {date-time-folder} [{variant} [{gt} [{options}]]]"
  exit 1
fi
if [ -z "$VARIANT" ]; then
  python3 perception_json2png.py --input ~/data/edr/$FOLDER/perception/logs/ --output ~/data/edr/$FOLDER/perception/reconstruction/ --lidar ~/data/edr/$FOLDER/lidar3d/lidar/ $3 $4 $5 $6 $7 $8 $9
else
  echo "Using variant: $VARIANT"
  if [ -z "$GT" ]; then
    # No ground-truth overlay
    python3 perception_json2png.py --input ~/data/edr/$FOLDER/perception/logs-$VARIANT/ --output ~/data/edr/$FOLDER/perception/reconstruction-$VARIANT/ --lidar ~/data/edr/$FOLDER/lidar3d/lidar/ $4 $5 $6 $7 $8 $9
  elif [ "$GT" == "gt" ]; then
    # Overlay 'true' ground-truth 
    echo "Using ground truth"
    python3 perception_json2png.py --gt ~/data/edr/$FOLDER/perception/logs/ --input ~/data/edr/$FOLDER/perception/logs-$VARIANT/ --output ~/data/edr/$FOLDER/perception/reconstruction-$VARIANT-gt/ --lidar ~/data/edr/$FOLDER/lidar3d/lidar/ $4 $5 $6 $7 $8 $9
  else
    # Overlay given variant as ground-truth
    echo "Using variant as ground truth: $GT"
    python3 perception_json2png.py --gt ~/data/edr/$FOLDER/perception/logs-$GT/ --input ~/data/edr/$FOLDER/perception/logs-$VARIANT/ --output ~/data/edr/$FOLDER/perception/reconstruction-$VARIANT-$GT/ --lidar ~/data/edr/$FOLDER/lidar3d/lidar/ $4 $5 $6 $7 $8 $9
  fi
fi
