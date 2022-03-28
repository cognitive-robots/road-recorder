#!/bin/bash
FOLDER=$1
VARIANT=$2
if [ -z "$VARIANT" ]; then
  echo "Usage: manipulate {date-time-folder} {variant} [{options}]"
  exit 1
fi

BASE_FOLDER=~/data/edr/$FOLDER/perception

python3 manipulate_perception.py --input $BASE_FOLDER/logs/ --output $BASE_FOLDER/logs-$VARIANT/ $3 $4 $5 $6 $7 $8 $9
