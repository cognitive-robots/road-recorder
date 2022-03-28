#!/bin/bash
FOLDER=$1
if [ -z "$FOLDER" ]; then
  echo "Usage: process {date-time-folder}"
  exit 1
fi

echo "-----------------------------------"
echo "PROCESSING EDR: $FOLDER"
echo "-----------------------------------"
echo 

echo "-----------------------------------"
echo "MANIPULATING PERCEPTION..."
echo "-----------------------------------"
./manipulate.sh $FOLDER 1 -m

echo 
echo "-----------------------------------"
echo "RECONSTRUCTING..."
echo "-----------------------------------"
./reconstruct.sh $FOLDER "" "" -v
echo "-----------------------------------"
./reconstruct.sh $FOLDER 1 "" -v
echo "-----------------------------------"
./reconstruct.sh $FOLDER 1 gt -v
