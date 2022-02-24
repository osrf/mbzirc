#!/bin/sh
if [ "$#" -eq  "0" ]
then
  echo "Usage: ./set_sea_state [0-2]"
  exit 1
fi

v="0.0"

if [ "$1" -eq  "0" ]
then
 v="0.0"
elif [ "$1" -eq  "1" ]
then
 v="0.3"
elif [ "$1" -eq  "2" ]
then
 v="0.5"
else
  echo "Sea state must be [0-2]"
  exit 1
fi

DIR="$( cd "$( dirname "$0" )" && pwd )"


find ../models -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<gain>.*<\/gain>/\1<gain>'"$v"'<\/gain>/g' {} \;
find ../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<gain>.*<\/gain>/\1<gain>'"$v"'<\/gain>/g' {} \;

echo "Done setting sea state to $1"
