#!/bin/sh

if [ "$#" -eq  "0" ]
then
  echo "Usage: ./update_sea_state_params.sh [wave_gain] [wind_speed]"
  exit 1
fi

wave_gain=$1
wind_speed=$2

DIR="$( cd "$( dirname "$0" )" && pwd )"

# update wave gain param
find $DIR/../models -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<gain>.*<\/gain>/\1<gain>'"$wave_gain"'<\/gain>/g' {} \;
find $DIR/../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wavefield>.*\)<gain>.*<\/gain>/\1<gain>'"$wave_gain"'<\/gain>/g' {} \;

# # update wind linear vel param
find $DIR/../worlds -type f -exec sed -i ':a;N;$!ba;s/\(<wind>.*\)<linear_velocity>.*<\/linear_velocity>/\1<linear_velocity>'"$wind_speed"'<\/linear_velocity>/g' {} \;
