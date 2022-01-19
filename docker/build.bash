#!/usr/bin/env bash

#
# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Builds a Docker image.

if [ $# -eq 0 ]
then
    echo "Usage: $0 directory-name [mbzirc_repo_url] [mbzirc_sha_commit]"
    exit 1
fi

# get path to current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ ! -d $DIR/$1 ]
then
  echo "image-name must be a directory in the same folder as this script"
  exit 2
fi

mbzirc_repo_url=${2:-https://github.com/osrf/mbzirc}
mbzirc_sha_commit=${3}
user_id=$(id -u)
image_name=$(basename $1)
image_plus_tag=$image_name:latest-$(date +%F_%H%M)

shift

docker build --rm \
  -t "$image_plus_tag" \
  --build-arg user_id="$user_id"  \
  --build-arg mbzirc_sha_commit="${mbzirc_sha_commit}" \
  --build-arg mbzirc_repo_url="${mbzirc_repo_url}" \
  -f "$DIR/$image_name/Dockerfile" . \

docker tag $image_plus_tag $image_name:latest

echo "Built $image_plus_tag and tagged as $image_name:latest"
