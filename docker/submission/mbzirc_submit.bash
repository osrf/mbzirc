#!/bin/bash

# Copyright 2022 Open Source Robotics Foundation, Inc.
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


# Check for the existance of the team name on the command line
if [ $# -lt 5 ]; then
  echo "Specify the team name, local docker image, local docker tag, upstream docker tag, and config file."
  echo "To push a new Docker image:"
  echo "  ./mbzirc_submit.sh <team_name> <local_docker_image> <local_docker_tag> <upstream_docker_tag> <config_file>"
  exit
fi

team=$1
image=$2
local_tag=$3
upstream_tag=$4
config_file=$5

# Check for the existance of your aws credentials
myAccessKeyId=`aws configure get aws_access_key_id`
mySecretAccessKey=`aws configure get aws_secret_access_key`
if [ -z "${myAccessKeyId}" ]; then
  echo "Your aws access key id is not set."
  exit
fi

if [ -z "${mySecretAccessKey}" ]; then
  echo "Your aws secret access key is not set."
  exit
fi

# Log into docker
echo "Logging into docker"
awsVersion=`aws --version`
if [[ "$awsVersion" == *"aws-cli/1"* ]]; then
  $(aws ecr get-login --no-include-email --region us-east-1)
elif [[ "$awsVersion" == *"aws-cli/2"* ]]; then
  eval 'aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin 200670743174.dkr.ecr.us-east-1.amazonaws.com'
else
  echo "Unsupported aws cli version $awsVersion"
  exit
fi

if [ $? != 0 ]; then
  echo "Failed to log into docker. Check your AWS credentials."
  exit
fi

# Tag the docker file
echo "Tagging image $image:$local_tag with $upstream_tag"
docker tag $image:$local_tag 200670743174.dkr.ecr.us-east-1.amazonaws.com/mbzirc/$team:$upstream_tag

# Docker push
echo "Pushing to docker repository"
docker push 200670743174.dkr.ecr.us-east-1.amazonaws.com/mbzirc/$team:$upstream_tag

echo "Docker image URL: 200670743174.dkr.ecr.us-east-1.amazonaws.com/mbzirc/$team:$upstream_tag"


echo "Uploading config file to s3"
aws s3 cp $config_file s3://cloudsim-mbzirc-logs/$team/config/config.yaml
