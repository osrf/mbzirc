# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install --no-install-recommends -y \
    tzdata \
    sudo \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
 && rm -rf /var/lib/apt/lists/* \
 && apt-get -qq clean

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get -qq update && apt-get -q -y install tzdata \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Set up repo to install Ignition
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# install fuel-tools, download the public models, and uninstall fuel-tools again to keep the image smaller
RUN sudo apt-get update \
 && sudo apt-get install -y \
    libignition-fuel-tools7-dev \
 && ign fuel download -v 4 -j 8 --type model -u "https://fuel.ignitionrobotics.org/OpenRobotics/collections/mbzirc" \
 && sudo apt-get remove libignition-fuel-tools7-dev -y \
 && sudo apt-get autoremove -y \
 && sudo apt-get clean
