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
 && rm -rf /var/lib/apt/lists/* \
 && apt-get -qq clean

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -fs /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get -qq update && apt-get -q -y install tzdata \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Tools useful during development
RUN apt-get update -qq \
 && apt-get install --no-install-recommends -y -qq \
        build-essential \
        atop \
        cmake \
        cppcheck \
        expect \
        gdb \
        git \
        gnutls-bin \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
        libfcl-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        software-properties-common \
        vim \
        net-tools \
        iputils-ping \
        xvfb \
        curl \
 && apt-get clean -qq

# set up ros2 repo
RUN /bin/sh -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros.list > /dev/null'

# install rosdep
RUN apt-get update \
  && apt install -y python3-rosdep \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq \
  && rosdep init \
  && rosdep update

# install ROS2
RUN apt-get update \
  && apt-get install -y ros-galactic-ros-base ros-galactic-ros1-bridge \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

# install ROS
RUN apt-get update \
  && apt-get install -y ros-noetic-ros-base \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

# install colcon
RUN apt-get -qq update && apt-get -q -y install \
  python3-vcstool \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

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

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

COPY mbzirc_ros1_bridge/run_bridge.bash ./run_bridge.bash
ENTRYPOINT ["./run_bridge.bash"]

