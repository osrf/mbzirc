# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04

ENV DEBIAN_FRONTEND noninteractive
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update \
 && apt-get install --no-install-recommends -y \
    tzdata \
    sudo \
    wget \
    gnupg2 \
    lsb-release

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
        mesa-utils \
        libgl1-mesa-glx \
 && apt-get clean -qq

# set up ros2 repo
RUN /bin/sh -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# Set up repo to install Ignition
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'

# install ignition fortress
RUN apt-get update \
  &&  apt-get install -y ignition-fortress \
  && apt-get clean

# install rosdep
RUN apt update \
  && apt install -y python3-rosdep \
  && rosdep init \
  && rosdep update \
  && apt-get clean

# install ROS2
RUN apt-get update \
  &&  apt-get install -y ros-galactic-ros-base \
  && apt-get clean

# install colcon
RUN apt-get -qq update && apt-get -q -y install \
  python3-vcstool \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ENV USERNAME developer
RUN useradd -U -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# clone mbzirc sim
RUN mkdir -p mbzirc_ws/src \
 && cd mbzirc_ws/src

# clone ros_ign bridge
RUN cd /home/$USERNAME/mbzirc_ws/src \
 && git clone https://github.com/osrf/ros_ign.git -b ros2

WORKDIR /home/$USERNAME/mbzirc_ws

COPY . src/mbzric

ENV IGNITION_VERSION fortress

RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -r --from-paths src -i -y --rosdistro galactic

RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash && colcon build --merge-install'

# RUN /bin/bash -c 'Xvfb :1 -ac -noreset -core -screen 0 1280x1024x24 &'

# ENV DISPLAY :1.0
ENV RENDER_ENGINE_VALUES ogre2
ENV MESA_GL_VERSION_OVERRIDE 3.3

RUN /bin/bash -c 'source /home/$USERNAME/mbzirc_ws/install/setup.bash && colcon test --merge-install --event-handlers console_direct+ --packages-select mbzirc_ros'
RUN /bin/bash -c 'source /home/$USERNAME/mbzirc_ws/install/setup.bash && colcon test-result'
