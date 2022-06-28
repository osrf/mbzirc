# Ubuntu 20.04 with nvidia-docker2 beta opengl support
FROM osrf/mbzirc:mbzirc_models_latest

USER root

ARG DEBIAN_FRONTEND=noninteractive

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
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'

# install ignition fortress
RUN apt-get update \
  && apt-get install -y ignition-fortress \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

# install rosdep
RUN apt-get update \
  && apt install -y python3-rosdep \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq \
  && rosdep init \
  && rosdep update

# install ROS2
RUN apt-get update \
  && apt-get install -y ros-galactic-ros-base \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get clean -qq

# install colcon
RUN apt-get -qq update && apt-get -q -y install \
  python3-vcstool \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/* \
  && apt-get -qq clean

# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Prepare the colcon workspace
RUN mkdir -p mbzirc_ws/src

# clone ros_ign bridge
RUN cd /home/$USERNAME/mbzirc_ws/src \
 && git clone https://github.com/osrf/ros_ign.git -b galactic

WORKDIR /home/$USERNAME/mbzirc_ws

COPY . src/mbzirc

ENV IGNITION_VERSION fortress

RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -r --from-paths src -i -y --rosdistro galactic \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt-get clean -qq

# build mbzirc_seed
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash \
  && colcon build --merge-install --packages-select ros_ign_interfaces mbzirc_seed'

RUN /bin/sh -c 'echo ". /opt/ros/galactic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/mbzirc_ws/install/setup.sh" >> ~/.bashrc'

# Copy entry point script, and set the entrypoint
COPY docker/mbzirc_seed/run_solution.bash ./
ENTRYPOINT ["./run_solution.bash"]
