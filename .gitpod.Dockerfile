FROM ros:melodic-ros-base-bionic

ENV ROS_DISTRO=melodic
ENV PYLON_ROOT=/opt/pylon
ENV PYLON_URl=https://www.baslerweb.com/fp-1589378344/media/downloads/software/pylon_software/pylon_6.1.1.19861-deb0_amd64.deb

RUN sudo apt update && DEBIAN_FRONTEND=noninteractive sudo apt install -y \
    wget \
    && rm -rf /var/lib/apt/lists/*

# install pylon
RUN wget -O pylon.deb $PYLON_URl && sudo dpkg -i pylon.deb && $PYLON_ROOT/bin/pylon-config --version-major

# install ros dependencies
RUN sudo apt-get install -y \
    python-catkin-tools python-catkin-pkg python-rosdep python-wstool \
    ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-nodelet-core ros-$ROS_DISTRO-ddynamic-reconfigure frei0r-plugins-dev

# prepare rosdep
RUN sudo rosdep init && \
    rosdep update --include-eol-distros  # Support EOL distros.

# install pylon ros dependencies
RUN sudo sh -c 'echo "yaml https://raw.githubusercontent.com/i3drobotics/pylon_camera/master/rosdep/pylon_sdk.yaml " > /etc/ros/rosdep/sources.list.d/15-plyon_camera.list' && \
    rosdep update --include-eol-distros

# setup ros workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws && \
    catkin build && \
    wstool init src https://raw.githubusercontent.com/i3drobotics/i3dr_titania-ros/master/install/i3dr_titania_https.rosinstall && \
    rosdep install --from-paths src --ignore-src -r -y
