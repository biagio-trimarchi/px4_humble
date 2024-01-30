# Ros image
FROM osrf/ros:humble-desktop

### SOME SETUP
# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive

# Make apt to always ignore recommended and suggested packages
# This is particularly important with rosdep which invoked apt without `--no-install-recommends`
RUN echo \
  'APT::Install-Recommends "0";\nAPT::Install-Suggests "0";' > /etc/apt/apt.conf.d/01norecommend

# Update system
RUN apt update && apt upgrade -y

# Install sudo
RUN apt-get update && apt-get install -q -y --no-install-recommends sudo && \
  rm -rf /var/lib/apt/lists/*

### CREATE NEW USER
# Build-time arguments
ARG USERNAME=docker-dev
ARG GROUPNAME=$USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Set username and home environment variables
ENV USERNAME $USERNAME
ENV HOME /home/$USERNAME

# Create a new user with the provided details
RUN groupadd --gid $USER_GID $GROUPNAME && \
  useradd --create-home --home-dir /home/$USERNAME --shell /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME

# Add the new user to the sudoers group with no password
RUN echo "$USERNAME:x:$USER_UID:$USER_GID:Developer,,,:$HOME:/bin/bash" >> /etc/passwd && \
  echo "$USERNAME:x:$USER_GID:" >> /etc/group && \
  echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
  chmod 0440 /etc/sudoers.d/$USERNAME && \
  chown $USER_UID:$USER_GID -R $HOME

### Install the necessary stuff
# Change user
USER $USERNAME:$GROUPNAME

# Install text editors
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install nano -y
RUN sudo apt install vim -y

# Other useful stuff
RUN sudo apt install curl -y
RUN sudo apt install wget -y
RUN sudo apt install xterm -y

# Install python depencies
RUN sudo apt install python3-pip -y
RUN pip3 install open3d 
RUN pip3 install rosbags
RUN pip3 install tk
RUN sudo apt install python3-tk

# Install PCL 
# Installation from sources
WORKDIR /home/$USERNAME
RUN git clone https://github.com/PointCloudLibrary/pcl.git
WORKDIR /home/$USERNAME/pcl
RUN mkdir build 
WORKDIR /home/$USERNAME/pcl/build
RUN cmake ../
RUN sudo make -j1 install
WORKDIR /home/$USERNAME/
RUN sudo rm -r pcl

# Install PX4
WORKDIR /home/$USERNAME
RUN mkdir PX4/
WORKDIR /home/$USERNAME/PX4/
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
WORKDIR /home/$USERNAME/PX4/PX4-Autopilot
RUN git checkout tags/v1.14.0
RUN git submodule sync --recursive 
RUN git submodule update --init --recursive
RUN ./Tools/setup/ubuntu.sh

# Download PX4_ROS2 interface 
WORKDIR /home/$USERNAME/PX4/
RUN mkdir px4_ros_interface
WORKDIR /home/$USERNAME/PX4/px4_ros_interface
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN git clone https://github.com/PX4/px4_msgs.git
RUN colcon build

# Download UXRCE Agent
WORKDIR /home/$USERNAME
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /home/$USERNAME/Micro-XRCE-DDS-Agent
RUN mkdir build
WORKDIR /home/$USERNAME/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make
RUN sudo make install
RUN sudo ldconfig /usr/local/lib/
WORKDIR /home/$USERNAME
RUN sudo rm -r Micro-XRCE-DDS-Agent

# Install some ROS2 dependencies
RUN rosdep update # --include-eol-distros
RUN sudo apt install ros-${ROS_DISTRO}-octomap -y
RUN sudo apt install ros-${ROS_DISTRO}-octomap-server -y
RUN sudo apt install ros-${ROS_DISTRO}-octomap-mapping -y
RUN sudo apt install ros-${ROS_DISTRO}-ros-gz -y
# RUN sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs -y
RUN sudo apt install ros-${ROS_DISTRO}-pcl-msgs -y
RUN sudo apt install ros-${ROS_DISTRO}-perception-pcl -y

# Install RealSense Dependencies (https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /home/$USERNAME/librealsense
RUN git checkout tags/v2.50.0
RUN sudo apt install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake -y
RUN sudo apt install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y
#RUN ./scripts/patch-realsense-ubuntu-lts.sh

RUN mkdir build
WORKDIR /home/$USERNAME/librealsense/build
RUN cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
RUN sudo make uninstall && make clean && make && sudo make install
WORKDIR /home/$USERNAME/
RUN sudo rm -r librealsense

# Install OpenCV
RUN git clone https://github.com/opencv/opencv.git
WORKDIR /home/$USERNAME/opencv
# RUN git checkout tags/4.2.0
RUN mkdir build
WORKDIR /home/$USERNAME/opencv/build
RUN cmake ../
RUN make && sudo make install
WORKDIR /home/$USERNAME/
RUN sudo rm -r opencv

# Install ROS Humble - Gazebo Garden Bridge
ENV GZ_VERSION=garden
sudo apt install ros-humble-actuator-msgs
sudo apt install ros-humble-vision-msgs
sudo apt install libgflags-dev
WORKDIR /home/$USERNAME/
RUN mkdir Gazebo/
WORKDIR /home/$USERNAME/Gazebo
RUN git clone -b humble https://github.com/gazebosim/ros_gz.git
WORKDIR /home/$USERNAME/Gazebo/ros_gz
RUN colcon build

# Install qpOASES (C++ QP solver)
WORKDIR /home/$USERNAME/
git clone https://github.com/coin-or/qpOASES.git
WORKDIR /home/$USERNAME/qpOASES
RUN mkdir build
WORKDIR /home/$USERNAME/qpOASES/build
RUN cmake .. && make && sudo make install
WORKDIR /home/$USERNAME/
RUN sudo rm -r qpOASES

### Final setup
WORKDIR /home/$USERNAME
RUN printf "\n#Source ROS\nsource /opt/ros/${ROS_DISTRO}/setup.bash" >> .bashrc
RUN printf "\n#Source PX4 ROS interface\nsource ~/PX4/px4_ros_interface/install/setup.bash" >> .bashrc
RUN printf "\n#Source Gazebo ROS interface\nsource ~/Gazebo/ros_gz/install/setup.bash" >> .bashrc
RUN printf "\n\n#Fix display\nexport DISPLAY=:0" >> .bashrc
