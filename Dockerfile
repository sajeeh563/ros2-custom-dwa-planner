# Use official ROS 2 Humble base image
FROM ros:humble-ros-base

# -------------------------------
# ENVIRONMENT VARIABLES
# -------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV USER=student
ENV HOME=/home/$USER
ENV WORKSPACE=/ros2_turtlebot3_ws

# -------------------------------
# STEP 1: TEMPORARILY ALLOW INSECURE APT TO INSTALL TOOLS
# -------------------------------
RUN apt-get update -o Acquire::AllowInsecureRepositories=true && \
    apt-get install -y --allow-unauthenticated \
    curl gnupg2 lsb-release locales tzdata \
    git wget build-essential \
    python3-pip python3-colcon-common-extensions \
    x11-apps mesa-utils libgl1-mesa-glx \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# STEP 2: SET LOCALE
# -------------------------------
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# -------------------------------
# STEP 3: DOWNLOAD FRESH ROS KEY AND OVERWRITE EXISTING ONE
# (DO NOT add new repo — just replace the key!)
# -------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor > /usr/share/keyrings/ros-archive-keyring.gpg

# ⚠️ CRITICAL: Find existing key path and overwrite it
# Base image likely uses this path:
RUN cp /usr/share/keyrings/ros-archive-keyring.gpg /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# -------------------------------
# STEP 4: UPDATE APT AND INSTALL ROS/TURTLEBOT3 PACKAGES
# -------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# INSTALL PYTHON DEPENDENCIES
# -------------------------------
RUN pip3 install --no-cache-dir numpy

# -------------------------------
# CREATE NON-ROOT USER
# -------------------------------
RUN groupadd -r $USER && useradd -m -r -g $USER $USER
RUN echo "$USER:$USER" | chpasswd
RUN usermod -aG sudo $USER
RUN mkdir -p $WORKSPACE/src && chown -R $USER:$USER $WORKSPACE

# -------------------------------
# SOURCE ROS + SET ENV IN .bashrc
# -------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> $HOME/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> $HOME/.bashrc
RUN echo "export HOME=$HOME" >> $HOME/.bashrc
RUN echo "alias cb='colcon build --symlink-install'" >> $HOME/.bashrc
RUN echo "source $WORKSPACE/install/setup.bash" >> $HOME/.bashrc

# -------------------------------
# SWITCH TO NON-ROOT USER
# -------------------------------
USER $USER
WORKDIR $HOME

# -------------------------------
# DEFAULT COMMAND
# -------------------------------
CMD ["/bin/bash"]