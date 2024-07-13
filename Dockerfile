# Use the official ROS2 Humble base image
#FROM osrf/ros:humble-desktop
ARG IMAGE_NAME=dustynv/ros:humble-desktop-l4t-r35.4.1 
FROM ${IMAGE_NAME}

# Set environment variables
ENV ROS_DISTRO=humble

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip 
#    ros-humble-ros-base \
#    ros-humble-teleop-twist-keyboard

# Joystick-driver dependencies
RUN apt-get update && apt-get install -y \
     libspnav-dev \
     libbluetooth-dev \
     libcwiid-dev \
     evtest

# Create a workspace directory
WORKDIR /ros2_ws/src

#RUN source /opt/ros/$ROS_DISTRO/install/setup.bash \
#    && source /opt/ros/$ROS_DISTRO/install/local_setup.bash \
#    && ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Clone the necessary repositories
#RUN git clone https://github.com/ros2/teleop_twist_joy.git -b humble\
#    && git clone https://github.com/ros2/teleop_twist_keyboard.git \
#    && git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2 \
#    && git clone https://github.com/ros-controls/ros2_control.git -b humble


# Install dependencies and build the workspace
#WORKDIR /ros2_ws
#RUN source /opt/ros/$ROS_DISTRO/install/setup.bash \
#    && source /opt/ros/$ROS_DISTRO/install/local_setup.bash \
#    && rosdep update --rosdistro $ROS_DISTRO \
#    && rosdep install --from-paths src --ignore-src -r -y \
#    && colcon build

# Source the workspace
#RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
#SHELL ["/bin/bash", "-c"]

# Add PCA9865 Python package or library.
RUN python3 -m pip install --upgrade pip && \ 
    pip3 install smbus2

#WORKDIR /ros2_ws/src
#RUN git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git \
#    && cd Adafruit_Python_PCA9685 \
#    && python3 setup.py install

# Set entrypoint for the container
#ENTRYPOINT ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && roslaunch your_launch_file.launch"]

#RUN apt-get update && apt-get install -y \
#    libspnav-dev \
#    libbluetooth-dev \
#    libcwiid-dev

#WORKDIR /root/ros2_ws/src && \
#    git clone https://github.com/ros-drivers/joystick_drivers.git -b ros2

# Set up your ROS2 workspace
#WORKDIR /root/ros2_ws
#COPY ros2_ws/src ./src
#RUN . /opt/ros/humble/setup.sh && \
#    colcon build

# Copy configuration files
#COPY config ./config
#COPY launch ./launch

WORKDIR /root/ros2_ws/src
#COPY i2c_pwm_board/ src/i2c_pwm_board
#RUN git clone --recursive https://github.com/vertueux/i2c_pwm_board
#RUN git clone --recursive https://github.com/rosblox/pca9685_ros2_control
RUN git clone https://github.com/artoo-ai/ros-pwm-pca9685.git -b ros2

# Install dependencies
#RUN chmod +x i2c_pwm_board/scripts/install_dependencies.sh && \
#    i2c_pwm_board/scripts/install_dependencies.sh

WORKDIR /root/ros2_ws
# Build the package
RUN source /opt/ros/$ROS_DISTRO/install/setup.bash && \ 
    colcon build

# Setup environment variables 
COPY ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
RUN sudo chmod 755 /sbin/ros_entrypoint.sh

# Source the ROS2 setup and entrypoint
ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]