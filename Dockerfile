FROM osrf/ros:noetic-desktop-full

# Update and install necessary tools
RUN apt-get update && apt-get install -y \
    git \
    software-properties-common \
    python3-pip && \
    apt-get clean

# Install Python 3.8 and Python 3.9
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes && \
    apt-get update && \
    apt-get install -y python3.8 python3.8-distutils python3.8-venv python3.9 python3.9-distutils python3.9-venv python3-pip && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --config python3 && \
    python3 -m pip install --upgrade pip

# Install ROS packages: effort_controllers, teleop_twist_keyboard, and foxglove_bridge
RUN apt-get update && apt-get install -y \
    ros-noetic-effort-controllers \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-foxglove-bridge && \
    apt-get clean

# Set up the Catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Copy the local src folder and requirements
COPY ./src /root/catkin_ws/src
COPY ./gemini_run.py /root/
COPY ./requirements.txt /root/

# Build the workspace (only if a valid src folder exists)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    if [ -f src/CMakeLists.txt ]; then catkin_make; else echo 'No valid src found, skipping build'; fi"

# Install Python dependencies with Python 3.8
RUN python3.8 -m pip install -r /root/requirements.txt

# Source ROS setup files automatically
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Add Foxglove Bridge to ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Default command
CMD roslaunch foxglove_bridge foxglove_bridge.launch & bash
