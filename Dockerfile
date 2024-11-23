FROM osrf/ros:noetic-desktop-full

# Update and install necessary tools
RUN apt-get update && apt-get install -y \
    git \
    python3-pip && \
    apt-get clean

# Install ROS packages: effort_controllers and teleop_twist_keyboard
RUN apt-get update && apt-get install -y \
    ros-noetic-effort-controllers \
    ros-noetic-teleop-twist-keyboard && \
    apt-get clean

# Set up the Catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Copy the local src folder into the container
COPY ./src /root/catkin_ws/src

# Build the workspace (only if a valid src folder exists)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    if [ -f src/CMakeLists.txt ]; then catkin_make; else echo 'No valid src found, skipping build'; fi"

# Source ROS setup files automatically
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Final message to confirm the setup
RUN echo "ALL Done"

# Default command
CMD ["bash"]
