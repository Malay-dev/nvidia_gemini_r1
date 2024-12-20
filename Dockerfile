FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    git \
    software-properties-common \
    python3-pip && \
    apt-get clean

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:deadsnakes && \
    apt-get update && \
    apt-get install -y python3.8 python3.8-distutils python3.8-venv python3.9 python3.9-distutils python3.9-venv python3-pip && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 2 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --config python3 && \
    python3 -m pip install --upgrade pip

RUN apt-get update && apt-get install -y \
    ros-noetic-effort-controllers \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-foxglove-bridge && \
    apt-get clean

RUN mkdir -p /root/catkin_ws/src

COPY ./src /root/catkin_ws/src
COPY ./gemini_run.py /root/
COPY ./requirements.txt /root/

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    if [ -f src/CMakeLists.txt ]; then catkin_make; else echo 'No valid src found, skipping build'; fi"

RUN python3.8 -m pip install -r /root/requirements.txt

RUN python3.9 -m pip install -r /root/requirements.txt

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

CMD roslaunch foxglove_bridge foxglove_bridge.launch & bash
