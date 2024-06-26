FROM osrf/ros:humble-desktop-full

WORKDIR /ws/

RUN apt install ros-humble-mavros
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh

