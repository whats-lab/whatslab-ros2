FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/ros2_ws
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src/airglove_ros2

COPY requirements.txt ${COLCON_WS}/src/airglove_ros2/
COPY airglove_msgs/package.xml ${COLCON_WS}/src/airglove_ros2/airglove_msgs/
COPY airglove_driver/package.xml airglove_driver/requirements.txt ${COLCON_WS}/src/airglove_ros2/airglove_driver/
COPY airglove_retargeting/package.xml airglove_retargeting/requirements.txt ${COLCON_WS}/src/airglove_ros2/airglove_retargeting/

RUN apt-get update && \
    rosdep update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        swig \
        libx11-6 \
        libxcursor1 \
        libxinerama1 \
        libxi6 \
        libxrandr2 \
        libxrender1 \
        libxfixes3 \
        libgl1 \
        libgles2 \
        libgl1-mesa-dri \
        libvulkan1 \
        libwayland-client0 && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y \
        --skip-keys="ament_python dexhand_description" && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages --default-timeout=1000 \
        -r ${COLCON_WS}/src/airglove_ros2/requirements.txt

COPY . ${COLCON_WS}/src/airglove_ros2/

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-up-to airglove_retargeting airglove_driver"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install --packages-up-to airglove_retargeting airglove_driver'" >> ~/.bashrc 
CMD ["bash"]
