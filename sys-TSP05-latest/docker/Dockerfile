# f1tenth_gym_ros image + what the TSP05 serial bridge needs:
# pyserial (Arduino USB serial) and foxglove_bridge (Foxglove GUI).
FROM f1tenth_gym_ros:latest

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-serial \
        ros-humble-foxglove-bridge && \
    rm -rf /var/lib/apt/lists/*

# Source ROS (and the TSP05 workspace once built) in every interactive shell
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /tsp05/ros2_ws/install/setup.bash ] && source /tsp05/ros2_ws/install/setup.bash' >> /root/.bashrc
