FROM ros:humble

WORKDIR /workspace

COPY . .

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

CMD ["bash"]
