FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-serial \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY 2026R2_ws /workspace/2026R2_ws

RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && \
    cd /workspace/2026R2_ws && \
    colcon build --packages-select base_omniwheel_r2_700 --symlink-install"

CMD ["/bin/bash"]
