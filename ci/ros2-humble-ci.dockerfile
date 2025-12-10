FROM ros:humble-ros-base-jammy

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-humble-image-tools \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-vision-msgs && \
    rm -rf /var/lib/apt/lists/*

