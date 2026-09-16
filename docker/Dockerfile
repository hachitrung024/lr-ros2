# Landfill Rover desktop/Jetson development environment.
# The base is produced by the corresponding ZED wrapper Docker builder.
ARG BASE_IMAGE=zed_ros2_desktop:latest
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG LR_PLATFORM=desktop
ARG TORCH_INDEX_URL=""
ARG TORCH_SPEC="torch>=2.1,<3"
ARG TORCHVISION_SPEC="torchvision"

USER root
COPY requirements-lr.txt /tmp/requirements-lr.txt
COPY scripts/install_lr_environment.sh /tmp/install_lr_environment.sh

RUN chmod 755 /tmp/install_lr_environment.sh && \
    /tmp/install_lr_environment.sh \
      /tmp/requirements-lr.txt \
      "${LR_PLATFORM}" \
      "${TORCH_INDEX_URL}" \
      "${TORCH_SPEC}" \
      "${TORCHVISION_SPEC}" && \
    rm -f /tmp/install_lr_environment.sh /tmp/requirements-lr.txt

WORKDIR /workspace

# ENTRYPOINT and CMD are inherited from the ZED wrapper image. That entrypoint
# sources the ROS and ZED wrapper workspace. LR source code, build products,
# models, recordings, and databases are mounted by the user at runtime.
