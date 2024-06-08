FROM ubuntu:22.04 as base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

# Ensure that the shell fails due to errors in any stage of a pipe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install ROS2 Humble and build dependencies
# hadolint ignore=SC2086
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get upgrade && \
    apt-get install -y \
      ros-${ROS_DISTRO}-ros-base \
      ros-dev-tools \
      libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]

FROM base as devel

ARG HOST_UID=1000
ARG HOST_GID=1000

RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    ripgrep \
    fish && \
    rm -rf /var/lib/apt/lists/*

# Create a user with uid and gid matching the host and with sudo priveleges
RUN addgroup --gid $HOST_GID docker && \
    adduser --uid $HOST_UID --ingroup docker --home /home/docker --shell /usr/bin/fish --disabled-password --gecos "" docker && \
    echo "docker ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/docker && \
    chmod 0440 /etc/sudoers.d/docker
USER docker:docker

RUN mkdir -p /home/docker/ros2_ws
WORKDIR /home/docker/ros2_ws

# Setup fish with bass for running bash scripts
RUN curl -L https://get.oh-my.fish > install && \
    fish install --noninteractive && \
    fish -c "omf install bass" && \
    rm install

RUN <<EOF cat>> ~/.config/fish/config.fish
# Workaround for newer git versions where git verifies that the parent directory
# is owned by an identical user
git config --global --add safe.directory /home/docker/ros2_ws
bass source /opt/ros/"${ROS_DISTRO}"/setup.bash
EOF

CMD ["fish"]

FROM base as prod
