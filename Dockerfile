FROM ubuntu:22.04 as cacher

# Copy build context into overlay
WORKDIR /opt/ros/overlay_ws
COPY . .

# Ensure that the shell fails due to errors in any stage of a pipe
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Copy manifests for caching
WORKDIR /opt
RUN mkdir -p /tmp/opt && \
    find ./ -name "package.xml" -print0 | \
      xargs -0 cp --parents -t /tmp/opt

FROM ubuntu:22.04 as base

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install ROS2 Humble and build dependencies
# hadolint ignore=SC2086
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
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
      libopencv-dev \
      libceres-dev && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/CheranMahalingam/DBow3.git && \
    mkdir DBow3/build && \
    cmake DBow3 -B DBow3/build && \
    cmake --build DBow3/build && \
    cp DBow3/build/src/libDBoW3.so /usr/local/lib/ && \
    mkdir /usr/local/include/DBoW3 && \
    cp DBow3/src/*.h /usr/local/include/DBoW3/ && \
    rm -rf DBow3

# Copy cached manifests to install dependencies defined by rosdep keys
COPY --from=cacher /tmp/opt/ros/overlay_ws /tmp
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep init && \
    rosdep update && \
    rosdep install -y --from-paths /tmp/src --ignore-src && \
    rm -rf /var/lib/apt/lists/*

FROM base as devel

ARG HOST_UID=1000
ARG HOST_GID=1000

# Install useful development tools
RUN apt-get update && apt-get install -y \
    ntp \
    ninja-build \
    gdb \
    gdbserver \
    tmux \
    vim \
    ripgrep \
    fish && \
    rm -rf /var/lib/apt/lists/*

# Create a user with uid and gid matching the host and grant sudo access
RUN addgroup --gid $HOST_GID docker && \
    adduser --uid $HOST_UID --ingroup docker --home /home/docker --shell /usr/bin/fish --disabled-password --gecos "" docker && \
    echo "docker ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/docker && \
    chmod 0440 /etc/sudoers.d/docker
USER docker:docker

WORKDIR /home/docker/ros2_ws

# Setup development tools tied to user shell
RUN curl -L https://get.oh-my.fish > install && \
    fish install --noninteractive && \
    fish -c "omf install bass" && \
    rm install && \
    git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && \
    ~/.fzf/install && \
    wget -P ~ https://github.com/cyrus-and/gdb-dashboard/raw/master/.gdbinit && \
    echo "target remote localhost:3000" > ~/.gdbinit

RUN <<EOF cat>> ~/.config/fish/config.fish
# Workaround for newer git versions where git verifies that the parent directory
# is owned by an identical user
git config --global --add safe.directory /home/docker/ros2_ws
bass source /opt/ros/"${ROS_DISTRO}"/setup.bash
EOF

EXPOSE 8765

CMD ["fish"]

FROM base as prod
