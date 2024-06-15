#!/usr/bin/env bash
set -eou pipefail

running_containers=$(docker ps |
  { grep -c "atlas-robot" || test $? = 1; })
if [[ "$running_containers" -eq 0 ]]
then
  exited_containers=$(docker ps -a |
    { grep -c "atlas-robot" || test $? = 1; })
  if [[ "$exited_containers" -gt 0 ]]
  then
    echo "Killing the atlas-robot container..."
    # Filter for all containers with name "atlas-robot".
    docker ps -a |
      awk '{if ($NF == "atlas-robot") print $1}' |
      xargs docker rm
  fi
  echo "Building the atlas image..."
  if [[ "$(uname -s)" == "Darwin" ]]
  then
    HOST_UID=1000
    HOST_GID=1000
  else
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
  fi
  docker build -t atlas . \
    --build-arg HOST_UID="$HOST_UID" \
    --build-arg HOST_GID="$HOST_GID" \
    --target devel
  echo "Initializing the atlas-robot container..."
  docker run -it \
    --user "$HOST_UID":"$HOST_GID" \
    --name atlas-robot \
    --network host \
    --mount type=bind,source="$(pwd)",target=/home/docker/ros2_ws \
    atlas:latest
else
    echo "Found an instance of the atlas image running, attaching..."
    docker attach atlas-robot
fi
