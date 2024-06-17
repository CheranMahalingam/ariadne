#!/usr/bin/env bash
set -eou pipefail

running_containers=$(docker ps |
  { grep -c "ariadne" || test $? = 1; })
if [[ "$running_containers" -eq 0 ]]
then
  exited_containers=$(docker ps -a |
    { grep -c "ariadne" || test $? = 1; })
  if [[ "$exited_containers" -gt 0 ]]
  then
    echo "Killing the ariadne container..."
    # Filter for all containers with name "ariadne".
    docker ps -a |
      awk '{if ($NF == "ariadne") print $1}' |
      xargs docker rm
  fi
  echo "Building the ariadne image..."
  if [[ "$(uname -s)" == "Darwin" ]]
  then
    HOST_UID=1000
    HOST_GID=1000
  else
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
  fi
  docker build -t ariadne . \
    --build-arg HOST_UID="$HOST_UID" \
    --build-arg HOST_GID="$HOST_GID" \
    --target devel
  echo "Initializing the ariadne container..."
  docker run -it \
    --user "$HOST_UID":"$HOST_GID" \
    --name ariadne \
    --mount type=bind,source="$(pwd)",target=/home/docker/ros2_ws \
    ariadne:latest
else
    echo "Found an instance of the ariadne image running, attaching..."
    docker exec -it ariadne fish
fi
