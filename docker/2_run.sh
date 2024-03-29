#!/bin/bash
set -e

echo "running image cosemap"
echo "creating container cosemap"

# Intial variable setup
#DOCKER_CUSTOM_ARGS=$@
DOCKER_DEFAULT_ARGS=()
DOCKER_DEFAULT_ARGS+=("-it")
DOCKER_USER_HOME="/root"

# Default: Disposable Container
PERSISTENT_CONTAINER=0

if [[ $PERSISTENT_CONTAINER -eq 0 ]];then
    DOCKER_DEFAULT_ARGS+=("--rm")
    echo "Using disposable container"
    echo ""
else
    echo "Saving container under: $CONTAINER_NAME"
    echo ""
fi

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Get the current version of docker-ce
# Strip leading stuff before the version number so it can be compared
DOCKER_VER=$(dpkg-query -f='${Version}' --show docker-ce | sed 's/[0-9]://')
if dpkg --compare-versions 19.03 gt "$DOCKER_VER"
then
    echo "Docker version is less than 19.03, using nvidia-docker2 runtime"
    if ! dpkg --list | grep nvidia-docker2
    then
        echo "Please either update docker-ce to a version greater than 19.03 or install nvidia-docker2"
	exit 1
    fi
    DOCKER_DEFAULT_ARGS="$DOCKER_DEFAULT_ARGS --runtime=nvidia"
else
    DOCKER_DEFAULT_ARGS="$DOCKER_DEFAULT_ARGS --gpus all"
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]
then
  echo "[$XAUTH] was not properly created. Exiting..."
  exit 1
fi

# Device and audio passtrough
DOCKER_DEFAULT_ARGS+=("--privileged")
DOCKER_DEFAULT_ARGS+=("-e 'TERM=xterm-256color'")
DOCKER_DEFAULT_ARGS+=("-e 'RCUTILS_COLORIZED_OUTPUT=1'")
DOCKER_DEFAULT_ARGS+=("--network host")

DOCKER_DEFAULT_ARGS+=("--memory 3000M")
DOCKER_DEFAULT_ARGS+=("-v /dev:/dev")
DOCKER_DEFAULT_ARGS+=("-v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket")

if [ ! -d "/var/run/mysqld" ]; then
    echo "MariaDB not installed in host machine."
    echo "Please run: bash scripts/install_mariadb_server_host.sh"
    exit
fi

DOCKER_DEFAULT_ARGS+=("-v /var/run/mysqld/mysqld.sock:/var/run/mysqld/mysqld.sock")
DOCKER_DEFAULT_ARGS+=("-v $(pwd)/../../../:/home/cosemap/ws")
# Edit to mount bag files inside container 
# DOCKER_DEFAULT_ARGS+=("-v /path/to/bags/directory/on/host:/home/cosemap/bags")


# detatched mode
DOCKER_DEFAULT_ARGS+=("-d")

DOCKER_DEFAULT_ARGS+=("--gpus all")
DOCKER_DEFAULT_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_DEFAULT_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_DEFAULT_ARGS+=("--runtime nvidia")


echo "Container initialized with following arguments:"
echo "docker run "${DOCKER_DEFAULT_ARGS[@]} --name cosemap cosemap
echo ""

docker run \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  ${DOCKER_DEFAULT_ARGS[@]} \
  --env=DISPLAY=${DISPLAY} \
  --env=TERM="xterm-color" \
  --name cosemap cosemap ;

docker exec cosemap chown -R mysql:root /run/mysqld/mysqld.sock # /var/run/mysqld/