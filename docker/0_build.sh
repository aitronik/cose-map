#!/bin/bash
set -e


rm -rf tmp

# Getting packages to build inside docker
mkdir -p tmp && cd tmp
mkdir -p opencv && cd opencv
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-4.x opencv
mv opencv_contrib-4.x opencv_contrib

cd ../
wget https://dlm.mariadb.com/3700662/Connectors/cpp/connector-cpp-1.0.3/mariadb-connector-cpp-1.0.3-ubuntu-focal-amd64.tar.gz
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.13.1.tar.gz
wget https://r.mariadb.com/downloads/mariadb_repo_setup

cd ../

DOCKER_BUILD_ARGS=()

# Map group and user id
DOCKER_BUILD_ARGS+=("--network host")
DOCKER_BUILD_ARGS+=("--build-arg USER_ID=$(id -u)")
DOCKER_BUILD_ARGS+=("--build-arg GROUP_ID=$(id -g)")
DOCKER_BUILD_ARGS+=("--progress=plain")


if docker inspect cosemap_opencv:latest &> /dev/null; then
    echo "docker image cosemap_opencv:latest already exists"
else
    docker build ${DOCKER_BUILD_ARGS[@]} -f Dockerfile.cosemap_opencv -t cosemap_opencv . 
fi

if docker inspect cosemap_pcl:latest &> /dev/null; then
    echo "docker image cosemap_pcl:latest already exists"
else
    docker build ${DOCKER_BUILD_ARGS[@]} -f Dockerfile.cosemap_pcl -t cosemap_pcl .
fi

if docker inspect cosemap_mariadb:latest &> /dev/null; then
    echo "docker image cosemap_mariadb:latest already exists"
else
    docker build ${DOCKER_BUILD_ARGS[@]} -f Dockerfile.cosemap_mariadb -t cosemap_mariadb .
fi

if docker inspect cosemap_ros:latest &> /dev/null; then
    echo "docker image cosemap_ros:latest already exists"
else
    docker build ${DOCKER_BUILD_ARGS[@]} -f Dockerfile.cosemap_ros -t cosemap_ros .
fi

if docker inspect cosemap:latest &> /dev/null; then
    echo "docker image cosemap:latest already exists"
else
    docker build ${DOCKER_BUILD_ARGS[@]} -f Dockerfile.cosemap -t cosemap .
    echo "cosemap image correctly built!"
fi

rm -rf tmp