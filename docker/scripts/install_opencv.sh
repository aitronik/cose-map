set -e

export DEBIAN_FRONTEND=noninteractive
apt-get update && apt-get install -y tzdata
export DEBIAN_FRONTEND=dialog

apt-get install -y \
	    python3-pip \
        build-essential \
        cmake \
        git \
        wget \
        unzip \
        yasm \
        pkg-config \
        libswscale-dev \
        libtbb2 \
        libtbb-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libavformat-dev \
        libpq-dev \
        libxine2-dev \
        libglew-dev \
        libtiff5-dev \
        zlib1g-dev \
        libjpeg-dev \
        libavcodec-dev \
        libavformat-dev \
        libavutil-dev \
        libpostproc-dev \
        libswscale-dev \
        libeigen3-dev \
        libtbb-dev \
        libgtk2.0-dev \
        pkg-config \
        webp \
        python3-dev \
        python3-numpy \
        && apt-get clean -qq && rm -rf /var/lib/apt/lists/* 
        

mkdir /home/cosemap/opencv/opencv/build
cd /home/cosemap/opencv/opencv/build
cmake \
    -D OPENCV_EXTRA_MODULES_PATH=/home/cosemap/opencv/opencv_contrib/modules \
    -D OPENCV_DNN_CUDA=ON \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    ..

MAX_CORES=4
NUM_CORES=$(nproc)
CORES_TO_USE=$((NUM_CORES < MAX_CORES ? NUM_CORES : MAX_CORES))
make -j"$CORES_TO_USE" && make install && ldconfig

rm -rf /home/cosemap/opencv /home/scripts