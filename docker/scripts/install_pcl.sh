set -e


apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libvtk7-dev \
    libusb-1.0-0-dev \
    libqhull-dev \
    libopenni-dev \
    git \
    libpcap-dev && apt-get clean -qq && rm -rf /var/lib/apt/lists/* 

cd /home/cosemap/
tar -xvzf pcl-1.13.1.tar.gz
rm -f pcl-1.13.1.tar.gz

mkdir /home/cosemap/pcl-pcl-1.13.1/build && cd /home/cosemap/pcl-pcl-1.13.1/build

cmake .. 
make -j1
make install
ldconfig

rm -rf /home/cosemap/pcl-pcl-1.13.1 /home/scripts

ldconfig && apt-get clean -qq && rm -rf /var/lib/apt/lists/* 
