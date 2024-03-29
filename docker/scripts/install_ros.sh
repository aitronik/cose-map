set -e
# ros2 galactic

apt-get update && apt install -y locales  software-properties-common curl \
        build-essential gpg-agent cmake git wget gcc-8 g++-8 gnupg2 \
        lsb-release \
        ca-certificates \
        libssl-dev \
        libyaml-cpp-dev \
        libi2c-dev \
        libzmqpp-dev \
        libbluetooth-dev \
        libglm-dev \
        libgsl-dev \
        xorg-dev \
        libglu1-mesa-dev \
        libzbar-dev \
        libusb-dev \
        htop \
        nano \
        udev \
        clang-8 \
        mariadb-server \
        mariadb-client \
        mlocate \
        libmariadb3 \
        libvtk7-dev

locale-gen en_US en_US.UTF-8 
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

add-apt-repository universe

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update && apt-get upgrade -y
#Communication libraries, message packages, command line tools. No GUI tools.
export ROS_DISTRO=galactic
 
apt-get update && \
    apt-get install -y --no-install-recommends \
		ros-${ROS_DISTRO}-ros-base \
        ros-dev-tools \
		python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-nav2-costmap-2d \
        ros-${ROS_DISTRO}-behaviortree-cpp-v3 \
        ros-${ROS_DISTRO}-nav2-common \
        ros-${ROS_DISTRO}-nav2-core \
        ros-${ROS_DISTRO}-nav2-bt-navigator \
        ros-${ROS_DISTRO}-nav2-msgs \
        ros-${ROS_DISTRO}-nav-2d-utils \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-nav2-costmap-2d \
        ros-${ROS_DISTRO}-sick-safetyscanners2 \
        ros-${ROS_DISTRO}-sick-safetyscanners2-interfaces \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-spatio-temporal-voxel-layer \
        ros-${ROS_DISTRO}-gazebo-ros-pkgs\
        ros-${ROS_DISTRO}-gazebo-ros2-control \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-trajectory-controller \
        ros-${ROS_DISTRO}-velocity-controllers \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-action-msgs \
        ros-${ROS_DISTRO}-dynamixel-sdk

update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9 
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 10 
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9 
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 10 
     
ldconfig && apt-get clean -qq && rm -rf /var/lib/apt/lists/* 

echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc
echo "source /home/cosemap/ws/install/setup.bash" >> /root/.bashrc

rm -rf /home/cosemap/pcl /home/scripts
