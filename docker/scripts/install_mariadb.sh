set -e

apt-get update && apt-get install -y curl
# C connector 
echo "30d2a05509d1c129dd7dd8430507e6a7729a4854ea10c9dcf6be88964f3fdc25  mariadb_repo_setup" \
    | sha256sum -c -
chmod +x mariadb_repo_setup
./mariadb_repo_setup \
   --mariadb-server-version="mariadb-10.6"

apt-get update && apt-get install -y \
        libmariadb3 \
        libmariadb-dev \

# C++ connector
tar -xvzf mariadb-connector-cpp-1.0.3-ubuntu-focal-amd64.tar.gz 
cd /home/mariadb-connector-cpp-1.0.3-ubuntu-focal-amd64
mkdir -p /usr/include/mariadb/conncpp
mkdir -p /usr/include/mariadb/conncpp/compat
cp -R include/mariadb/. /usr/include/mariadb/
cp -R include/mariadb/conncpp/. /usr/include/mariadb/conncpp
cp -R include/mariadb/conncpp/compat/. /usr/include/mariadb/conncpp/compat
mkdir -p /usr/lib/mariadb
mkdir -p /usr/lib/mariadb/plugin
cp lib/mariadb/libmariadbcpp.so /usr/lib
cp -R lib/mariadb/plugin/. /usr/lib/mariadb/plugin
rm -rf /home/mariadbcppconn.tar.gz mariadb-connector-cpp-1.0.3-ubuntu-focal-amd64

apt-get install -y \
        gcc \
        python3-dev \
        openssl \
        software-properties-common

add-apt-repository ppa:deadsnakes/ppa
apt-get install -y \
        python3.9 \
        python3-pip

# Python connector
pip3 install mariadb
pip3 install ultralytics
pip3 install subprocess

ldconfig && apt-get clean -qq && rm -rf /var/lib/apt/lists/* 
