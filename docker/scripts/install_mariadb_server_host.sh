#!/bin/bash
set -e

sudo apt update && sudo apt install -y mariadb-server 
sudo systemctl restart mariadb 
