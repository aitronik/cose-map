#!/bin/bash
set -e

echo "pulling cosemap:latest from Aitronik Docker Hub"

docker pull 192.168.1.51:4242/cosemap:latest
docker tag  192.168.1.51:4242/cosemap:latest cosemap:latest


echo "pulled image from Aitronik Docker Hub"
 