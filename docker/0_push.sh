#!/bin/bash
set -e

echo "pushing cosemap to Docker Hub"
echo "NB: did you remember to 'docker login 192.168.1.51:4242 '?? (use your nas credentials)"
  
docker image tag cosemap:latest 192.168.1.51:4242/cosemap:latest
docker push                      192.168.1.51:4242/cosemap

echo "pushed image to Docker Hub"
