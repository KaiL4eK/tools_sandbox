#!/bin/bash

docker run --rm -it --privileged \
    -v /dev:/dev -v `pwd`:/home/developer/sample \
    docker-openvino \
    bash -c  "cd sample; mkdir build; cd build; cmake .."
