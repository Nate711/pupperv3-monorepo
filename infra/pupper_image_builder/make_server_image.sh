#!/bin/bash -e

set -x

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init ubuntu_server_24.04_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build ubuntu_server_24.04_arm64.pkr.hcl