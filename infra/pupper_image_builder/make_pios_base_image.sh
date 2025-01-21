#!/bin/bash -e

set -x

docker pull mkaczanowski/packer-builder-arm:latest
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest init pios_base_arm64.pkr.hcl
docker run --rm --privileged -v /dev:/dev -v ${PWD}:/build mkaczanowski/packer-builder-arm:latest build pios_base_arm64.pkr.hcl