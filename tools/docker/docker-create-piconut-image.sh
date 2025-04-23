#!/bin/bash

#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------


TOOL="${0##*/}"
LOCAL_BASE_IMAGE="piconut_base_image"
LOCAL_BASE_IMAGE_MINIMAL="${LOCAL_BASE_IMAGE}_minimal"
LOCAL_BASE_IMAGE_MINIMAL_TARGET="misc"

###############################################################################
# Usage
###############################################################################

usage() {
  echo "This script is used to build a docker image for the PicoNut project with all necessary software and dependencies installed."
  echo
  echo "Syntax: ${TOOL} [-c|b|k|m|r|h]"
  echo
  echo "Options:"
  echo
  echo "  -c"
  echo "      Specify if the --no-cache option will be used for the build process of the docker images."
  echo
  echo "  -b"
  echo "      Specify if only the base image will be built or pulled from registry."
  echo
  echo "  -k"
  echo "      Specify if the base image should be kept."
  echo "      In case the -b is set, the base image will be kept anyway."
  echo
  echo "  -m"
  echo "      Specify if the base image should be build in minimal version."
  echo "      The minimal version is without ICSC and Qt."
  echo
  echo "  -r <registry-path>"
  echo "      Specify path to the docker registry where the base image is pulled from."
  echo "      If not specified the base image is built locally from Dockerfile.base."
  echo
  echo "  -h"
  echo "      Print this help."
  echo
  echo "Generated outputs:"
  echo
  echo "  piconut_image :  The docker image to be used with the piconut project."
  echo
}

###############################################################################
# Parse input options
###############################################################################

ARG_NO_CACHE=0
BASE_IMAGE_ONLY=0
KEEP_BASE_IMAGE=0
MINIMAL_BASE_IMAGE=0
REGISTRY_BASE_IMAGE=""

while getopts ":cbkmr:h" opt; do
  case "$opt" in
    c)
      ARG_NO_CACHE=1
      ;;
    b)
      BASE_IMAGE_ONLY=1
      ;;
    k)
      KEEP_BASE_IMAGE=1
      ;;
    m)
      MINIMAL_BASE_IMAGE=1
      ;;
    r)
      REGISTRY_BASE_IMAGE="$OPTARG"
      ;;
    h)
      usage
      exit 0
      ;;
    \?)
      echo "Error: Invalid option"
      usage
      exit 1
      ;;
  esac
done

ARGS=""
if [[ $ARG_NO_CACHE -eq 1 ]]; then
  ARGS+="--no-cache"
fi

echo "Configuration:"
echo "Image build arguments: ${ARGS}"
echo "Base image only: ${BASE_IMAGE_ONLY}"
echo "Keep base image: ${KEEP_BASE_IMAGE}"
echo "Minimal Base image: ${MINIMAL_BASE_IMAGE}"
echo "Registry base image: ${REGISTRY_BASE_IMAGE}"
echo

###############################################################################
# Build images
###############################################################################

if [[ -n "$REGISTRY_BASE_IMAGE" ]]; then
    echo "Registry provided: Pulling base image from ${REGISTRY_BASE_IMAGE}"
    docker pull $REGISTRY_BASE_IMAGE
    BASE_IMAGE=$REGISTRY_BASE_IMAGE
else
    echo "No registry provided: Building base image locally"

    if [[ $MINIMAL_BASE_IMAGE -eq 1 ]]; then
      docker build -f Dockerfile.base $ARGS -t $LOCAL_BASE_IMAGE_MINIMAL --target $LOCAL_BASE_IMAGE_MINIMAL_TARGET .
      BASE_IMAGE=$LOCAL_BASE_IMAGE_MINIMAL
    else
      docker build -f Dockerfile.base $ARGS -t $LOCAL_BASE_IMAGE .
      BASE_IMAGE=$LOCAL_BASE_IMAGE
    fi
fi

# build final image if specified
if [[ $BASE_IMAGE_ONLY -eq 0 ]]; then
  echo "Building image"
  docker build \
    -f Dockerfile.final $ARGS \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    --build-arg USER_ID=$(id -u) \
    --build-arg GROUP_ID=$(id -g) \
    -t piconut_image .
fi

###############################################################################
# Cleanup
###############################################################################

# delete base image if specified
if [[ $KEEP_BASE_IMAGE -eq 0 && $BASE_IMAGE_ONLY -eq 0 ]]; then
  docker image rm $BASE_IMAGE
fi

# remove docker build files
if [[ $ARG_NO_CACHE -eq 1 ]]; then
  yes | docker builder prune
fi
