#!/bin/bash

#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
#                2025 Tristan Kundrat <tristan.kundrat@tha.de>
#                2025 Lukas Bauer <lukas.bauer1@tha.de>
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

set -e

TOOL="${0##*/}"

IMAGE_VERSION="1.0.4"
CLOUD_URL="https://cloud.hs-augsburg.de/public.php/dav/files/GizTAYQHmdLgmq6"

# image to start from
START_IMAGE="debian:trixie"

# namings for generated images
IMAGE_NAME="piconut"
RAW_IMAGE_NAME="${IMAGE_NAME}_raw"

IMAGE_TAG=$IMAGE_VERSION
IMAGE_TAG_EXT_BASE="base"
IMAGE_TAG_EXT_RISCV="toolchain" # riscv toolchain + riscof
IMAGE_TAG_EXT_ICSC="icsc"
IMAGE_TAG_EXT_DOC="doc"
IMAGE_TAG_EXT_OSS="oss"
IMAGE_TAG_EXT_QT="qt"

# dockerfiles
DOCKERFILES_PATH="dockerfiles"
DOCKERFILE_NAME_BASE="Dockerfile.base"
DOCKERFILE_NAME_RISCV="Dockerfile.riscv"
DOCKERFILE_NAME_ICSC="Dockerfile.icsc"
DOCKERFILE_NAME_DOC="Dockerfile.doc"
DOCKERFILE_NAME_OSS="Dockerfile.oss"
DOCKERFILE_NAME_QT="Dockerfile.qt"
DOCKERFILE_NAME_FINALIZED="Dockerfile.finalized"

# CLI Arguments
# general
GEN_KEEP_BASE=0
GEN_LOCAL_RAW_IMAGE_NAME_WITH_TAG=""
GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG=""
GEN_CLOUD_URL=""
GEN_TAG=""

# docker args
ARG_NO_CACHE=0

# build args
BUILD_ARG_USER_ID=$(id -u)
BUILD_ARG_GROUP_ID=$(id -g)

# stages
WITH_BASE=1
WITH_RISCV=0
WITH_ICSC=0
WITH_DOC=0
WITH_OSS=0
WITH_QT=0
WITH_FINALIZED=1

###############################################################################
# Usage
###############################################################################

usage() {
  echo "This script is used to build a docker image for the PicoNut project with all necessary software and dependencies installed."
  echo
  echo "Usage: ${TOOL} [options...]"
  echo
  echo "Options:"
  echo
  echo "  -c, --no-cache"
  echo "      Specify if the --no-cache option will be used for the build process of the docker images."
  echo
  echo "  -A, --all"
  echo "      Specify that the raw image include all packages."
  echo "      This includes: basic tools, RISC-V Toolchain, ICSC, Doc, OSS CAD Suite and Qt."
  echo
  echo "  -a, --all-no-qt"
  echo "      Specify that the raw image include all packages without Qt."
  echo "      This includes: basic tools, RISC-V Toolchain, ICSC, Doc, OSS CAD Suite."
  echo
  echo "  -s, --sim"
  echo "      Specify that the raw image include all packages necessary for"
  echo "      simulation and software development."
  echo "      This includes: basic tools, RISC-V Toolchain."
  echo
  echo "  -w, --hw"
  echo "      Specify that the raw image include all packages necessary for"
  echo "      simulation, software development and hardware development."
  echo "      This includes: basic tools, RISC-V Toolchain, ICSC, OSS CAD Suite."
  echo
  echo "  -m, --minimal"
  echo "      Specify that the raw image include all packages necessary for"
  echo "      simulation"
  echo "      This includes: basic tools"
  echo
  echo "      --toolchain"
  echo "      Specify that the raw image include the RISC-V Toolchain."
  echo
  echo "      --icsc"
  echo "      Specify that the raw image include ICSC."
  echo
  echo "      --doc"
  echo "      Specify that the raw image include all the tools for documentation."
  echo
  echo "      --oss"
  echo "      Specify that the raw image include the OSS CAD Suite."
  echo
  echo "      --qt"
  echo "      Specify that the raw image include Qt."
  echo
  echo "      --no-finalized"
  echo "      Specify that the raw image will be build only."
  echo "      Note: The raw image will not be deleted. The --keep option is not needed."
  echo
  echo "      --keep"
  echo "      Specify if the raw image will be kept."
  echo "      In case the -b is set, the raw image will be kept anyway."
  echo
  echo "      --local-raw <local-image-name-with-tag>"
  echo "      Specify the name of the raw image that will be used for the finalized image."
  echo "      Note: The image <local-image-name> has to exist on your system."
  echo
  echo "      --registry-raw <registry-image-name-with-tag>"
  echo "      Specify where the raw image will be pulled from."
  echo
  echo "      --cloud-raw[=<url>]"
  echo "      Specify where the raw image will be pulled from a cloud. If <url> is not specified the latest version is pulled."
  echo "      Latest raw image url: ${CLOUD_URL}."
  echo
  echo "      --tag <tag>"
  echo "      Specify the tag of the finalized image."
  echo
  echo "      --uid <uid>"
  echo "      Specify the user id to which the files in the finalized image will be assigned."
  echo "      Note: This option is only used when building the finalized image."
  echo "      Default: User id of current user."
  echo
  echo "      --gid <gid>"
  echo "      Specify the group id to which the files in the finalized image will be assigned."
  echo "      Note: This option is only used when building the finalized image."
  echo "      Default: Group id of current user."
  echo
  echo "  -h, --help"
  echo "      Print this help."
  echo
  echo "Generated outputs:"
  echo
  echo "  piconut :       The docker image to be used with the piconut project."
  echo "  piconut_raw :   The docker image to be used as a foundation for the image called piconut."
  echo "                  Note: This output is optional. See option --keep or --no-finalized"
  echo
  echo "Examples:"
  echo
  echo "  ./${TOOL}"
  echo "  ./${TOOL} --gid 1000 --uid 1000"
  echo "  ./${TOOL} --toolchain --icsc --keep"
  echo "  ./${TOOL} --all-no-qt --no-finalized"
  echo "  ./${TOOL} --cloud-raw --qt"
  echo "  ./${TOOL} --local-raw piconut_raw:${IMAGE_TAG}-toolchain-icsc-doc-oss --qt"
  echo
}

error () {
  # Arguments: <ERROR_MSG>
  echo
  echo "ERROR: $1"
  echo
  usage
  exit 1
}

###############################################################################
# Helpers
###############################################################################

CURRENT_RAW_IMAGE_TAG=""

build_image() {
  # Arguments: <DOCKERFILE_NAME> <IMAGE_NAME> <PREV_IMAGE_NAME>
  local DOCKERFILE_NAME="$1"
  local IMAGE_NAME="$2"
  local PREV_IMAGE_NAME="$3"
  shift 3

  echo "Building image from ${DOCKERFILE_NAME}"
  docker build $ARGS \
    -f ${DOCKERFILES_PATH}/${DOCKERFILE_NAME} \
    -t $IMAGE_NAME \
    --build-arg PREV_IMAGE=${PREV_IMAGE_NAME} \
    . || error "Unable to build image from ${DOCKERFILE_NAME}."
}

build_raw_image() {
  # Arguments: <DOCKERFILE_NAME> <IMAGE_TAG_EXT>
  local DOCKERFILE_NAME="$1"
  local IMAGE_TAG_EXT="$2"
  shift 2

  local PREV_IMAGE_NAME_WITH_TAG="${RAW_IMAGE_NAME}:${CURRENT_RAW_IMAGE_TAG}"
  
  CURRENT_RAW_IMAGE_TAG="${CURRENT_RAW_IMAGE_TAG}-${IMAGE_TAG_EXT}"
  local IMAGE_NAME_WITH_TAG="${RAW_IMAGE_NAME}:${CURRENT_RAW_IMAGE_TAG}"

  build_image \
    $DOCKERFILE_NAME \
    $IMAGE_NAME_WITH_TAG \
    $PREV_IMAGE_NAME_WITH_TAG

  # cleanup obsolete previous docker image
  docker image rm $PREV_IMAGE_NAME_WITH_TAG
}

###############################################################################
# Parse input options
###############################################################################

STAGES_SPECIFIED=0

PARSED_ARGUMENTS=$(getopt -n $TOOL -s "bash" -o "nAswamh" \
  -l "no-cache,all,sim,hw,all-no-qt,minimal,toolchain,icsc,doc,oss,qt,no-finalized,\
      keep,local-raw:,registry-raw:,cloud-raw::,tag:,gid:,uid:,help" -- "$@")

eval set -- "$PARSED_ARGUMENTS"
while true; do
  case "$1" in
    -c | --no-cache)
      ARG_NO_CACHE=1
      shift
      ;;
    -A | --all)
      WITH_RISCV=1
      WITH_ICSC=1
      WITH_DOC=1
      WITH_OSS=1
      WITH_QT=1
      STAGES_SPECIFIED=1
      shift
      ;;
    -s | --sim)
      WITH_RISCV=1
      STAGES_SPECIFIED=1
      shift
      ;;
    -w | --hw)
      WITH_RISCV=1
      WITH_ICSC=1
      WITH_OSS=1
      STAGES_SPECIFIED=1
      shift
      ;;
    -a | --all-no-qt)
      WITH_RISCV=1
      WITH_ICSC=1
      WITH_DOC=1
      WITH_OSS=1
      STAGES_SPECIFIED=1
      shift
      ;;
    -m | --minimal)
      # Only WITH_BASE
      STAGES_SPECIFIED=1
      shift
      ;;
    --toolchain)
      WITH_RISCV=1
      STAGES_SPECIFIED=1
      shift
      ;;
    --icsc)
      WITH_ICSC=1
      STAGES_SPECIFIED=1
      shift
      ;;
    --doc)
      WITH_DOC=1
      STAGES_SPECIFIED=1
      shift
      ;;
    --oss)
      WITH_OSS=1
      STAGES_SPECIFIED=1
      shift
      ;;
    --qt)
      WITH_QT=1
      STAGES_SPECIFIED=1
      shift
      ;;
    --no-finalized)
      WITH_FINALIZED=0
      shift
      ;;
    --keep)
      GEN_KEEP_BASE=1
      shift
      ;;
    --local-raw)
      GEN_LOCAL_RAW_IMAGE_NAME_WITH_TAG="$2"
      WITH_BASE=0
      shift 2
      ;;
    --registry-raw)
      GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG="$2"
      WITH_BASE=0
      shift 2
      ;;
    --cloud-raw)
      if [[ -n "$2" ]]; then
        GEN_CLOUD_URL="$2"
      else
        GEN_CLOUD_URL=$CLOUD_URL
      fi
      WITH_BASE=0
      shift 2
      ;;
    --tag)
      GEN_TAG="$2"
      shift 2
      ;;
    --uid)
      BUILD_ARG_USER_ID="$2"
      shift 2
      ;;
    --gid)
      BUILD_ARG_GROUP_ID="$2"
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    \?)
      error "Invalid option: $1."
      ;;
  esac
done

# build all stages if none specified
if [[ $STAGES_SPECIFIED -eq 0 ]]; then
  WITH_RISCV=1
  WITH_ICSC=1
  WITH_DOC=1
  WITH_OSS=1
  WITH_QT=1
fi

###############################################################################
# Build images
###############################################################################

# make docker arguments
ARGS=""
if [[ $ARG_NO_CACHE -eq 1 ]]; then
  ARGS+="--no-cache"
fi


# Get or build raw image
if [[ -n "$GEN_LOCAL_RAW_IMAGE_NAME_WITH_TAG" ]]; then
  echo "Local raw image provided: Using raw image ${GEN_LOCAL_RAW_IMAGE_NAME_WITH_TAG}"
  
  # split combined name with tag into separate name and tag
  RAW_IMAGE_NAME=${GEN_LOCAL_RAW_IMAGE_NAME_WITH_TAG%%:*}
  CURRENT_RAW_IMAGE_TAG=${GEN_LOCAL_RAW_IMAGE_NAME_WITH_TAG#*:}
  if [[ -z $RAW_IMAGE_NAME || -z $CURRENT_RAW_IMAGE_TAG ]]; then
    error "Invalid tag or name specified."
  fi

elif [[ "$GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG" ]]; then
  echo "Registry raw image provided: Pulling raw image from ${GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG}"
    
  # split combined name with tag into separate name and tag
  RAW_IMAGE_NAME=${GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG%%:*}
  CURRENT_RAW_IMAGE_TAG=${GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG#*:}
  if [[ -z $RAW_IMAGE_NAME || -z $CURRENT_RAW_IMAGE_TAG ]]; then
    error "Invalid tag or name specified."
  fi

  docker pull $GEN_REGISTRY_RAW_IMAGE_NAME_WITH_TAG \
  || error "Unable to pull image from registry."


elif [[ "$GEN_CLOUD_URL" ]]; then
  echo "Cloud raw image provided: Download raw image from ${GEN_CLOUD_URL}"

  # Get raw image from cloud and load it
  wget ${GEN_CLOUD_URL} \
  || error "Unable to download the image."

  echo "Loading image, this may take a while..."
  RAW_IMAGE_FILENAME=$(basename "${GEN_CLOUD_URL}")
  DOCKER_LOAD_OUTPUT=$(docker load < $RAW_IMAGE_FILENAME) \
  || error "Unable to load the image."
  rm $RAW_IMAGE_FILENAME

  # Parse output string of docker load to get the loaded image name and tag
  DOCKER_LOAD_OUTPUT_PART="${DOCKER_LOAD_OUTPUT#Loaded image: }"

  RAW_IMAGE_NAME="${DOCKER_LOAD_OUTPUT_PART%%:*}"
  CURRENT_RAW_IMAGE_TAG="${DOCKER_LOAD_OUTPUT_PART#*:}"
  if [[ -z $RAW_IMAGE_NAME || -z $CURRENT_RAW_IMAGE_TAG ]]; then
    error "Unable to load the image."
  fi

else    
  echo "Neither registry nor local base image provided: Building raw image locally"

  if [[ $WITH_BASE -eq 1 ]]; then
    echo "Building base image"

    build_image \
      $DOCKERFILE_NAME_BASE \
      "${RAW_IMAGE_NAME}:${IMAGE_TAG}" \
      $START_IMAGE
    CURRENT_RAW_IMAGE_TAG=$IMAGE_TAG
  fi
fi

# Extensions
if [[ $WITH_RISCV -eq 1 ]]; then
  echo "Building riscv image"
  build_raw_image $DOCKERFILE_NAME_RISCV $IMAGE_TAG_EXT_RISCV
fi
  
if [[ $WITH_ICSC -eq 1 ]]; then
  echo "Building icsc image"
  build_raw_image $DOCKERFILE_NAME_ICSC $IMAGE_TAG_EXT_ICSC
fi

if [[ $WITH_DOC -eq 1 ]]; then
  echo "Building doc image"
  build_raw_image $DOCKERFILE_NAME_DOC $IMAGE_TAG_EXT_DOC
fi

if [[ $WITH_OSS -eq 1 ]]; then
  echo "Building oss image"
  build_raw_image $DOCKERFILE_NAME_OSS $IMAGE_TAG_EXT_OSS
fi

if [[ $WITH_QT -eq 1 ]]; then
  echo "Building qt image"
  build_raw_image $DOCKERFILE_NAME_QT $IMAGE_TAG_EXT_QT
fi

CURRENT_RAW_IMAGE_NAME_WITH_TAG="${RAW_IMAGE_NAME}:${CURRENT_RAW_IMAGE_TAG}"
# Set tag to last raw image if specified
if [[ -n "$GEN_TAG" ]]; then
  docker image tag $CURRENT_RAW_IMAGE_NAME_WITH_TAG "${RAW_IMAGE_NAME}:${GEN_TAG}"
  docker image rm $CURRENT_RAW_IMAGE_NAME_WITH_TAG
  CURRENT_RAW_IMAGE_NAME_WITH_TAG="${RAW_IMAGE_NAME}:${GEN_TAG}"
fi

# Finalized image
if [[ $WITH_FINALIZED -eq 1 ]]; then
  echo "Building finalized image"

  if [[ -n "$GEN_TAG" ]]; then
    # Use tag from argument
    IMAGE_TAG=$GEN_TAG
  else
    # Use same tag as the last raw image
    IMAGE_TAG=$CURRENT_RAW_IMAGE_TAG
  fi

  docker build $ARGS \
    -f ${DOCKERFILES_PATH}/${DOCKERFILE_NAME_FINALIZED} \
    --build-arg PREV_IMAGE=$CURRENT_RAW_IMAGE_NAME_WITH_TAG \
    --build-arg USER_ID=$BUILD_ARG_USER_ID \
    --build-arg GROUP_ID=$BUILD_ARG_GROUP_ID \
    -t "${IMAGE_NAME}:${IMAGE_TAG}" \
    . || error "Unable to build finalized image."

    # create symlink to finalized image called piconut:latest for use in docker compose file
    docker image tag "${IMAGE_NAME}:${IMAGE_TAG}" "${IMAGE_NAME}:latest"
fi

###############################################################################
# Cleanup
###############################################################################

# delete raw image if specified
if [[ $GEN_KEEP_BASE -eq 0 && $WITH_FINALIZED -eq 1 ]]; then
  echo "Cleanup raw image"
  docker image rm $CURRENT_RAW_IMAGE_NAME_WITH_TAG
  yes | docker builder prune
fi

echo "Script finished successfully!"

