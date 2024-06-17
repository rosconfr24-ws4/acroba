#!/bin/bash

set -e 

#################################################################################
# setup_docker_config.sh
# Display on stdout the docker config (docker compose file) to use
#################################################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DOCKER=$SCRIPT_DIR/../docker

source $SCRIPT_DIR/parse_setup_args.sh

#-----------------------------------------
# Generating the docker config 
#-----------------------------------------

TMP_DIR=$(eval echo ~$USER)/.acroba
TIMESTAMP=$(date +"%y%m%d-%H%M%S")

echo "# setting up the platform with ${_GPU} and ${_X} server support for profile ${PROFILE}"
echo "# docker container project name = ${PROJECT}"
echo "# logs subfolder: ${TIMESTAMP}"

if [ "${CELL}" = "NONE" ]; then
    echo "# setting up the platform without a cell config"
    docker_cmd="GPU=${_GPU} X=${_X} VG_SETUP=${_VG} NET=${_NET} LOG_DIR=${TIMESTAMP} PROJECT=${PROJECT} COMPOSE_PROFILES=${_X},${PROFILE} docker compose -f $DOCKER/docker-compose.acroba.yml"
else
    echo "# setting up the platform with the cell config '${CELL}'"
    docker tag acroba/${CELL} acroba/cell-config
    container_id=$(docker create acroba/cell-config)
    rm -f $TMP_DIR/cell-config.yml
    mkdir -p $TMP_DIR
    echo "# looking for a docker config file for the cell '${CELL}'"
    docker cp "${container_id}:/home/acroba/cell-config/docker-compose.yml" "$TMP_DIR/cell-config.yml" 
    #2> /dev/null
    docker rm "${container_id}" >/dev/null 2>&1
    docker_cmd="GPU=${_GPU} X=${_X} VG_SETUP=${_VG} NET=${_NET} LOG_DIR=${TIMESTAMP} PROJECT=${PROJECT} COMPOSE_PROFILES=${_X},${PROFILE} CELL=${CELL} docker compose -f $DOCKER/docker-compose.acroba.yml -f $DOCKER/docker-compose.cell-config.yml"
    if [ -f $TMP_DIR/cell-config.yml ]; then
        echo "# /home/acroba/cell-config/docker-compose.yml file found"
        docker_cmd="${docker_cmd} -f $TMP_DIR/cell-config.yml"
    else
        echo "# no specific docker setup found"
    fi
    echo "# docker command ${docker_cmd}"
fi;

config_used=$(eval ${docker_cmd} -p ${PROJECT} config)
echo -e "# final docker config:\n"
echo "${config_used}"
