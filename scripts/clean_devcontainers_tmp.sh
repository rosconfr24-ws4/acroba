#!/bin/bash

set -e

#################################################
# Run devcontainers and open them in vscode
#################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_DIR=$SCRIPT_DIR/..
CONTAINER_DIR=$WORKSPACE_DIR/.devcontainer

rm -f $CONTAINER_DIR/docker-compose.acroba.yml
rm -f $CONTAINER_DIR/docker-compose.dev.yml
rm -f $CONTAINER_DIR/docker-compose.devcontainer.yml

find $CONTAINER_DIR/* -maxdepth 1 -type d -not -name "templates" -exec rm -r {} +


