#!/bin/bash

#==================================
# SETUP 
#----------------------------------

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PARENT_DIR=$(dirname "$(readlink -f "$SCRIPT_DIR")")

#==================================
# 0. Parsing which setup to run
#==================================

LONG_OPTIONS=pat,dev
OPTIONS=pd

#--------------------------------------------------
# default arguments 
_pat=false
_dev=true

#---------------------------------------------------
# parsing input 

PARSED=$(getopt --options=$OPTIONS --longoptions=$LONG_OPTIONS --name "setup" -- "$@")
if [[ $? -ne 0 ]]; then
    # getopt should have complained about wrong arguments to stdout
    exit 2
fi    

has_args=false
pat=false 
dev=false 

eval set -- "$PARSED"
while true; do
    case "$1" in
        -p|--pat)
            pat=true
            has_args=true
            shift
            ;;
        -v|--dev)
            dev=true
            has_args=true
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "unknown option $1"
            usage;
            exit 1
            ;;
    esac
done


if [[ "$has_args" = "false" ]]; then 
    # use default arguments 
    pat=$_pat
    dev=$_dev
fi 

echo "Will be running following setup steps:"

if [[ "$pat" = "true" ]]; then
    echo "- PAT setup"
fi 
if [[ "$dev" = "true" ]]; then
    echo "- Install dev container requirements"
fi

read -p "continue? [Y|n]" go
go=${go:-y}
if [ "${go,,}" != "y" ]; then 
    echo "aborting."
    exit 1 
fi 

#=====================================================================

check_docker() {
    while ! docker info >/dev/null 2>&1; do
        echo "docker is not running, please start the Docker engine"
        sleep 5
    done
    echo "docker is running"
}

start_time=$(date +%s)

#====================================
# GIT PERSONAL ACCESS TOKEN SETUP
#====================================

if [[ "$pat" = "true" ]]; then 
    echo "setting up Github registry access"
    sudo apt-get install libsecret-1-0 libsecret-1-dev
    sudo make --directory=/usr/share/doc/git/contrib/credential/libsecret 
    echo "setting up git credential helper"
    git config --global credential.helper /usr/share/doc/git/contrib/credential/libsecret/git-credential-libsecret
    echo "reading pat"
    read -r PAT < "$PARENT_DIR/.pat" || echo "$PAT"
    check_docker
    echo "setting up git registry access"
    git_user_name=$(git config user.name)
    docker login docker.irtjv.local -u $git_user_name --password-stdin <<< "$PAT"
fi 

#==================================
# DEV CONTAINER SETUP
#==================================

if [[ "$dev" = "true" ]]; then 
    if command -v jq &> /dev/null; then 
        echo "jq already installed"
    else
        echo "installing jq"
        sudo apt-get install jq
    fi

    if [ -f ~/.nvm/nvm.sh ]; then
        echo "nvm already installed"    
    else
        echo "installing nvm"
        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
    fi 
    source ~/.nvm/nvm.sh

    if command -v npm &> /dev/null; then
        echo "npm already installed"
    else
        echo "installing npm"
        nvm install node
    fi
    npm install -g @devcontainers/cli
fi

#==================================

end_time=$(date +%s)
duration=$((end_time - start_time))

echo "all set up :) ! took $duration seconds."

