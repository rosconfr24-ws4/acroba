#!/usr/bin/env bash

#set -euxo pipefail

if [ "$1" = 'start_atp' ]; then
    sleep infinity 
else
    exec "$@"
fi 


