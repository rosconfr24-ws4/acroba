#!/bin/bash

set -e 

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
CONF_DIR=$SCRIPT_DIR/../.config

if [ ! -f $CONF_DIR/ssh/private/acroba ]; then
    echo -n "no ssh keys found, generating..."
    mkdir -p $CONF_DIR/ssh/public $CONF_DIR/ssh/private
    ssh-keygen -t rsa -b 4096 -C "platform@acrobaproject.eu" -f $CONF_DIR/ssh/acroba -N ""
    mv $CONF_DIR/ssh/acroba.pub $CONF_DIR/ssh/public/
    mv $CONF_DIR/ssh/acroba $CONF_DIR/ssh/private/
    echo "done!"
else
    echo "found ssh keys, reusing."
fi 

echo -e "ssh config ok!\n"
