#!/usr/bin/env bash

set -euo pipefail

# setup the ssh keys
SSH_PUB_KEY=/home/acroba/config/ssh/acroba.pub
if [ -f "$SSH_PUB_KEY" ]; then 
    echo "ssh public key $SSH_PUB_KEY found, adding to authorized keys"
    mkdir -p ~/.ssh && cp $SSH_PUB_KEY ~/.ssh/authorized_keys
    chmod 600 ~/.ssh/authorized_keys
else
    echo "can't find ssh public key $SSH_PUB_KEY"
fi

if [ -n "$SSH_PORT" ]; then
    echo "set up sshd to use port $SSH_PORT"
    sudo sed -i "s/^#Port.*/Port $SSH_PORT/" /etc/ssh/sshd_config
    sudo sed -i "s/^#PermitUserEnvironment.*/PermitUserEnvironment yes/" /etc/ssh/sshd_config
fi

exec sudo service ssh start -D
