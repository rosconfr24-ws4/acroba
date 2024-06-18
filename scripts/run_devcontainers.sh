#!/bin/bash

set -e

#################################################
# Run devcontainers and open them in vscode
#################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_DIR=$SCRIPT_DIR/..
CONTAINER_DIR=$WORKSPACE_DIR/.devcontainer

# asks whether to copy volumes from acroba project to the acroba-dev project

if docker volume inspect "acroba_data" &> /dev/null; then        
    # read -p "copy data from acroba_data volume ? [y/N] " copy_volume
    copy_volume=${copy_volume:-n}
    if [ "${copy_volume,,}" = "y" ]; then 
        $SCRIPT_DIR/copy_volume.sh acroba_data acroba-dev_data
    fi
fi 

# launch and open devcontainers in vscode (using devcontainer cli)
# devcontainer cli installed from the command palette is buggy (freezes)

services=($(jq -r '.services[]' $CONTAINER_DIR/setup.json))

set +e 

for service in "${services[@]}"; do
    echo -e "\n\n###########################################"
    echo "#  Starting devcontainer for $service "
    echo -e "###########################################\n"
    output=$(COMPOSE_PROFILES=* devcontainer up --config $CONTAINER_DIR/$service/devcontainer.json --workspace-folder $WORKSPACE_DIR)
    
    outcome=$(echo "$output" | jq -r '.outcome')
    if [ "$outcome" = "success" ]; then 
        container_id=$(echo "$output" | jq -r .containerId) 
        hex=$(printf "$container_id" | od -A n -t x1 | sed 's/ *//g' | tr -d '\n')
        workspace_folder=$(jq -r '.workspaceFolder' $CONTAINER_DIR/$service/devcontainer.json)
        code -n --folder-uri vscode-remote://attached-container+$hex$workspace_folder
    else 
        echo "$output" | jq -r '.message'
        echo "$output" | jq -r '.description'
    fi 

done 

