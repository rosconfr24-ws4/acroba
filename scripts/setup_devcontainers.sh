#!/bin/bash

# set -e 

if ! docker --version &> /dev/null; then 
    echo "Error: docker is not running." 
    exit 1
fi 

###############################################################################
# Setting up dev containers for developping/debugging on the ACROBA platform
###############################################################################

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
LOCAL_WORKSPACE=$SCRIPT_DIR/..
DEVCONTAINER=$LOCAL_WORKSPACE/.devcontainer

#---------------------------------------------------------------------
# Parsing user config 
#---------------------------------------------------------------------

source $SCRIPT_DIR/parse_setup_args.sh

#----------------------------------------------------------------------
# retrieving last config used, i.e. cell-config & services to debug
#----------------------------------------------------------------------

if [[ -s $DEVCONTAINER/setup.json ]]; then    
    last_services=$(jq -r '.services[]' $DEVCONTAINER/setup.json)
    last_cell_config=$(jq -r '.["cell-config"].name // "NONE"' $DEVCONTAINER/setup.json)
fi

last_cell_config=${last_cell_config:-cell-config-roscon}
last_services=${last_services//$'\n'/ }

#----------------------------------------------------------------------
# Getting dev Container config
#----------------------------------------------------------------------

source $SCRIPT_DIR/utils.sh 

echo "Platform setup"

# _CELLCONFIGS=(NONE $(docker images --format "{{.Repository}}" | grep "^acroba/cell-config-" | grep -v cell-config-base | sed 's/^acroba\///'))
# cell_config=$(choose_value "cell config to use?" _CELLCONFIGS[@] "$last_cell_config")

cell_config=cell-config-roscon

# default_profile="acroba"
# declare -A valid_profiles=([acroba]=1 [drl]=1)

# while true; do
#     read -p "Enter profile [default: $default_profile]: " profile
#     profile=${profile:-$default_profile}
#     profile_lower=$(echo "$profile" | tr '[:upper:]' '[:lower:]')

#     if [[ ${valid_profiles[$profile_lower]+_} ]]; then
#         profile=$profile_lower
#         break
#     else
#         echo "Invalid input. Please enter a valid profile or leave empty for default."
#         echo "Valid profiles are: ${!valid_profiles[@]}"
#     fi
# done

profile=acroba

#--- Service(s) choice

_SERVICES=($(GPU="$_GPU" X="$_X" LOG_DIR="" PROJECT="" VG_SETUP="$_VG" NET="$_NET" COMPOSE_PROFILES="$profile" docker compose -f docker/docker-compose.acroba.yml config --services | grep -v roscore | grep -v initializer | grep -v ros1-bridge))
services=$(choose_value "Service(s) to debug?" _SERVICES[@] "$last_services" true)

echo -e "\n---------------------------------------------"
echo "Setting up devcontainers" 
echo "cell_config='$cell_config'"
echo "services='$services'"
echo -e "\n"

#----------------------------------------------------------------------
# Remove previously generated configs
#----------------------------------------------------------------------

rm -f $DEVCONTAINER/docker-compose.acroba.yml
rm -f $DEVCONTAINER/docker-compose.dev.yml
find $DEVCONTAINER/* -type d -not -name '.' -not -name 'templates' -print0 | xargs -0 rm -rf -- 

#----------------------------------------------------------------------
# Dev Container configuration files creation
#----------------------------------------------------------------------

source $SCRIPT_DIR/devcontainers_config.sh
source $SCRIPT_DIR/devcontainers_utils.sh

echo "generating platform docker compose file..." 
$SCRIPT_DIR/setup_docker_config.sh --GPU=$GPU --X11=$X11 --VG=$VG --CELL=$cell_config --PROJECT=acroba-dev --PROFILE=${profile} > $DEVCONTAINER/docker-compose.acroba.yml

# keeping track of the setting used: 

if [[ ! -f $DEVCONTAINER/setup.json ]]; then 
    # create default file when no existing setup
    echo "{}" > $DEVCONTAINER/setup.json
    jq '."cell-config" = {}' $DEVCONTAINER/setup.json > $DEVCONTAINER/setup.new.json
    mv -f $DEVCONTAINER/setup.new.json $DEVCONTAINER/setup.json 
fi 

# saving the cell config used
if [[ "$cell_config" = "NONE" ]]; then 
    jq 'del(.["cell-config"]["name"])' $DEVCONTAINER/setup.json > $DEVCONTAINER/setup.new.json 
else
    jq --arg cell_config "$cell_config" '.["cell-config"].name = $cell_config' $DEVCONTAINER/setup.json >  $DEVCONTAINER/setup.new.json 
fi 

# saving the services used 
jq --argjson services "[\"${services// /\",\"}\"]" '.services = $services' $DEVCONTAINER/setup.new.json > $DEVCONTAINER/setup.json 

echo "" > $DEVCONTAINER/docker-compose.devcontainer.yml
IFS=' ' read -ra services <<< $services
for service in "${services[@]}"; do 

    if [[ "$service" == "cell-config" ]] || [[ "$service" == "cell_config" ]] || [[ "$service" == "$cell_config" ]]; then 

        #-----------------------------------
        # Cell config setup 
        #-----------------------------------
        
        # if [[ "$cell_config" == "NONE" ]]; then 
        #     echo "Cannot debug cell-config service, no cell-config was given"
        #     exit 1 
        # fi 
    
        # #-------------------------------------
        # # retrieving the cell config repo path 

        # last_cell_config_path=$(jq -r --arg config "$cell_config" '.["cell-config"].paths[$config]' $DEVCONTAINER/setup.json)

        # if [ -z $last_cell_config_path ]; then 
        #     ask="path to cell config '${cell_config}' repo? "
        # else 
        #     ask="path to cell config '${cell_config}' repo? [default: $last_cell_config_path] " 
        # fi 

        # read -p "$ask" cell_config_path
        # cell_config_path=${cell_config_path:-"$last_cell_config_path"}
        cell_config_path=$LOCAL_WORKSPACE/acroba-modules/cell_config

        #-------------------------------------
        # keeping track of the settings used 
        jq --arg cell "$cell_config" --arg path "$cell_config_path" '.["cell-config"].paths += {($cell): $path}' $DEVCONTAINER/setup.json > $DEVCONTAINER/setup.new.json
        mv -f $DEVCONTAINER/setup.new.json $DEVCONTAINER/setup.json 
        
        #-----------------------------------------------------------------
        # generating devcontainer files
        echo -n "generating devcontainer setup for the cell-config module..."
        mkdir -p $DEVCONTAINER/cell-config
        #-------------------
        # devcontainer.json
        sed "s/__service_name__/cell-config/" $DEVCONTAINER/templates/devcontainer.template.json > $DEVCONTAINER/cell-config/devcontainer.json
        create_platform_devcontainer "$cell_config_path/.devcontainer/devcontainer.json" "$DEVCONTAINER/cell-config/devcontainer.json" $cell_config_path
        #-------------------
        # docker dev file 
        sed "s#__cell_config_path__#$cell_config_path#" $DEVCONTAINER/templates/docker-compose.dev.cell-config.yml >> $DEVCONTAINER/docker-compose.dev.cell-config.yml
        # replace the mounting point of the workspace folder in the docker file
        workspace_folder=$(jq '.workspaceFolder' "$DEVCONTAINER/cell-config/devcontainer.json")
        sed "s#__workspace_folder__#$workspace_folder#" $DEVCONTAINER/docker-compose.dev.cell-config.yml >> $DEVCONTAINER/docker-compose.dev.yml
        rm -f $DEVCONTAINER/docker-compose.dev.cell-config.yml
        #------------------
        # docker compose container features file
        echo -e "services:\n  cell-config:" > $DEVCONTAINER/cell-config/docker-compose.devcontainer.yml
        create_container_features_docker_compose  $DEVCONTAINER/cell-config/devcontainer.json $cell_config_path $DEVCONTAINER/cell-config/docker-compose.devcontainer.yml
        cat $DEVCONTAINER/cell-config/docker-compose.devcontainer.yml >> $DEVCONTAINER/docker-compose.devcontainer.yml
        # rm -f $DEVCONTAINER/cell-config/docker-compose.devcontainer.yml
        echo "ok"

    else 

        #-----------------------------------
        # acroba module setup 
        #-----------------------------------
        module_name=$(get_module_name "$service")
        echo -n "generating devcontainer setup for service $service (i.e. module $module_name)..."
        mkdir -p $DEVCONTAINER/$service
        #-------------------
        # devcontainer.json
        sed "s/__service_name__/$service/" $DEVCONTAINER/templates/devcontainer.template.json > $DEVCONTAINER/$service/devcontainer.json
        # merging module .devcontainer.json settings into dev container
        create_platform_devcontainer "$LOCAL_WORKSPACE/acroba-modules/$module_name/.devcontainer/devcontainer.json" "$DEVCONTAINER/$service/devcontainer.json" "\${localWorkspaceFolder}/acroba-modules/$module_name"
        #-------------------
        # docker compose dev file 
        sed "s/__service_name__/$service/" $DEVCONTAINER/templates/docker-compose.dev.module.yml >> $DEVCONTAINER/docker-compose.dev.$service.yml
        sed -i "s/__module_name__/$module_name/" $DEVCONTAINER/docker-compose.dev.$service.yml
        workspace_folder=$(jq -r '.workspaceFolder' "$DEVCONTAINER/$service/devcontainer.json")
        sed "s#__workspace_folder__#$workspace_folder#" $DEVCONTAINER/docker-compose.dev.$service.yml >> $DEVCONTAINER/docker-compose.dev.yml
        rm -f $DEVCONTAINER/docker-compose.dev.$service.yml
        #-------------------
        # docker compose container features file 
        echo -e "services:\n  $service:" > $DEVCONTAINER/$service/docker-compose.devcontainer.yml
        create_container_features_docker_compose  $DEVCONTAINER/$service/devcontainer.json $LOCAL_WORKSPACE $DEVCONTAINER/$service/docker-compose.devcontainer.yml
        cat $DEVCONTAINER/$service/docker-compose.devcontainer.yml >> $DEVCONTAINER/docker-compose.devcontainer.yml
        # rm -f $DEVCONTAINER/$service/docker-compose.devcontainer.yml
        echo "ok"
    fi

done 

echo -n "generating the docker compose development file..."
rm -f $DEVCONTAINER/setup.new.json 
# fixing the docker-compose.dev.yml file (templates were concatenated, simply removing extra "services:" line)
awk -v pat="services:" '{ if (!seen[$0]++ || $0 !~ pat) print}' $DEVCONTAINER/docker-compose.dev.yml > $DEVCONTAINER/docker-compose.dev.yml.tmp
mv -f $DEVCONTAINER/docker-compose.dev.yml.tmp $DEVCONTAINER/docker-compose.dev.yml
awk -v pat="services:" '{ if (!seen[$0]++ || $0 !~ pat) print}' $DEVCONTAINER/docker-compose.devcontainer.yml > $DEVCONTAINER/docker-compose.devcontainer.yml.tmp
mv -f $DEVCONTAINER/docker-compose.devcontainer.yml.tmp $DEVCONTAINER/docker-compose.devcontainer.yml

echo -e "ok\n"
echo "devcontainer setup successfully completed."
echo -e "---------------------------------------------\n"
