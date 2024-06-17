#!/usr/bin/env bash

# script is called automatically, **once**, when setting up the docker container. 

#--------------------------------------------------------
# Shipping cell config files & packages to the platform 
# (such that they can be accessed by other modules)
#---------------------------------------------------------

#--------------------------------------------------------
# cell config files (scripts, launch files) to be shared
mkdir -p /home/acroba/share/cell-config-roscon
cp -r /home/acroba/cell-config/scripts /home/acroba/share/cell-config-roscon/scripts
cp -r /home/acroba/cell-config/launch /home/acroba/share/cell-config-roscon/launch

#------------------------------------------------------------
# copying (overwritting) cell config files in the data volume
CELL_CONFIG_DIR=/home/acroba/data/cell-config/sigma
rm -rf $CELL_CONFIG_DIR

#-----------------------
# 1. cell-description
mkdir -p $CELL_CONFIG_DIR/description

cp -r $SRC_DIR/cell_setup/acroba_description $CELL_CONFIG_DIR/description/
cp -r $SRC_DIR/cell_setup/universal_robot/ur_description $CELL_CONFIG_DIR/description/

echo "$CELL_CONFIG_DIR" > /home/acroba/share/.setup/cell-config


#-----------------------------
# Ros setup
#-----------------------------
 
/ros_entrypoint.sh "$@"