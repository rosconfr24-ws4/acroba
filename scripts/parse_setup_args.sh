#!/bin/bash

usage()
{
    echo "Usage: parse_setup_args.sh [--CELL=<cell-config-name>] [--X11=<YES|NO>] [--GPU=<YES|NO>] [--PROJECT=<acroba|acroba-dev>] [--PROFILE=<acroba|drl>]"
}

#----------------------------    
# Default args 
#----------------------------    

CELL=NONE
X11=YES
GPU=YES 
PROJECT=acroba
PROFILE=acroba
VG=YES

dev=false

#----------------------------
# Parsing args
#----------------------------
    
LONG_OPTIONS="CELL:,X11:,GPU:,PROJECT:,PROFILE:,VG:,dev"
OPTIONS="c:x:g:p:P:v:d"

# Parse arguments
PARSED=$(getopt --options="$OPTIONS" --longoptions="$LONG_OPTIONS" -- "$@")
if [[ $? -ne 0 ]]; then
    # getopt should have complained about wrong arguments to stdout
    exit 2
fi    

eval set -- "$PARSED"
while true; do
    case "$1" in
        --CELL)
            CELL="$2"
            shift 2
            ;;
        --X11)
            X11="$2"
            shift 2
            ;;
        --GPU)
            GPU="$2"
            shift 2
            ;;
        --VG)
            VG="$2"
            shift 2
            ;;
        --PROJECT)
            PROJECT="$2"
            shift 2
            ;;
        --PROFILE)
            PROFILE="$2"
            shift 2
            ;;
        --dev)
            dev="true"
            shift 1
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

echo "" 
echo "#-----------------------"
echo "# Docker config options"
echo "#  |-> X11: $X11"
echo "#  |-> GPU: $GPU"
echo "#  |-> VG: $VG"

if [ "$dev" == "false" ]; then
    echo "#  |-> CELL: $CELL"
    echo "#  |-> project-name: $PROJECT"
    echo "#  |-> profile: $PROFILE"
fi

if [ "$GPU" == "YES" ]; then
    _GPU="gpu"
else
    _GPU="cpu"
fi

if [ "$X11" == "YES" ]; then
    _X="x"
else
    _X="novnc"
fi

if [ "$VG" == "HL" ]; then
    _VG="headless"
    _NET="host"
elif [ "$VG" == "WIN" ]; then 
    _VG="win"
    _NET="bridge"
elif [ "$VG" == "WSL" ]; then 
    _VG="wsl"
    _NET="bridge"
else 
    _VG="default"
    _NET="host"
fi

echo "#-----------------------"
echo "#" 
echo "# (_GPU: $_GPU, _X: $_X, _VG: $_VG, _NET: $_NET)"
echo "" 

