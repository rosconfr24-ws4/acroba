#!/bin/bash

# A mapping to infer the "module" name from the docker service name
declare -A service_to_module

service_to_module["virtual-gym"]="VirtualGym"
service_to_module["acroblock"]="GUI"
service_to_module["skills-vg"]="skills"

get_module_name() {
    local service="$1"
    if [[ -v service_to_module["$service"] ]]; then
        echo "${service_to_module["$service"]}"
    else
        echo "$service"
    fi
}
