#!/bin/bash

create_platform_devcontainer() {    
    # Usage: 
    #   create_platform_devcontainer <source_file> <target_file> <local_workspace> [<output_file>=<target_file>]
    # Description:
    #   merges the two devcontainer.json setups in files <source_file> and <target_file>, 
    #   by taking the fields from <source_file> not in <target_file> 
    #   except for `workspaceFolder`` which is taken from target_file and overwritten by source_file entry. 
    # used to generate platform integrated modules devcontainer files from standalone devcontainer files 

    source_file="$1"
    target_file="$2"
    new_local_workspace="$3"
    output_file="${4:-$2}"

    if [ -f "$source_file" ]; then 
        # Remove C-style comments from the source file
        sed 's|//.*$||' "$source_file" > "$output_file.source"
    
        # exclude all fields from the target_file, except workspaceFolder 
        exclude_target_fields=($(jq -r 'keys_unsorted[]' $target_file))
        # keeping workspaceFolder, will be overwritten and used as default value
        exclude_target_fields=("${exclude_target_fields[@]/workspaceFolder}")

        # exclude all fields related to building and workspace mount
        exclude_source_fields=("context" "dockerFile" "image" "build" "workspaceMount") 
        exclude_fields=("${exclude_target_fields[@]}" "${exclude_source_fields[@]}")

        jq_cmd='del('
        for field in "${exclude_fields[@]}"; do
            jq_cmd="$jq_cmd.\"$field\", "
        done
        jq_cmd="${jq_cmd%, })"

        # excluding fields above from the source json file 
        jq "$jq_cmd" "$output_file.source" > "$output_file.merge"

        # merging with target json file 
        jq -s ".[1] * .[0]"  "$output_file.merge" "$target_file" > "$output_file.tmp"
        
        cp "$output_file.tmp" $output_file
    fi 
    
    if [ "$new_local_workspace" ]; then 
        sed -i "s|\${localWorkspaceFolder}|$new_local_workspace|g" $output_file
    fi

    rm -f "$output_file.merge" "$output_file.source" "$output_file.tmp"
}


create_container_features_docker_compose() {    
    # hack to overcome the limitations of the devcontainer cli
    # which does not support multi devcontainer features setup
    devcontainer_file="$1"
    local_workspace="$2"
    docker_compose_file="$3"

    # exclude all fields related to building and workspace mount
    exclude_fields=("name" "dockerComposeFile" "service" "context" "dockerFile" "image" "build" "workspaceMount" "workspaceFolder") 
        
    jq_cmd='del('
    for field in "${exclude_fields[@]}"; do
        jq_cmd="$jq_cmd.\"$field\", "
    done
    jq_cmd="${jq_cmd%, })"

    # excluding fields above from the source json file 
    jq "$jq_cmd" "$devcontainer_file" > "$devcontainer_file.features"
    metadata=$(jq -c -s . "$devcontainer_file.features")
    #escaping dollars 
    metadata=$(echo "$metadata" | sed 's/\$/\$\$/g')
    
    echo "    labels:" >> $docker_compose_file
    echo -e "      - 'devcontainer.local_folder=$local_workspace'" >> $docker_compose_file
    echo -e "      - 'devcontainer.config_file=$devcontainer_file'" >> $docker_compose_file
    echo -e "      - 'devcontainer.metadata=$metadata'\n" >> $docker_compose_file

    if jq -e '.mounts | if type == "array" then length > 0 else false end' "$devcontainer_file" > /dev/null; then
        echo -e "    volumes:" >> $docker_compose_file
        jq -r '.mounts | .[]' "$devcontainer_file" | while IFS= read -r mount; do
            source=$(echo "$mount" | awk -F',' '{print $1}' | sed 's/source=//')
            target=$(echo "$mount" | awk -F',' '{print $2}' | sed 's/target=//')
            source=$(echo "$source" | sed "s|\${localWorkspaceFolder}|$local_workspace|;s/\"//g; s/\/\+/\//g")

            if [ -n "$source" ] && [ -n "$target" ]; then
                echo -e "      - $source:$target" >> $docker_compose_file
            fi
        done
    fi 

    rm -f "$output_file.features"
}


