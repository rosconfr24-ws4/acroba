#!/bin/bash 

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <volume_from> <volume_to>"
    exit 1
fi

volume_from="$1"
volume_to="$2"


if docker volume inspect "$volume_to" &> /dev/null; then
    echo "Volume $volume_to already exists."
else
    echo "Volume $volume_tp does not exist, creating."
    docker volume create --name $volume_to
fi

echo "Copying volume $volume_from to volume $volume_to" 
docker run --rm -it -v $volume_from:/from -v $volume_to:/to alpine ash -c "cd /from ; cp -av . /to"
