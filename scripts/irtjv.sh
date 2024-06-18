#!/bin/bash

sudo echo '{
    "insecure-registries" : [ "https://docker.irtjv.local" ]
}' >> /etc/docker/daemon.json
sudo systemctl restart docker
