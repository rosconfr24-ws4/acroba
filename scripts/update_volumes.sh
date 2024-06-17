#!/bin/bash

./copy_volume.sh acroba_acroba-data acroba_data 
docker volume rm acroba_acroba-data

./copy_volume.sh acroba_acroba-logs acroba_logs
docker volume rm acroba_acroba-logs

