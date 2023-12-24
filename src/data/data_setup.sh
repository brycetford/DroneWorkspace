#! /usr/bin/env bash

# Used to setup all data-sets

directory="/DroneWorkspace/data/"

echo "Parsing Data Sets"

# Find all logs with .ulg extention
logs=($(find "$directory" -type f -name "*.ulg"))

for item in ${logs[*]}
do
    echo $item
    ulog2csv -m 'actuator_outputs,estimator_states,estimator_selector_status,vehicle_attitude,vehicle_local_position,home_position' $item
done