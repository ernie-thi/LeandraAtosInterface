#!/usr/bin/env bash

## script to launch atos interface node
# source ws
source ./install/setup.bash
sleep 0.5
source ../leandros/install/setup.bash
sleep 0.5

# launch ros
ros2 launch leandra_atos_interface leandra_atos_interface.launch.py
