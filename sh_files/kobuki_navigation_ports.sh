#!/bin/bash

case "$1" in
    -h|--help)
        echo "kobuki_navigation.sh - launches kobuki_navigation.launch with a given mission"
        echo " "
        echo "./kobuki_navigation.sh [options]"
        echo " "
        echo "options:"
        echo "-h, --help          show brief help"
        echo "-m, --mission       loads mission name to se initial position and parameters of the robot"
        exit 0
        ;;
    -m|--mission)
        roslaunch fuzzymar_multi_robot kobuki_navigation.launch using_ports:=True kobuki_id:=$KOBUKI_ID robot_name:=$KOBUKI_NAME mission_name:=$2
        exit 0
        ;;
    *)
      break
      ;;
esac