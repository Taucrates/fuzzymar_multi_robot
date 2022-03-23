#!/bin/bash

case "$1" in
    -h|--help)
        echo "kobuki_navigation.sh - launches kobuki_navigation.launch with a given mission"
        echo " "
        echo "./kobuki_navigation.sh [options]"
        echo " "
        echo "options:"
        echo "-h, --help          show brief help"
        echo "-i, --initial       loads initial_pose from fuzzymar_multi_robot/missions/initial_pose folder"
        exit 0
        ;;
    -i|--initial)
    -a|--alpha)
    -b|--beta)
    -g|--gamma)
    -d|--decision)
        roslaunch fuzzymar_multi_robot kobuki_navigation.launch using_ports:=True kobuki_id:=$KOBUKI_ID robot_name:=$KOBUKI_NAME initial_pose_label:=$2 alpha_utility:=$3 beta_distance:=$4 gamma_ports:=$5 possibilistic:=$6
        exit 0
        ;;
    *)
      break
      ;;
esac