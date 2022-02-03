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
        roslaunch fuzzymar_multi_robot kobuki_navigation.launch kobuki_id:=$KOBUKI_ID robot_name:=$KOBUKI_NAME initial_pose_label:=$2
        exit 0
        ;;
    *)
      break
      ;;
esac