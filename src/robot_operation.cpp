/**
 * @Author: Toni Tauler
 * @File: robot_operation.cpp
 * @Description: TODO
 * @Date: January 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fuzzymar_multi_robot/mission.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

bool new_mission_state = false;
std::string state_sub_topic = "mission_state";

// Global Variable where save the mission
std::vector<std::vector<float>> missions;

void missionStateCallback(const fuzzymar_multi_robot::mission::ConstPtr& mission)
{
  
  float id, deadline, weight, x, y; // In a 3D navigation have to add z pose

  id = (float)mission->id_task;
  deadline = mission->deadline_task;
  weight = mission->weight_task;
  x = mission->location_task.x;
  y = mission->location_task.y;
  //z = mission->location_task.z;  // 3D navigation

  // building mission vector
  std::vector<float> aux_mission = {id, deadline, weight, x, y}; // In a 3D navigation have to add z pose

  missions.push_back(aux_mission);

  new_mission_state = true;
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_operator");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe(state_sub_topic, 1000, missionStateCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // When new mission state is received by monitor
    if(new_mission_state)
    {
      for(int i = 0 ; i < missions.size() ; i++)
      {
        ROS_INFO("ID: %f , DD: %f , WEIGTH: %f , X: %f , Y: %f", missions[i][0], missions[i][1], missions[i][2], missions[i][3], missions[i][4]);
      }

      //TODO
      //TODO
      //TODO
      //TODO

      new_mission_state = false;

    }

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}