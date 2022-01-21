/**
 * @Author: Toni Tauler
 * @File: monitor.cpp
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

bool first_loop = true;
std::string state_pub_topic = "mission_state";


// Functions
void getMission(std::vector<std::vector<float>>* missions, const std::string& mission_path_file);
void publishMission(const std::vector<float>& mission_vect, ros::Publisher* mission_pub);


// Main
int main(int argc, char **argv)
{

  ros::init(argc, argv, "monitor");

  ros::NodeHandle n;

  ros::Publisher mission_pub = n.advertise<fuzzymar_multi_robot::mission>(state_pub_topic, 10);

  // Global Variable where save the mission
  std::vector<std::vector<float>> missions;

  // Global Variable as buffer to read de keyboard 
  char cmd[1];

  // Getting path to .txt file (mission)
  std::string mission_path_file;
  n.getParam("/monitor/mission_path_file", mission_path_file);   // "/node_name/param_name"

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    if(first_loop){

        ROS_INFO("Mission file executed: %s",mission_path_file.c_str());
        ROS_INFO("Press ENTER to begin");
        
        std::cin.getline(cmd, 1);

        getMission(& missions, mission_path_file);    // get the info of the current mission

        // VERY FIRST publish mission
        for(int i = 0; i < missions.size(); i++){
            publishMission(missions[i], &mission_pub);
        }

        ROS_INFO("Missions have been published for the first time");

        first_loop = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

void getMission(std::vector<std::vector<float>>* missions, const std::string& mission_path_file){

    ROS_DEBUG("Inside getMission");
    // clearing mission vector
    missions->clear();

    // reading mission from .txt file
    std::ifstream inputFile(mission_path_file); //"/home/tonitauler/catkin_ws/src/fuzzymar_multi_robot/missions/default.txt"
    std::string line;
    while (std::getline(inputFile, line))
    {
        std::istringstream iss(line);
        float id, deadline, weight, x, y, z;
        if (!(iss >> id >> deadline >> weight >> x >> y >> z)) { break; } // error

        // building mission vector
        std::vector<float> aux_mission = {id, deadline, weight, x, y, z};
        missions->push_back(aux_mission);
        ROS_DEBUG("ID: %f , DD: %f , WEIGTH: %f , X: %f , Y: %f ", aux_mission[0], aux_mission[1], aux_mission[2], aux_mission[3], aux_mission[4]);
    }
    
}


void publishMission(const std::vector<float>& mission_vect, ros::Publisher* mission_pub){

    ROS_DEBUG("Inside publishMission");
    // buffer to be published of 'mission.msg' type
    fuzzymar_multi_robot::mission mission_buff;
    
    mission_buff.id_task = mission_vect[0];
    mission_buff.deadline_task = mission_vect[1];
    mission_buff.weight_task = mission_vect[2];
    mission_buff.location_task.x = mission_vect[3];
    mission_buff.location_task.y = mission_vect[4];
    mission_buff.location_task.z = mission_vect[5];

    ROS_DEBUG("ID: %f , DD: %f , WEIGTH: %f , X: %f , Y: %f ", mission_vect[0], mission_vect[1], mission_vect[2], mission_vect[3], mission_vect[4]);
   
    mission_pub->publish(mission_buff);
}