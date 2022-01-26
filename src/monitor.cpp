/**
 * @Author: Toni Tauler
 * @File: monitor.cpp
 * @Description: TODO
 * @Date: January 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fuzzymar_multi_robot/mission.h>
#include <fuzzymar_multi_robot/missionArray.h>
#include <fuzzymar_multi_robot/action_task.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

#define LOOPHZ 10

// STRUCTS

struct Mission {
    uint8_t id_task;
    float deadline;
		float weight;
		float x;
		float y;
		uint8_t doing_task;
};

// FLAGS
bool first_loop = true;
bool missions_update = false;

// TOPICS
std::string state_pub_topic = "/mission_state";

// Global Variable where save the mission
//std::vector<std::vector<float>> missions; // vector where the tasks are saved {id_task, deadline, weight, x, y, z, doing_task}
std::vector<Mission> missions;

// Functions Declaration
void getMission(std::vector<Mission>* missions, const std::string& mission_path_file);
void publishMission(ros::Publisher* mission_pub);
void publishMarkers(ros::Publisher* vis_pub);
void missionsUpdate();

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void taskUpdateCallback(const fuzzymar_multi_robot::action_task::ConstPtr& task)
{
	for(int i = 0 ; i < missions.size() ; i++)
	{
		if((int)missions[i].id_task == (int)task->id_task)   // updating the doing_task variable
		{
			missions[i].doing_task += 1;   
		}
	}

	missions_update = true;
}


/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "monitor");

  ros::NodeHandle n;

	// Subscribers
  ros::Subscriber task_sub = n.subscribe("/treball", 1000, taskUpdateCallback);

	// Publishers
  ros::Publisher mission_pub = n.advertise<fuzzymar_multi_robot::missionArray>(state_pub_topic, 10);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

  // Variable as buffer to read de keyboard 
  char cmd[1];

  // Getting path to .txt file (mission)
  std::string mission_path_file;
  n.getParam("/monitor/mission_path_file", mission_path_file);   // "/node_name(indicated inside this code)/param_name"

  ros::Rate loop_rate(LOOPHZ);

  while (ros::ok())
  {

    if(first_loop){

        ROS_INFO("Mission file executed: %s",mission_path_file.c_str());
        ROS_INFO("Press ENTER to begin");
        
        std::cin.getline(cmd, 1);

        getMission(& missions, mission_path_file);    // get the info of the current mission

        for(int i = 0 ; i < missions.size() ; i++)
        {
          ROS_INFO("ID: %i , DD: %f , WEIGTH: %f , X: %f , Y: %f , Robots: %i ", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task);
        }

        // VERY FIRST publish mission
        publishMission(&mission_pub);
        

        ROS_INFO("Missions have been published for the first time");

        first_loop = false;
    }

    missionsUpdate();

		if(missions_update) // publish the vector missions when is updated
		{

			publishMission(&mission_pub);

			for(int i = 0 ; i < missions.size() ; i++)
			{
				ROS_INFO("ID: %i , DD: %f , WEIGTH: %f , X: %f , Y: %f , Robots: %i ", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task);
			}

			missions_update = false;
		}

    publishMarkers(&vis_pub);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


/********************************************************************************************
*************************************** FUNCTIONS *******************************************
********************************************************************************************/

void getMission(std::vector<Mission>* missions, const std::string& mission_path_file)
{

    ROS_DEBUG("Inside getMission");
    // clearing mission vector
    missions->clear();

    // reading mission from .txt file
    std::ifstream inputFile(mission_path_file); //"/home/tonitauler/catkin_ws/src/fuzzymar_multi_robot/missions/default.txt"
    std::string line;
    while (std::getline(inputFile, line))
    {
        std::istringstream iss(line);
				Mission aux_mission;
				float task_id, z;
				if (!(iss >> task_id >> aux_mission.deadline >> aux_mission.weight >> aux_mission.x >> aux_mission.y >> z)) { break; } // error
				aux_mission.id_task = (uint8_t)task_id;
				aux_mission.doing_task = 0;

				missions->push_back(aux_mission);
    }
    
}


void publishMission(ros::Publisher* mission_pub)
{

    ROS_DEBUG("Inside publishMission");
    // buffer to be published of 'mission.msg' type
    fuzzymar_multi_robot::mission mission_line;
		fuzzymar_multi_robot::missionArray mission_buff;
    
		for(int i = 0 ; i < missions.size() ; i++)
		{
			mission_line.id_task = missions[i].id_task;
			mission_line.deadline_task = missions[i].deadline;
			mission_line.weight_task = missions[i].weight;
			mission_line.x = missions[i].x;
			mission_line.y = missions[i].y;
			mission_line.doing_task = missions[i].doing_task;

			mission_buff.missions.push_back(mission_line);
		}
    
		mission_buff.vector_size = missions.size();
    //ROS_DEBUG("ID: %f , DD: %f , WEIGTH: %f , X: %f , Y: %f ", mission_vect[0], mission_vect[1], mission_vect[2], mission_vect[3], mission_vect[4]);
   
    mission_pub->publish(mission_buff);
}

void publishMarkers(ros::Publisher* vis_pub)
{
    visualization_msgs::MarkerArray marker;
    visualization_msgs::Marker aux_marker;

    for(int i = 0 ; i < missions.size() ; i++)
    {
        aux_marker.header.frame_id = "map";
        aux_marker.header.stamp = ros::Time::now();
        aux_marker.ns = "tasks";
        aux_marker.id = i;
        aux_marker.type = visualization_msgs::Marker::CYLINDER;
        aux_marker.action = visualization_msgs::Marker::ADD;
        aux_marker.pose.position.x = missions[i].x;
        aux_marker.pose.position.y = missions[i].y;
        aux_marker.pose.position.z = 0.5;
        aux_marker.pose.orientation.x = 0.0;
        aux_marker.pose.orientation.y = 0.0;
        aux_marker.pose.orientation.z = 0.0;
        aux_marker.pose.orientation.w = 1.0;
        aux_marker.scale.x = 0.1;
        aux_marker.scale.y = 0.1;
        aux_marker.scale.z = 1.0;
        aux_marker.color.a = 1.0; // Don't forget to set the alpha!
        aux_marker.color.r = 1.0;
        aux_marker.color.g = 0.0;
        aux_marker.color.b = 0.0;

        marker.markers.push_back(aux_marker);

        //vis_pub->publish(marker.markers[i]);
    }

    vis_pub->publish(marker);

}

void missionsUpdate()
{
  for(uint16_t i = 0 ; i < missions.size() ; i++)
  {
    missions[i].weight = missions[i].weight - ((float)missions[i].doing_task * (1.0/(float)LOOPHZ)); // the weight is decreased due to robot/s work
    ROS_INFO("Task %i has weight: %f", missions[i].id_task, missions[i].weight);
    if(missions[i].weight <= 0)
    {
      missions.erase(missions.begin()+i);// eliminate this task of vector missions
      i--; //because the vector missions decrease 1
    }
  }

  missions_update = true;

}