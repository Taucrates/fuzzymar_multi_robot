/**
 * @Author: Toni Tauler
 * @File: monitor.cpp
 * @Description: TODO
 * @Date: January 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <fuzzymar_multi_robot/task.h>
#include <fuzzymar_multi_robot/taskArray.h>
#include <fuzzymar_multi_robot/action_task.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

#define LOOPHZ 5

// STRUCTS

struct Task {
    uint8_t id_task;
    int number_ports;
    float deadline;
		float weight;
		float x;
		float y;
		uint8_t doing_task;
    int utility;
};

// FLAGS
bool first_loop = true;
bool missions_update = false;
bool mission_finalized = false;

// TOPICS
std::string state_pub_topic = "/mission_state";

// Global Variable where save the mission
std::vector<Task> missions;
ros::Time aux_time;
ros::Time mission_time;

// Functions Declaration
double time2Float(ros::Time time);
void getMission(std::vector<Task>* missions, const std::string& mission_path_file);
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
      if(task->active){
			  missions[i].doing_task += 1; // A robot reach and begin to work in this task
      } else {
        missions[i].doing_task -= 1; // A robot leave the task
      }
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
  ros::Publisher mission_pub = n.advertise<fuzzymar_multi_robot::taskArray>(state_pub_topic, 10);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1);
  ros::Publisher end_mission_pub = n.advertise<std_msgs::Empty>( "/mission_accomplished", 10);

  // Variable as buffer to read de keyboard 
  char cmd[1];

  // Getting path to .txt file (mission)
  std::string mission_path_file;
  n.getParam("/monitor/mission_path_file", mission_path_file);   // "/node_name(indicated inside this code)/param_name"

  ros::Rate loop_rate(LOOPHZ);

  


  while (ros::ok())
  {

    if(first_loop){ // first loop

        //printf("Task file executed: %s\n",mission_path_file.c_str());
        printf("\nPress ENTER to begin. \n");
        
        std::cin.getline(cmd, 1);

        getMission(& missions, mission_path_file);    // get the info of the current mission

        // VERY FIRST publish mission
        publishMission(&mission_pub);
        

        printf("Mission has been published for the first time. \n");

        mission_time = ros::Time::now();

        mission_finalized = false;
        first_loop = false;
    } 
    else  // not the first loop
    {
      if(!mission_finalized) // vector missions is updated
      {
        missionsUpdate();
        aux_time = ros::Time::now();
      }

      if(missions_update) 
      {

        publishMission(&mission_pub); // publish the vector missions when is updated

        printf("\n \n \n \n \n \n \n");
        printf("+----+-------+--------+--------+-------+-------+-----+-----+\n");
        printf("| ID | Ports |   DD   | WEIGHT |   X   |   Y   | Rob |  U  |\n");
        printf("+----+-------+--------+--------+-------+-------+-----+-----+\n");
        for(int i = 0 ; i < missions.size() ; i++)
        {
          printf("| %2i |   %i   | %6.2f | %5.2f  | %5.2f | %5.2f |  %1i  | %3i |\n", missions[i].id_task, missions[i].number_ports, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task, missions[i].utility);
        }
        printf("+----+-------+--------+--------+-------+-------+-----+-----+\n");
        printf("\nCurrent time: %6.3f secs\n", (double)(time2Float(ros::Time::now()) - time2Float(mission_time)));

        if(missions.size() == 0) // the mission is accomplished
        {
          double end_time = time2Float(ros::Time::now()) - time2Float(mission_time);
          printf("Mission accomplished in %6.3f secs\n", end_time);
          std_msgs::Empty end_msg;
          end_mission_pub.publish(end_msg);
          
          mission_finalized = true;
          first_loop = true;
        } 

        publishMarkers(&vis_pub);

        missions_update = false;
      }
    }


    ros::spinOnce();
    
    loop_rate.sleep();
  }

  return 0;
}


/********************************************************************************************
*************************************** FUNCTIONS *******************************************
********************************************************************************************/

double time2Float(ros::Time time)
{
  double timer = time.sec + time.nsec*(1.0/1000000000.0);

  return timer;
}

void getMission(std::vector<Task>* missions, const std::string& mission_path_file)
{

    // clearing mission vector
    missions->clear();

    // reading mission from .txt file
    std::ifstream inputFile(mission_path_file); //"/home/tonitauler/catkin_ws/src/fuzzymar_multi_robot/missions/default.txt"
    std::string line;
    while (std::getline(inputFile, line))
    {
        std::istringstream iss(line);
				Task aux_task;
				float task_id, z;
				if (!(iss >> task_id >> aux_task.number_ports >> aux_task.deadline >> aux_task.weight >> aux_task.x >> aux_task.y >> z >> aux_task.utility)) { break; } // error
				aux_task.id_task = (uint8_t)task_id; // this way because ifstream doesn't read sized variables (uint8_t, float32, etc)
				aux_task.doing_task = 0;

				missions->push_back(aux_task);
    }
    
}


void publishMission(ros::Publisher* mission_pub)
{

    // buffer to be published of 'mission.msg' type
    fuzzymar_multi_robot::task mission_line;
		fuzzymar_multi_robot::taskArray mission_buff;
    
		for(int i = 0 ; i < missions.size() ; i++)
		{
			mission_line.id_task = missions[i].id_task;
      mission_line.number_ports = missions[i].number_ports;
			mission_line.deadline_task = missions[i].deadline;
			mission_line.weight_task = missions[i].weight;
			mission_line.x = missions[i].x;
			mission_line.y = missions[i].y;
			mission_line.doing_task = missions[i].doing_task;
      mission_line.utility = missions[i].utility;

			mission_buff.missions.push_back(mission_line);
		}
    
		mission_buff.vector_size = missions.size();
   
    mission_pub->publish(mission_buff);
}

void publishMarkers(ros::Publisher* vis_pub)
{
    visualization_msgs::MarkerArray marker;
    visualization_msgs::Marker aux_marker;
    aux_marker.action = visualization_msgs::Marker::DELETEALL;

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

    }

    vis_pub->publish(marker);

}

void missionsUpdate()
{

  for(uint16_t i = 0 ; i < missions.size() ; i++)
  {
    // OLD VERSION missions[i].weight -= (float)missions[i].doing_task * (1.0/(float)LOOPHZ); // the weight is decreased due to robot/s work

    missions[i].weight -= (float)missions[i].doing_task * (time2Float(ros::Time::now()) - time2Float(aux_time));

    ROS_DEBUG("Task %i has weight: %f", missions[i].id_task, missions[i].weight);
    if(missions[i].weight <= 0)
    {
      missions.erase(missions.begin()+i);// eliminate this task of vector missions
      i--; //because the vector missions decrease 1
    }
  }

  missions_update = true;

}