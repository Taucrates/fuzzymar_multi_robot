/**
 * @Author: Toni Tauler
 * @File: task_ports.cpp
 * @Description: TODO
 * @Date: February 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
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
#include <math.h>

#define PI 3.14159265

#define DEBUGGING 1

std::string state_sub_topic = "/mission_state";

struct Port {
  float x;
  float y;
};

struct Task {
  uint8_t id_task;
  float x;
  float y;
  uint8_t number_ports;
  std::vector<Port> ports;
};

std::vector<Task> missions;

// Parameters with default values
int number_ports = 6;
float ports_dist = 0.3;  
float robot_radius = 0.2;
float inflation = 0.4;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid costmap;

bool got_map = false;
bool map_update = false;
bool got_mission = false;

// FUNCTIONS DECLARATION
geometry_msgs::Pose2D vector2cell(int aux_vector);
int cell2vector(float x, float y);
geometry_msgs::Pose2D vector2pose(int aux_vector);
int pose2vector(float x, float y);
geometry_msgs::Pose2D taskConcentrationPoint(); // calculate the central point doing the average of all tasks position
void setCellValue(geometry_msgs::Pose2D cell, int x, int y);
void setInflation(geometry_msgs::Pose2D cell);
void getPorts();
void setCostmap();
int numMaxPorts();
bool collision(float x , float y); // comprove if a point is in an occupied cell
void possiblePorts(); // erase all the impossible ports and decide the number of designed ports for each task, if it is possible
void printDebug(int max_ports);

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void missionsCallback(const fuzzymar_multi_robot::taskArray::ConstPtr& mission)
{
  
  missions.clear();

  Task task_data;
  Port port_data;
  
  for(int i = 0 ; i < mission->vector_size ; i++)
  {
    task_data.ports.clear();

    task_data.id_task = mission->missions[i].id_task;
    task_data.x = mission->missions[i].x;
    task_data.y = mission->missions[i].y;
    task_data.number_ports = mission->missions[i].number_ports; // TODO have to solve this ERROR:  free(): invalid next size (fast) \n Aborted (core dumped)
    //task_data.number_ports = number_ports;

    for(int j = 0 ; j < number_ports ; j ++)
    {
      port_data.x = 0.0;
      port_data.y = 0.0;

      task_data.ports.push_back(port_data);
    }

    missions.push_back(task_data);
  }

  got_mission = true;

}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
  map.info.resolution = map_msg->info.resolution;
  map.info.width = map_msg->info.width;
  map.info.height = map_msg->info.height;
  map.info.origin = map_msg->info.origin;

  costmap.info.resolution = map_msg->info.resolution;
  costmap.info.width = map_msg->info.width;
  costmap.info.height = map_msg->info.height;
  costmap.info.origin = map_msg->info.origin;

  for(int i = 0 ; i < map_msg->data.size() ; i++) // filling our map
  {
    map.data.push_back(map_msg->data[i]);
    costmap.data.push_back(map_msg->data[i]);
  }

  printf("\nResolució: %6.3f \n", costmap.info.resolution);
  printf("Width: %i \n", costmap.info.width);
  printf("Height: %i \n", costmap.info.height);
  printf("Origen:\n x: %5.2f y: %5.2f\n", costmap.info.origin.position.x, costmap.info.origin.position.y);
  //printf("\n Num cel·les original: %i", map_msg->data.size());
  //printf("\n Num cel·les nostro: %i \n", costmap.data.size());

  map_update = true;

}

/********************************************************************************************
****************************************** MAIN *********************************************
********************************************************************************************/

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "task_ports");
  
  ros::NodeHandle n;

  // SUBSCRIBERS
  ros::Subscriber mission_sub = n.subscribe(state_sub_topic, 1000, missionsCallback);
  ros::Subscriber map_sub = n.subscribe("/map", 1000, mapCallback);
  
  // PUBLISHERS
  ros::Publisher ports_pub = n.advertise<std_msgs::String>("/ports", 1000);
  ros::Publisher costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/costmap_tomeu", 1000);

  ros::Rate loop_rate(5);

  bool got_ports;

  while (ros::ok())
  {
    
    

    if(map_update)
    {
      
      setCostmap();

      got_map = true;
      map_update = false;
    }

    if(got_map)
    {

      costmap_pub.publish(costmap);

    }

    if(got_mission && !got_ports)
    {
      getPorts();

      if(DEBUGGING)
      {
        printDebug(numMaxPorts());
      }

      possiblePorts();

      if(DEBUGGING)
      {
        printDebug(numMaxPorts());
      }

      got_mission = false;
      got_ports = true;

    }

    ros::spinOnce();

    loop_rate.sleep();
   
  }


  return 0;
}

/********************************************************************************************
*************************************** FUNCTIONS *******************************************
********************************************************************************************/

geometry_msgs::Pose2D vector2cell(int aux_vector)
{
  geometry_msgs::Pose2D pose;

  pose.x = int(aux_vector / costmap.info.width);
  pose.y = aux_vector % costmap.info.width;

  return pose;
}

int cell2vector(float x, float y)
{
  int i = abs(x * costmap.info.width) + y;

  return i;
}

geometry_msgs::Pose2D vector2pose(int aux_vector)
{

}

int pose2vector(float x, float y)
{

}

geometry_msgs::Pose2D taskConcentrationPoint()
{
  geometry_msgs::Pose2D pose;
  float x = 0.0, y = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    x += missions[i].x;
    y += missions[i].y;
  }

  pose.x = x / missions.size();
  pose.y = y / missions.size();

  return pose;

}

void setCellValue(geometry_msgs::Pose2D cell, int x, int y)
{
  if(cell.x + x <= map.info.height && cell.x + x >= 0 && cell.y + y <= map.info.width && cell.y + y >= 0)
  {
    costmap.data[cell2vector(cell.x + x , cell.y + y)] = 200;
  }
  if(cell.x + x <= map.info.height && cell.x + x >= 0 && cell.y - y <= map.info.width && cell.y - y >= 0)
  {
    costmap.data[cell2vector(cell.x + x , cell.y - y)] = 200;
  }
  if(cell.x - x <= map.info.height && cell.x - x >= 0 && cell.y + y <= map.info.width && cell.y + y >= 0)
  {
    costmap.data[cell2vector(cell.x - x , cell.y + y)] = 200;
  }
  if(cell.x - x <= map.info.height && cell.x - x >= 0 && cell.y - y <= map.info.width && cell.y - y >= 0)
  {
    costmap.data[cell2vector(cell.x - x , cell.y - y)] = 200;
  }
  
}

void setInflation(geometry_msgs::Pose2D cell)
{
  int cells_dist = inflation / map.info.resolution;
  int loop = 1;

  for(int x = 0 ; x <= cells_dist ; x++)
  {
    if(x < cells_dist/2)
    {
      for(int y = 0 ; y <= cells_dist ; y++)
      {

        setCellValue(cell, x, y);
        
      }
      
    } else {

      for(int y = 0 ; y <= cells_dist - loop; y++)
      {

        setCellValue(cell, x, y);
        
      }

      loop++;
    }
  }

  /*for(int x = cell.x - cells_dist ; x <= cell.x + cells_dist ; x++)
  {
    for(int y = int(cell.y - ((cells_dist - a saber)/2)) ; y <= cell.y + ((cells_dist - a saber)/2) ; y++)
    {

    }
  } */

}

void setCostmap()
{
  for(int i = 0 ; i < map.data.size() ; i++)
  {
    if(map.data[i] > 0)  // cell is occupied
    {
      setInflation(vector2cell(i));
    }
  }
  
}

void getPorts()
{
  float deg = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    for(int j = 0 ; j < number_ports ; j++)
    {
      deg = j * 360/number_ports;

      missions[i].ports[j].x = missions[i].x + (ports_dist + robot_radius) * sin(deg*PI/180);
      missions[i].ports[j].y = missions[i].y + (ports_dist + robot_radius) * cos(deg*PI/180);

    }
    //ROS_INFO("____________________Task %i has %i ports", missions[i].id_task, missions[i].ports.size());
  }
}

int numMaxPorts()
{
  int num_ports = 0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    for(int j = 0 ; j < missions[i].ports.size() ; j++)
    {
      if(num_ports < missions[i].ports.size())
      {
        num_ports = missions[i].ports.size();
        
      }
    }
    ROS_INFO("Task %i has %i ports", missions[i].id_task, num_ports);
    
  }

  return num_ports;
}

bool collision(float x , float y)
{
  ROS_INFO(" Comprovacio port localitzat a x: %5.2f y: %5.2f", x , y);
  /*if(costmap.data[pose2vector(x , y)] > 0)
  {
    printf("Ocupat\n");
    return true;
  }*/
  printf("Viable\n");
  return false;
}

void possiblePorts()
{
  for(int i = 0 ; i < missions.size() ; i++)
  {

    for(int j = 0 ; j < missions[i].number_ports ; j++)
    {
      if(collision(missions[i].ports[j].x, missions[i].ports[j].y))
      {
        missions[i].ports.erase(missions[i].ports.begin()+j);// eliminate this port of vector ports
        j--; //because the vector ports decrease 1
      }
    }

  }

}

void printDebug(int max_ports)
{
  printf("+------+------------------+");
  for(int i = 0 ; i < max_ports ; i++)
  {
    printf("------------------+");
  }
  printf("\n");
  
  printf("|  ID  |     LOCATION     |");
  for(int i = 1 ; i <= max_ports ; i++)
  {
    printf("      PORT %i      |", i);
  }
  printf("\n");

  printf("+------+------------------+");
  for(int i = 0 ; i < max_ports ; i++)
  {
    printf("------------------+");
  }
  printf("\n");

  for(int i = 0 ; i < missions.size() ; i++)
  {
    printf("|  %2i  | x:%5.2f  y:%5.2f |", missions[i].id_task, missions[i].x, missions[i].y);

    for(int j = 0 ; j < max_ports ; j++)
    {
      printf(" x:%5.2f  y:%5.2f |", missions[i].ports[j].x, missions[i].ports[j].y);
    }
    printf("\n");
  }
  printf("+------+------------------+");
  for(int i = 0 ; i < max_ports ; i++)
  {
    printf("------------------+");
  }
  printf("\n");
}