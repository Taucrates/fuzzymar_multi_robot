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
#include <fuzzymar_multi_robot/task_w_ports.h>
#include <fuzzymar_multi_robot/task_w_portsArray.h>
#include <fuzzymar_multi_robot/action_task_w_ports.h>
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
  float yaw;
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
int number_ports = 4;
float ports_dist = 0.35;  
float robot_radius = 0.2;
float inflation = 0.45;

nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid costmap;

bool got_map = false;
bool map_update = false;
bool got_mission = false;

// FUNCTIONS DECLARATION
geometry_msgs::Pose2D vector2cell(int aux_vector);
int cell2vector(float x, float y);
geometry_msgs::Pose2D world2map(float x, float y);
//int pose2vector(float x, float y);
geometry_msgs::Pose2D taskConcentrationPoint(); // calculate the central point doing the average of all tasks position
void setCellValue(geometry_msgs::Pose2D cell, int x, int y);
void setInflation(geometry_msgs::Pose2D cell);
void getPorts();
void setCostmap();
int numMaxPorts();
bool collision(float x , float y); // comprove if a point is in an occupied cell
void possiblePorts(); // erase all the impossible ports and decide the number of designed ports for each task, if it is possible
void printDebug(int max_ports);
void publishPorts(ros::Publisher* ports_pub);

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void missionsCallback(const fuzzymar_multi_robot::task_w_portsArray::ConstPtr& mission)
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
  ros::Publisher ports_pub = n.advertise<fuzzymar_multi_robot::task_w_portsArray>("/ports", 1000);
  ros::Publisher costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/costmap_tomeu", 1000);

  // PARAMS
  n.getParam("/task_ports/num_ports", number_ports);   // "/node_name(indicated inside this code)/param_name" 

  printf("\nTrying to assign %i ports per task.\n", number_ports);

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

      publishPorts(&ports_pub);
      got_mission = false;
      got_ports = true;
      ros::shutdown(); //kill the node because all work is done

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
  int i = abs(x * costmap.info.width) + abs(y);

  return i;
}

geometry_msgs::Pose2D world2map(float x , float y) //for row-major order of the OccupancyGrid format
{
  geometry_msgs::Pose2D pose;


  pose.x = floor((x - costmap.info.origin.position.x) / costmap.info.resolution);
  pose.y = floor((y - costmap.info.origin.position.y) / costmap.info.resolution);
  //pose.x = costmap.info.width*costmap.info.resolution + (costmap.info.origin.position.x - x);
  //pose.y = costmap.info.height*costmap.info.resolution + (costmap.info.origin.position.y - y);

  return pose;
}

/*int pose2vector(float x, float y)
{

}*/

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
    if(x < cells_dist/2) // this if else construction give "circular" form to our inflation
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
      missions[i].ports[j].yaw = atan2((missions[i].y - missions[i].ports[j].y) , (missions[i].x - missions[i].ports[j].x));

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
    //ROS_INFO("Task %i has %i ports", missions[i].id_task, num_ports);
    
  }

  return num_ports;
}

bool collision(float x , float y)
{
  
  geometry_msgs::Pose2D map_pose = world2map(x , y);

  //ROS_INFO(" Comprovacio port localitzat a x: %5.2f y: %5.2f", x , y);
  //ROS_INFO("                      mapa-> a x: %5.2f y: %5.2f", map_pose.x , map_pose.y);

  int8_t cost = costmap.data[map_pose.x + map_pose.y * (costmap.info.width)];
  if (cost != 0 && cost != -1 && cost != 255) // free && unknown
  {
    //printf("                Ocupat\n");
    return true;
  }
  //printf("                Viable\n");
  return false;
}

void possiblePorts()
{
  for(int i = 0 ; i < missions.size() ; i++)
  {

    for(int j = 0 ; j < missions[i].ports.size() ; j++)
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
  printf("+------+------------------++");
  for(int i = 0 ; i < max_ports ; i++)
  {
    printf("-----------------------------+");
  }
  printf("\n");
  
  printf("|  ID  |     LOCATION     ||");
  for(int i = 1 ; i <= max_ports ; i++)
  {
    printf("           PORT  %i           |", i);
  }
  printf("\n");

  printf("+------+------------------++");
  for(int i = 0 ; i < max_ports ; i++)
  {
    printf("-----------------------------+");
  }
  printf("\n");

  for(int i = 0 ; i < missions.size() ; i++)
  {
    printf("|  %2i  | x:%5.2f  y:%5.2f ||", missions[i].id_task, missions[i].x, missions[i].y);

    for(int j = 0 ; j < max_ports ; j++)
    {
      if(j < missions[i].ports.size())
      {
        printf(" x:%5.2f  y:%5.2f  yaw:%5.2f |", missions[i].ports[j].x, missions[i].ports[j].y, missions[i].ports[j].yaw);
      } else {
        printf("                             |");
      }
      
    }
    printf("\n");
  }
  printf("+------+------------------++");
  for(int i = 0 ; i < max_ports ; i++)
  {
    printf("-----------------------------+");
  }
  printf("\n");
}

void publishPorts(ros::Publisher* ports_pub)
{

    // buffer to be published of 'mission.msg' type
    fuzzymar_multi_robot::port port_aux;
    fuzzymar_multi_robot::task_w_ports mission_line;
		fuzzymar_multi_robot::task_w_portsArray mission_buff;
    
		for(int i = 0 ; i < missions.size() ; i++)
		{
      mission_line.ports.clear();
      
			mission_line.id_task = missions[i].id_task;
      mission_line.number_ports = missions[i].number_ports;
			mission_line.x = missions[i].x;
			mission_line.y = missions[i].y;

      for(int j = 0 ; j < missions[i].ports.size() ; j++)
      {
        port_aux.x = missions[i].ports[j].x;
        port_aux.y = missions[i].ports[j].y;
        port_aux.yaw = missions[i].ports[j].yaw;
        port_aux.id_port = j+1;
        port_aux.id_kobuki = 0;  // means any kobuki willing this port

        mission_line.ports.push_back(port_aux);
      }

			mission_buff.missions.push_back(mission_line);
		}
    
		mission_buff.vector_size = missions.size();
   
    ports_pub->publish(mission_buff);
}