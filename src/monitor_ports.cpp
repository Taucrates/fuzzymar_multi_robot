/**
 * @Author: Toni Tauler
 * @File: monitor_ports.cpp
 * @Description: TODO
 * @Date: February 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <fuzzymar_multi_robot/task_w_ports.h>
#include <fuzzymar_multi_robot/task_w_portsArray.h>
#include <fuzzymar_multi_robot/action_task_w_ports.h>
#include <fuzzymar_multi_robot/time_task.h>
#include <fuzzymar_multi_robot/taskObjective.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include <fuzzymar_multi_robot/token.h>

#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <bits/stdc++.h>
#include <ctime>
#include <time.h>

#define LOOP_RATE 10

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"

// STRUCTS

struct Task {
    uint8_t id_task;
    int number_ports;
    float deadline;
		float weight;
		float x;
		float y;
		uint8_t doing_task;
    double utility;
    std::vector<fuzzymar_multi_robot::port> ports;
};

// FLAGS
bool first_loop = true;
bool missions_update = false;
bool mission_finalized = false;
bool got_ports = false;
bool paintGreen[5] = {false, false, false, false, false};

// TOPICS
std::string state_pub_topic = "/mission_state";

// Global Variable where save the mission
std::vector<Task> missions;
ros::Time aux_time;
bool aux_time_initialized = false;
ros::Time mission_time;
int task_working[5] = {0, 0, 0, 0, 0};

int num_tasks = 0;

// Functions Declaration
double time2Float(ros::Time time);
void getMission(std::vector<Task>* missions, const std::string& mission_path_file);
void publishMission(ros::Publisher* mission_pub);
void publishMarkers(ros::Publisher* vis_pub);
void publishTimeTask(ros::Publisher& time_pub, int id_task, int sec, int nsec, float deadline, float u_max, int total_ports);
void missionsUpdate(ros::Publisher& time_pub);
void cleanprintGreen();   // avoid paint green one cycle, when robot change task the interface
void printInterface();
std::string current_date();



// SEQUENCIAL
void publishToken(ros::Publisher* token_pub);
fuzzymar_multi_robot::token token_msg;

bool publish_token = true;
uint8_t token_counter = 1;
uint8_t aux_token_cnt = 0;
uint8_t token = 1;

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void taskUpdateCallback(const fuzzymar_multi_robot::action_task_w_ports::ConstPtr& task)
{
	for(int i = 0 ; i < missions.size() ; i++)
	{
		if((int)missions[i].id_task == (int)task->id_task)   // updating the doing_task variable
		{
      if(task->active){
			  missions[i].doing_task += 1; // A robot reach and begin to work in this task
        paintGreen[(task->id_kobuki)-1] = true;
        task_working[(task->id_kobuki)-1] = (int)task->id_task;
        missions_update = true;
        return;
      } else {
        missions[i].doing_task -= 1; // A robot leave the task (task no finished)
        paintGreen[(task->id_kobuki)-1] = false;
        task_working[(task->id_kobuki)-1] = 0;
        missions_update = true;
        return;
      }
		} 
	}
  paintGreen[(task->id_kobuki)-1] = false; // A robot leave the task (task finished)
	missions_update = true;
}

void portsCallback(const fuzzymar_multi_robot::task_w_portsArray::ConstPtr& ports)
{

  fuzzymar_multi_robot::port aux_port;
  
    for(int i = 0 ; i < missions.size() ; i++)
    {

      for(int j = 0 ; j < ports->missions[i].ports.size() ; j++)
      {
        
        aux_port.x = ports->missions[i].ports[j].x;
        aux_port.y = ports->missions[i].ports[j].y;
        aux_port.yaw = ports->missions[i].ports[j].yaw;
        aux_port.id_port = ports->missions[i].ports[j].id_port;
        aux_port.id_kobuki = 0; // means any kobuki willing this port

        missions[i].ports.push_back(aux_port);
      }

      //printf("______________________ Task %i has %i ports.\n", missions[i].id_task, missions[i].ports.size());
    }
	
  got_ports = true;

}

void willingPortCallback(const fuzzymar_multi_robot::taskObjective::ConstPtr& willing)
{

  for(int i = 0 ; i < missions.size() ; i++) //clean previous ports
  {
    for(int j = 0 ; j < missions[i].ports.size() ; j++)
    {
      if(missions[i].ports[j].id_kobuki == willing->id_kobuki)
      {
        missions[i].ports[j].id_kobuki = 0;
      }
    }
  }

  for(int i = 0 ; i < missions.size() ; i++)
  {
    //printf("willing loop 1 count: %i\n", i);
    if(missions[i].id_task == willing->id_task)
    {
      //printf("willing if 1\n");
      for(int j = 0 ; j < missions[i].ports.size() ; j++)
      {
        //printf("port -> %i\n", willing->port_priority[j]);
        if(missions[i].ports[(willing->port_priority[j])-1].id_kobuki == 0)
        {
          missions[i].ports[(willing->port_priority[j])-1].id_kobuki = willing->id_kobuki;
          //printf("Kobuki_%i set the port %i of the task %i as objective.\n", willing->id_kobuki, willing->port_priority[j], willing->id_task);
          //printf("Kobuki ID in missions: %i\n", missions[i].ports[(willing->port_priority[j])-1].id_kobuki);

          //if(willing->id_kobuki == token){publish_token = true;}

          break;
        }
      }
      break;
    }
  }
}

void tokenUpdateCallback(const std_msgs::Empty::ConstPtr& update)
{
  publish_token = true;
}


/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "monitor_ports");

  ros::NodeHandle n;

	// Subscribers
  ros::Subscriber task_sub = n.subscribe("/treball", 1000, taskUpdateCallback);
  ros::Subscriber ports_sub = n.subscribe("/ports", 1000, portsCallback);
  ros::Subscriber willing_sub = n.subscribe("/decidedTask", 1000, willingPortCallback);

  ros::Subscriber token_update_sub = n.subscribe("/token_update", 1000, tokenUpdateCallback);

	// Publishers
  ros::Publisher mission_pub = n.advertise<fuzzymar_multi_robot::task_w_portsArray>(state_pub_topic, 10);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1);
  ros::Publisher end_mission_pub = n.advertise<std_msgs::Empty>( "/mission_accomplished", 10);
  ros::Publisher time_pub = n.advertise<fuzzymar_multi_robot::time_task>( "/time_information", 10);

  ros::Publisher token_pub = n.advertise<fuzzymar_multi_robot::token>( "/token", 10);

  // Variable as buffer to read de keyboard 
  char cmd[1];

  // Getting path to .txt file (mission)
  std::string mission_path_file;
  n.getParam("/monitor_ports/mission_path_file", mission_path_file);   // "/node_name(indicated inside this code)/param_name" 

  // Write mission info
  std::ofstream outfile;
  std::string folder_directory = "/home/tonitauler/ROS/kobuki_ws/src/fuzzymar_multi_robot/data/";
  std::string file_name = folder_directory + current_date() + ".csv";

  ros::Rate loop_rate(LOOP_RATE);

  token_msg.token_kobuki = 0;

  while (ros::ok())
  {

    if(first_loop){ // first loop

        //printf("Task file executed: %s\n",mission_path_file.c_str());

        if(got_ports)
        {
          //outfile.open(file_name, std::fstream::in | std::fstream::out | std::fstream::app);
          //outfile << "%Task" << ", " << "End time" << std::endl;

          printf("\nPress ENTER to begin. \n");

          num_tasks = missions.size();
          
          std::cin.getline(cmd, 1);

          mission_time = ros::Time::now();

          printf("Init Time: sec: %i , nsec: %i\n", mission_time.sec, mission_time.nsec);
          printf("Init Time: sec: %f\n", mission_time.toSec());

          publishTimeTask(time_pub, 0, mission_time.sec, mission_time.nsec, 0, 0, 0);

          first_loop = false;

        } else {

          getMission(& missions, mission_path_file);    // get the info of the current mission

          ros::Duration(0.5).sleep(); // wait for the task_ports execution
          // VERY FIRST publish mission
          publishMission(&mission_pub);

          printf("Mission has been published for the first time. \n");

          ros::Duration(0.5).sleep(); // wait for the task_ports execution

        }
        
        mission_finalized = false;  
        
    } 
    else  // not the first loop
    {
      if(!mission_finalized) // vector missions is updated
      {
        missionsUpdate(time_pub);
        aux_time = ros::Time::now();
      }

      if(missions_update) 
      {

        publishMission(&mission_pub); // publish the vector missions when is updated

        if(publish_token){
          publishToken(&token_pub); // SEQUENCIAL
          publish_token = false;
        }

        cleanprintGreen();

        printInterface();

        if(missions.size() == 0) // the mission is accomplished
        {
          double end_time = time2Float(ros::Time::now()) - time2Float(mission_time);
          printf("Mission accomplished in %6.3f secs\n", end_time);
          std_msgs::Empty end_msg;
          end_mission_pub.publish(end_msg);
          
          mission_finalized = true;
          //first_loop = true;
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
    fuzzymar_multi_robot::port port_aux;
    fuzzymar_multi_robot::task_w_ports mission_line;
		fuzzymar_multi_robot::task_w_portsArray mission_buff;
    
		for(int i = 0 ; i < missions.size() ; i++)
		{

      mission_line.ports.clear();

			mission_line.id_task = missions[i].id_task;
      mission_line.number_ports = missions[i].number_ports;
			mission_line.deadline_task = missions[i].deadline;
			mission_line.weight_task = missions[i].weight;
			mission_line.x = missions[i].x;
			mission_line.y = missions[i].y;
			mission_line.doing_task = missions[i].doing_task;
      mission_line.utility = missions[i].utility;

      for(int j = 0 ; j < missions[i].ports.size() ; j++)
      {
        port_aux.id_port = missions[i].ports[j].id_port;
        port_aux.x = missions[i].ports[j].x;
        port_aux.y = missions[i].ports[j].y;
        port_aux.yaw = missions[i].ports[j].yaw;
        port_aux.id_kobuki = missions[i].ports[j].id_kobuki; // means any kobuki willing this port

        mission_line.ports.push_back(port_aux);
      }

			mission_buff.missions.push_back(mission_line);
		}
    
		mission_buff.vector_size = missions.size();
   
    mission_pub->publish(mission_buff);
}

void publishToken(ros::Publisher* token_pub)
{

  if(aux_token_cnt == 5)
  {
    token = 0;
    token_counter++;
    if(token_counter == 6)
    {
      token_counter = 1;
    }
    aux_token_cnt = 0;
  }

  token = aux_token_cnt + token_counter;

  if(token > 5){
    token = token % 5;
  }

  token_msg.token_kobuki = token;
  
  //printf("Token: %i\n", token_msg.token_kobuki);
  token_pub->publish(token_msg);

  aux_token_cnt++;
  
}

void publishMarkers(ros::Publisher* vis_pub)
{
    visualization_msgs::MarkerArray marker;
    visualization_msgs::Marker aux_marker;
    aux_marker.action = visualization_msgs::Marker::DELETEALL;

    int id_task = 0;
    int id_port = 0;
    int id_text = 0;
    float red, green, blue;

    for(int i = 0 ; i < missions.size() ; i++)
    {

        float angle = ((360 / num_tasks) * missions[i].id_task) - (360 / num_tasks)/2;

        if (angle<60){
          red = 255; green = round(angle*4.25-0.01); blue = 0;
        } else if (angle<120){
          red = round((120-angle)*4.25-0.01); green = 255; blue = 0;
        } else if (angle<180){
          red = 0, green = 255; blue = round((angle-120)*4.25-0.01);
        } else if (angle<240){
          red = 0, green = round((240-angle)*4.25-0.01); blue = 255;
        } else if (angle<300){
          red = round((angle-240)*4.25-0.01), green = 0; blue = 255;
        } else {
          red = 255, green = 0; blue = round((360-angle)*4.25-0.01);} 
        
        red = red / 255;
        green = green / 255;
        blue = blue / 255;

        std::string text = std::to_string(missions[i].id_task);

        aux_marker.header.frame_id = "map";
        aux_marker.header.stamp = ros::Time::now();
        aux_marker.lifetime = ros::Duration(0.5);
        aux_marker.ns = "tasks";
        aux_marker.id = id_task;
        id_task++;
        aux_marker.type = visualization_msgs::Marker::CYLINDER;
        aux_marker.action = visualization_msgs::Marker::ADD;
        aux_marker.pose.position.x = missions[i].x;
        aux_marker.pose.position.y = missions[i].y;
        aux_marker.pose.position.z = 0.05;
        aux_marker.pose.orientation.x = 0.0;
        aux_marker.pose.orientation.y = 0.0;
        aux_marker.pose.orientation.z = 0.0;
        aux_marker.pose.orientation.w = 1.0;
        aux_marker.scale.x = 0.2;
        aux_marker.scale.y = 0.2;
        aux_marker.scale.z = 0.10;
        aux_marker.color.a = 1.0; // Don't forget to set the alpha!
        aux_marker.color.r = red;
        aux_marker.color.g = green;
        aux_marker.color.b = blue;

        marker.markers.push_back(aux_marker);

        aux_marker.header.frame_id = "map";
        aux_marker.header.stamp = ros::Time::now();
        aux_marker.lifetime = ros::Duration(0.5);
        aux_marker.ns = "tasks_name";
        aux_marker.id = id_text;
        id_text++;
        aux_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        aux_marker.action = visualization_msgs::Marker::ADD;
        aux_marker.text = text;
        aux_marker.pose.position.x = missions[i].x;
        aux_marker.pose.position.y = missions[i].y;
        aux_marker.pose.position.z = 0.1;
        aux_marker.pose.orientation.x = 0.0;
        aux_marker.pose.orientation.y = 0.0;
        aux_marker.pose.orientation.z = 0.0;
        aux_marker.pose.orientation.w = 1.0;
        aux_marker.scale.x = 0.1;
        aux_marker.scale.y = 0.1;
        aux_marker.scale.z = 0.2;
        aux_marker.color.a = 1.0; // Don't forget to set the alpha!
        aux_marker.color.r = 0.0;
        aux_marker.color.g = 0.0;
        aux_marker.color.b = 0.0;

        marker.markers.push_back(aux_marker);

        for(int j = 0 ; j < missions[i].ports.size() ; j++)
        {
        aux_marker.header.frame_id = "map";
        aux_marker.header.stamp = ros::Time::now();
        aux_marker.lifetime = ros::Duration(0.5);
        aux_marker.ns = "ports";
        aux_marker.id = id_port;
        id_port++;
        aux_marker.type = visualization_msgs::Marker::CYLINDER;
        aux_marker.action = visualization_msgs::Marker::ADD;
        aux_marker.pose.position.x = missions[i].ports[j].x;
        aux_marker.pose.position.y = missions[i].ports[j].y;
        aux_marker.pose.position.z = 0.07;
        aux_marker.pose.orientation.x = 0.0;
        aux_marker.pose.orientation.y = 0.0;
        aux_marker.pose.orientation.z = 0.0;
        aux_marker.pose.orientation.w = 1.0;
        aux_marker.scale.x = 0.1;
        aux_marker.scale.y = 0.1;
        aux_marker.scale.z = 0.10;
        aux_marker.color.a = 1.0; // Don't forget to set the alpha!
        aux_marker.color.r = red;
        aux_marker.color.g = green;
        aux_marker.color.b = blue;

        marker.markers.push_back(aux_marker);
        }

    }

    vis_pub->publish(marker);

}

void publishTimeTask(ros::Publisher& time_pub, int id_task, int sec, int nsec, float deadline, float u_max, int total_ports)
{

    fuzzymar_multi_robot::time_task time_buff;

    time_buff.id_task = id_task;
    time_buff.sec = sec;
    time_buff.nsec = nsec;
    time_buff.deadline = deadline;
    time_buff.utility_max = u_max;
    time_buff.total_ports = total_ports;
   
    time_pub.publish(time_buff);
}

void missionsUpdate(ros::Publisher& time_pub)
{
  //printf("\n************\n Actual: %f\nNOW: %f, AUX_TIME: %f, Resta: %f\n*************\n", missions[i].deadline, time2Float(ros::Time::now()), time2Float(aux_time), (time2Float(ros::Time::now()) - time2Float(aux_time)));
  
  if(aux_time_initialized)
  {
    for(uint16_t i = 0 ; i < missions.size() ; i++)
    {
      // OLD VERSION missions[i].weight -= (float)missions[i].doing_task * (1.0/(float)LOOP_RATE); // the weight is decreased due to robot/s work
      
      /*if(missions[i].deadline > 0.0)
      {
        missions[i].deadline -= time2Float(ros::Time::now()) - time2Float(aux_time);
      } else {
        missions[i].deadline = 0.0;
      }*/
      
      missions[i].weight -= (float)missions[i].doing_task * (time2Float(ros::Time::now()) - time2Float(aux_time));

      ROS_DEBUG("Task %i has weight: %f", missions[i].id_task, missions[i].weight);
      if(missions[i].weight <= (0.5/(float)LOOP_RATE)) // (0.5/LOOP_RATE) to avoid infim weights
      {
        // WRITE IN TXT
        //*outfile << std::to_string(missions[i].id_task) << ", " << std::to_string(ros::Time::now().toSec() - mission_time.toSec()) << std::endl;
        ros::Time time_aux = ros::Time::now();
        publishTimeTask(time_pub, missions[i].id_task, time_aux.sec, time_aux.nsec, missions[i].deadline, missions[i].utility, missions[i].ports.size());

        missions.erase(missions.begin()+i);// eliminate this task of vector missions
        i--; //because the vector missions decrease 1
      }
    }

    missions_update = true;
  } else {
    aux_time_initialized = true;
  }
  

}

void cleanprintGreen()
{
  bool changed[5] = {false, false, false, false, false};

  for(int i = 0 ; i < missions.size() ; i++) 
  {
    for(int j = 0 ; j < missions[i].ports.size() ; j++)
    {
      if(missions[i].ports[j].id_kobuki != 0)
      {
        if(missions[i].id_task != task_working[missions[i].ports[j].id_kobuki - 1])
        {
          changed[(missions[i].ports[j].id_kobuki) - 1] = true;
        }
      }
    }
  }

  for(int i = 0 ; i < 5 ; i++)
  {
    if(changed[i])
    {
      paintGreen[i] = false;
    }
  }

}

void printInterface()
{

  printf("\n \n \n \n \n \n \n");
  printf("+----+--------+--------+-------+-------+-----+-------+------------------+\n");
  printf("| ID |   DL   | WEIGHT |   X   |   Y   | Rob |   U   |  Assigned Ports  |\n");
  printf("+----+--------+--------+-------+-------+-----+-------+------------------+\n");
  for(int i = 0 ; i < missions.size() ; i++)
  {
    if(missions[i].deadline < ((ros::Time::now().toSec() - mission_time.toSec())))
    {

      printf("| %2i |" ANSI_COLOR_RED " %6.2f " ANSI_COLOR_RESET "| %5.2f  | %5.2f | %5.2f |  %1i  | %5.2f |", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task, missions[i].utility);
    
    } else if(missions[i].deadline < (missions[i].weight + (ros::Time::now().toSec() - mission_time.toSec())))
    {

      printf("| %2i |" ANSI_COLOR_YELLOW " %6.2f " ANSI_COLOR_RESET "| %5.2f  | %5.2f | %5.2f |  %1i  | %5.2f |", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task, missions[i].utility);
    
    } else {

      printf("| %2i | %6.2f | %5.2f  | %5.2f | %5.2f |  %1i  | %5.2f |", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task, missions[i].utility);

    }
    
    for(int j = 0 ; j < 6 ; j++) // 6 is max num of ports assignable (due to the robots size)
    {
      if(j < missions[i].ports.size())
      {
        if(paintGreen[(missions[i].ports[j].id_kobuki) - 1] && missions[i].ports[j].id_kobuki != 0)
        {
          printf(ANSI_COLOR_GREEN " %i " ANSI_COLOR_RESET, missions[i].ports[j].id_kobuki);
        } else {
          printf(" %i ", missions[i].ports[j].id_kobuki);
        }
        
      } else {
        printf("   ");
      }
      
    }
    
    printf("|\n");
  }
  printf("+----+--------+--------+-------+-------+-----+-------+------------------+\n");
  printf("\nCurrent time: %6.3f secs\n", (double)(time2Float(ros::Time::now()) - time2Float(mission_time)));

}

std::string current_date()
{
    time_t now = time(0);

    tm *ltm = localtime(&now);

    std::string date;

    date = std::to_string(1900 + ltm->tm_year) + "-" + std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_hour) + ":" + std::to_string(ltm->tm_min);
    //ltm->tm_sec //if we want the seconds
    return date;
}