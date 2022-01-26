/**
 * @Author: Toni Tauler
 * @File: robot_operation.cpp
 * @Description: TODO
 * @Date: January 2022
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fuzzymar_multi_robot/mission.h>
#include <fuzzymar_multi_robot/missionArray.h>
#include <fuzzymar_multi_robot/action_task.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>

#define DEBUGGING 1

// STRUCTS

struct Mission {
    uint8_t id_task;
    float deadline;
		float weight;
		float x;
		float y;
		uint8_t doing_task;
    bool enable;
};

// GLOBAL VARIABLES
int kobuki_id;
std::string state_sub_topic = "/mission_state";
std::string pose_sub_topic = "amcl_pose";
std::string goal_status_sub_topic = "move_base/status";

std::vector<Mission> missions; // vector where the tasks are saved {id_task, deadline, weight, x, y, doing_task, enable}
std::vector<std::vector<float>> probabilities; // vector where probabilities are saved
float threshold = 0.0; // threshold
std::pair<float, float> current_position; // current position of the robot
std::vector<float> current_goal; // current task declared as objective {task_id, x, y, yaw}

uint8_t status = 255; // objective status of the robot -> 1 == in the way to goal // 3 == goal reached // 255 == no status 

ros::Time init_task; // time at which current task begins

// FLAGS
bool new_mission_state = false;
bool way_to_task = false; // indicates if the robot is on the way of a task
bool send_action_task = false; // allow to send action_task msg

// FUNCTIONS DECLARATION
float distance(float x1, float y1, float x2, float y2); // calculate the distance between 2 points in 2D
float getThreshold(); // get the threshold == max_dist(between tasks)/2
float getStimulous(int task); // get the current robot stimulous for a given task
void setProbabilities(); // build the probabilities vector {id_task, stimulous, probability}
std::vector<float> setTaskObjective(); // return the global goal {task_id, x, y, yaw} to publish to /kobuki_x/move_base_simple/goal

void publishGoal(ros::Publisher* goal_pub, std::vector<float> goal);
void publishTask(ros::Publisher* task_pub);

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void missionStateCallback(const fuzzymar_multi_robot::missionArray::ConstPtr& mission)
{

  /*float id_task, deadline, weight, x, y, doing_task, enable; // In a 3D navigation have to add z pose
  id_task = (float)mission->id_task;
  deadline = mission->deadline_task;
  weight = mission->weight_task;
  x = mission->location_task.x;
  y = mission->location_task.y;
  doing_task = 0.0; // 0 robots doing the task
  enable = 1.0; // is enable

  //z = mission->location_task.z;  // 3D navigation

  // building mission vector
  std::vector<float> aux_mission = {id_task, deadline, weight, x, y, doing_task, enable}; // In a 3D navigation have to add z pose

  missions.push_back(aux_mission);*/
  missions.clear();

  Mission mission_data;
  
  for(int i = 0 ; i < mission->vector_size ; i++)
  {
    mission_data.id_task = mission->missions[i].id_task;
    mission_data.deadline = mission->missions[i].deadline_task;
    mission_data.weight = mission->missions[i].weight_task;
    mission_data.x = mission->missions[i].x;
    mission_data.y = mission->missions[i].y;
    mission_data.doing_task = mission->missions[i].doing_task;
    if(mission->missions[i].doing_task <= 4){mission_data.enable = true;}

    missions.push_back(mission_data);
  }

  new_mission_state = true;
  
}

void actualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  
  current_position.first = pose->pose.pose.position.x;
  current_position.second = pose->pose.pose.position.y;
  
}

void statusGoalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& stat)
{
  
  if (!stat->status_list.empty())  // we have to ensure status_list array is not empty else we have error and node die due to memory allocation error
  {
    status = stat -> status_list[stat->status_list.size()-1].status; // this structure is used to always have the status updated (due to move_base way to work)
  } else {
    status = 255;
  }

  if(status == 3 && way_to_task)
  {
    init_task = stat -> header.stamp;
    send_action_task = true;
    way_to_task = false;
    ROS_DEBUG("Task: %i initialized by Kobuki_%i at sec: %i  nsec: %i", (int)current_goal[0], kobuki_id, init_task.sec, init_task.nsec);
  } else if(status == 1){
    way_to_task = true;
  }

}
/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_operation");

  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber mission_sub = n.subscribe(state_sub_topic, 1000, missionStateCallback);
  ros::Subscriber pose_sub = n.subscribe(pose_sub_topic, 1000, actualPoseCallback);
  ros::Subscriber goal_status_sub = n.subscribe(goal_status_sub_topic, 1000, statusGoalCallback);

  // Publishers
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  ros::Publisher task_pub = n.advertise<fuzzymar_multi_robot::action_task>("/treball", 10);

  // Params
  n.getParam("robot_operation/kobuki_id", kobuki_id);   // "/node_name(indicated inside this code)/param_name" --> the first '/' depends if groupns is used or not

  ros::Rate loop_rate(10);

  bool first_loop = true;

  while (ros::ok())
  {
    
    uint8_t aux_current_goal; 

    if(new_mission_state) // first time missions is updated
    {

      threshold = getThreshold(); // the threshold is gotten

      setProbabilities(); // the probabilities for each task are obtained

      current_goal = setTaskObjective(); // set de current objective (task) for the robot

      if(first_loop || (uint8_t)current_goal[0] != aux_current_goal)  // only publish new goal if is the first task or a new task assignment
      {
        publishGoal(&goal_pub, current_goal); // the current_goal is published on /kobuki_x/move_base_simple/goal topic
      }

      // ***************DUBUGGING*****************

      if(DEBUGGING)
      {
        // KOBUKI
        ROS_INFO("kobuki_%i initialized", kobuki_id);

        // Missions vector
        for(int i = 0 ; i < missions.size() ; i++)
        {
          ROS_INFO("ID: %i , DD: %f , WEIGTH: %f , X: %f , Y: %f , Robots: %i , ENABLE(robots < ports):%i ", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task, (int)missions[i].enable);
        }

        // Threshold
        ROS_INFO("Threshold: %f", threshold);

        // Probabilities vector
        for(int i = 0 ; i < probabilities.size() ; i++)
        {
          ROS_INFO("ID: %i , STIMULOUS: %f , PROBABILITY: %f", (int)probabilities[i][0], probabilities[i][1], probabilities[i][2]);
        }

        // Current_goal vector
        ROS_INFO("Current task -> ID: %i , X: %f , Y: %f , YAW: %f", (int)current_goal[0], current_goal[1],current_goal[2], current_goal[3]);

        // Current position
        ROS_INFO("Current position -> X: %f, Y: %f", current_position.first, current_position.second);
      }

      aux_current_goal = (uint8_t)current_goal[0];
      new_mission_state = false;

    }

    if(send_action_task) // send when robot arrives to the current task (goal reached)
    {
      // publish
      publishTask(&task_pub); // publish {id_kobuki, id_task, sec, nsec}
      send_action_task = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
    first_loop = false;

  } 

  return 0;
}

/********************************************************************************************
*************************************** FUNCTIONS *******************************************
********************************************************************************************/

float distance(float x1, float y1, float x2, float y2)
{
  float dist = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  return dist;
}

float getThreshold()
{
  float aux_thres = 0.0;
  float dist = 0.0;
  for(int i = 0 ; i < missions.size() - 1 ; i++)
  {
    for (int j = 1 ; j < missions.size() ; j++)
    {

      dist = distance(missions[i].x, missions[i].y, missions[j].x, missions[j].y);

      if(aux_thres < dist)
      {
        aux_thres = dist;
      }
    }
  }

  float threshold = aux_thres/2;
  //ROS_INFO("Threshold: %f", threshold);
  return threshold;
}

float getStimulous(int task)
{
  float stim = 1 / distance(current_position.first, current_position.second, missions[task-1].x, missions[task-1].y); // TODO change 0.0's to /kobuki_x/amcl_pose x and y

  return stim;
}

void setProbabilities()
{
  probabilities.clear();

  float id, stimulous, probability;

  for(int i = 1 ; i <= missions.size() ; i++)
  {
    id = (float)i;
    stimulous = getStimulous(i); // i+1 because we give the id task not the vector position
    probability = (stimulous*stimulous) / ((stimulous*stimulous) + (threshold*threshold));

    std::vector<float> aux_prob = {id , stimulous, probability};

    probabilities.push_back(aux_prob);
  }

}

std::vector<float> setTaskObjective()
{
  std::vector<float> aux_objective;
  float id, x, y, yaw;
  float prob = 0.0;

  for(int i = 0 ; i < probabilities.size() ; i ++)
  {
    if(probabilities[i][2] > prob)
    {
      prob = probabilities[i][2];
      id = missions[i].id_task;
      x = missions[i].x;
      y = missions[i].y;
      yaw = 0.0;  // TODO es ports per a cada missio
    }
  }

  aux_objective = {id, x, y, yaw};
  return aux_objective;

}

void publishGoal(ros::Publisher* goal_pub, std::vector<float> goal)
{
  geometry_msgs::PoseStamped goal_msg;

  goal_msg.header.frame_id = "map";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.pose.position.x = goal[1];
  goal_msg.pose.position.y = goal[2];
  goal_msg.pose.position.z = 0.0;

  goal_msg.pose.orientation.x = 0.0;
  goal_msg.pose.orientation.y = 0.0;    // TODO fer l'orientació com toca 
  goal_msg.pose.orientation.z = 0.0;
  goal_msg.pose.orientation.w = 1.0;  

  goal_pub->publish(goal_msg);
}

void publishTask(ros::Publisher* task_pub)
{
  fuzzymar_multi_robot::action_task aux_task;

  aux_task.id_kobuki = kobuki_id;
  aux_task.id_task = (uint8_t)current_goal[0];
  aux_task.sec = init_task.sec;
  aux_task.nsec = init_task.nsec;

  ROS_INFO("kobuki_%i starts task %i at sec:%i nsec: %i", aux_task.id_kobuki, aux_task.id_task, aux_task.sec, aux_task.nsec);

  task_pub->publish(aux_task);
}