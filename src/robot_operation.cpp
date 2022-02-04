/**
 * @Author: Toni Tauler
 * @File: robot_operation.cpp
 * @Description: TODO
 * @Date: January 2022
 */

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalID.h"
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
#include <random>

#define DEBUGGING 0
#define CLEAR_COSTMAP 0

// STRUCTS

struct Mission {
  uint8_t id_task;
  float deadline;
  float weight;
  float x;
  float y;
  uint8_t doing_task;
  bool enable;
  int utility;
};

struct Probability {
  uint8_t id_task;
  float d_stimulous;
  float u_stimulous;
  float stimulous;
  float probability;
  float norm_probability;
  float accumulative;
};

struct Current_goal {
  uint8_t id_task;
  float x;
  float y;
  float yaw;
};

// ENUMS

enum Kobuki_State {
  Computing, Start_task, Navigating, Working, Leave_task, Stuck
};

// GLOBAL VARIABLES
int kobuki_id;
int alpha_utility;
std::string state_sub_topic = "/mission_state";
std::string pose_sub_topic = "amcl_pose";
std::string goal_status_sub_topic = "move_base/status";

std::vector<Mission> missions; // vector where the tasks are saved {id_task, deadline, weight, x, y, doing_task, enable}
std::vector<Probability> probabilities; // vector where probabilities are saved
std::pair<float, float> current_position; // current position of the robot
Current_goal current_goal; // current task declared as objective {task_id, x, y, yaw}

uint8_t status = 255; // objective status of the robot -> 1 == in the way to goal // 3 == goal reached // 255 == no status 

ros::Time init_task; // time at which current task begins

Kobuki_State kobuki_state = Computing;

uint8_t last_task_id;



// FLAGS
bool new_mission_state = false;
bool way_to_task = false; // indicates if the robot is on the way of a task
bool send_action_task = false; // allow to send action_task msg
bool end_mission = false;
bool force_clear_costmap = false; // indicates if have to clear the costmaps due to an error

// FUNCTIONS DECLARATION
float distance(float x1, float y1, float x2, float y2); // calculate the distance between 2 points in 2D
float getThreshold(); // get the threshold == max_dist(between tasks)/2
float getStimulous(int task); // get the current robot stimulous for a given task
float getUtilityStimulous(int task); // get the utility stimulous
void setProbabilities(float thres); // build the probabilities vector {id_task, stimulous, probability}
Current_goal setTaskObjective(float rand); // return the global goal {task_id, x, y, yaw} to publish to /kobuki_x/move_base_simple/goal

void publishGoal(ros::Publisher* goal_pub, Current_goal goal);
void publishTask(ros::Publisher* task_pub);

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void missionStateCallback(const fuzzymar_multi_robot::missionArray::ConstPtr& mission)
{
  //ROS_INFO("missionStateCallback");
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
    mission_data.utility = mission->missions[i].utility;

    missions.push_back(mission_data);
  }

  new_mission_state = true;
  
}

void actualPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{
  //ROS_INFO("actualPoseCallback");
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

}

void endMissionCallback(const std_msgs::Empty::ConstPtr& end_msg)
{
  //ROS_INFO("endMissionCallback");
  end_mission = true;
}
/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_operation");

  ros::NodeHandle n;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0,1.0);

  // Subscribers
  ros::Subscriber mission_sub = n.subscribe(state_sub_topic, 1000, missionStateCallback);
  ros::Subscriber pose_sub = n.subscribe(pose_sub_topic, 1000, actualPoseCallback);
  ros::Subscriber goal_status_sub = n.subscribe(goal_status_sub_topic, 1000, statusGoalCallback);
  ros::Subscriber end_mission_sub = n.subscribe("/mission_accomplished", 1000, endMissionCallback);

  // Publishers
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  ros::Publisher task_pub = n.advertise<fuzzymar_multi_robot::action_task>("/treball", 10);
  ros::Publisher cancel_navigation_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);


  // Params
  n.getParam("robot_operation/kobuki_id", kobuki_id);   // "/node_name(indicated inside this code)/param_name" --> the first '/' depends if groupns is used or not
  n.getParam("robot_operation/alpha_utility", alpha_utility);

  ros::Rate loop_rate(10);

  ros::service::waitForService("move_base/clear_costmaps");
  ros::ServiceClient clearClient = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  std_srvs::Empty srv;

  int loop_counter = 0;

  bool first_loop = true; 

  uint8_t aux_current_goal; 
  uint8_t aux_task_number; // how much tasks are in missions vector
  float threshold;
  float random;

  ROS_INFO("Kobuki_%i has an alpha_utility of %i", kobuki_id, alpha_utility);

  while (ros::ok())
  {

    // KOBUKI_STATE UPDATE
    if(status == 3 && kobuki_state == Navigating) { // TODO es possible que haguem d'afegir status == 3 && kobuki_state == Computing per si sa tasca està a la posició actual del robot (no s'ha de moure per anar a la tasca)
      kobuki_state = Start_task;
      init_task = ros::Time::now();
      //ROS_INFO("Start_task");
    } else if(status == 1 && kobuki_state == Working) {
      kobuki_state = Leave_task;
      //ROS_INFO("Leave_task");
    } else if(status == 1) {
      kobuki_state = Navigating;
      //ROS_INFO("Navigating");
    } else if(status == 3) {
      kobuki_state = Working;
      //ROS_INFO("Working");
    } else if(status == 4){
      kobuki_state = Stuck;
      //ROS_INFO("Stuck");
    } else {
      kobuki_state = Computing;
      //ROS_INFO("Computing");
    }

    if(new_mission_state) // when missions is updated
    {

      if(first_loop){
        threshold = getThreshold(); // the threshold is gotten
      }

      if(end_mission) // mission is accomplished
      {
        ROS_INFO("Mission accomplished");
        actionlib_msgs::GoalID cancel_msg;
        cancel_navigation_pub.publish(cancel_msg);
        
      } else {

        if(first_loop || aux_task_number != missions.size()){  // NOW ONLY RECALCULATE A POSSIBLE NEW GOAL WHEN A TASK IS END ¿¿¿¿¿¿ HOW TO DO ??????

          setProbabilities(threshold); // the probabilities for each task are obtained

          random = dis(gen);

          ROS_INFO("RANDOM: %f", random);

          current_goal = setTaskObjective(random); // set de current objective (task) for the robot

        }


        if((first_loop || current_goal.id_task != aux_current_goal))  // only publish new goal if is the first task or a new task assignment
        {
          publishGoal(&goal_pub, current_goal); // the current_goal is published on /kobuki_x/move_base_simple/goal topic
        }
      
      }

      // ***************DUBUGGING*****************

      if(DEBUGGING)
      {
        // KOBUKI
        ROS_DEBUG("kobuki_%i initialized", kobuki_id);

        // Missions vector
        for(int i = 0 ; i < missions.size() ; i++)
        {
          ROS_DEBUG("ID: %i , DD: %f , WEIGTH: %f , X: %f , Y: %f , Robots: %i , ENABLE(robots < ports):%i ", missions[i].id_task, missions[i].deadline, missions[i].weight, missions[i].x, missions[i].y, missions[i].doing_task, (int)missions[i].enable);
        }

        // Threshold
        ROS_DEBUG("Threshold: %f", threshold);

        // Probabilities vector
        for(int i = 0 ; i < probabilities.size() ; i++)
        {
          ROS_INFO("ID: %i , STIMULOUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f", probabilities[i].id_task,  probabilities[i].stimulous,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
        }

        // Current_goal vector
        ROS_DEBUG("Current task -> ID: %i , X: %f , Y: %f , YAW: %f", current_goal.id_task, current_goal.x,current_goal.y, current_goal.yaw);

        // Current position
        ROS_DEBUG("Current position -> X: %f, Y: %f", current_position.first, current_position.second);
      }

      aux_current_goal = current_goal.id_task;
      aux_task_number = (uint8_t)missions.size();
      new_mission_state = false;

      first_loop = false;

    }

    if(kobuki_state == Start_task || kobuki_state == Leave_task) // send when robot arrives to the current task (goal reached)
    {
      // publish
      publishTask(&task_pub); // publish {id_kobuki, id_task, sec, nsec}
      /*if(kobuki_state == Start_task){ROS_INFO("Publish Start_task");}
      else{ROS_INFO("Publish Leave_task");} */
      
    }

    if(loop_counter == 150 || kobuki_state == Stuck){ // if the robot abort the mission or each 15 secs, clear the costmaps and publish again the objective
      
      clearClient.call(srv);

      if(kobuki_state == Stuck){publishGoal(&goal_pub, current_goal);} // republish the objective
      
      loop_counter = 0;
    }

    ros::spinOnce();
    //ROS_INFO("________________________spinOnce %i", loop_counter);
    loop_rate.sleep();

    loop_counter++;

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

float getUmax()
{
  float u_max = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    if(missions[i].utility > u_max)
    {
      u_max = missions[i].utility;
    }
  }
  
  return u_max;
}

float getStimulous(int task)
{
  float stim = 1 / distance(current_position.first, current_position.second, missions[task-1].x, missions[task-1].y); // TODO change 0.0's to /kobuki_x/amcl_pose x and y

  return stim;
}

float getUtilityStimulous(int task)
{
  float stim = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    if(missions[i].utility - missions[task-1].utility > stim)
    {
      stim = missions[i].utility - missions[task-1].utility;
    }
  }

  return stim;
}

void setProbabilities(float thres)
{
  probabilities.clear();

  uint8_t id;
  float stim, prob, norm_prob, accumul;
  float distance_stim = 0.0;
  float utility_stim = 0.0;
  float sum_prob, sum_norm;
  sum_prob = 0.0;
  sum_norm = 0.0;

  for(int i = 1 ; i <= missions.size() ; i++)
  {
    id = missions[i-1].id_task;
    // OLD SIMPLY VERSION --> stim = getStimulous(i); // i+1 because we give the id task not the vector position
    //                        prob = (stim*stim) / ((stim*stim) + (thres*thres));
    distance_stim = 1/getStimulous(i);
    utility_stim = getUtilityStimulous(i);
    stim = (alpha_utility * ((thres*2.0)/(getUmax()))) * utility_stim + distance_stim; // thres*2.0 == d_max
    prob = (thres*thres) / ((stim*stim) + (thres*thres));
    norm_prob = 0.0;
    accumul = 0.0;
    sum_prob += prob;

    Probability aux_prob = {id, distance_stim, utility_stim, stim, prob, norm_prob, accumul};

    probabilities.push_back(aux_prob);
  }
  
  for(int j = 0 ; j < probabilities.size() ; j++) { // fill the normalized probability and the accumulative normalized probability
    probabilities[j].norm_probability = probabilities[j].probability / sum_prob;
    sum_norm += probabilities[j].norm_probability;
    probabilities[j].accumulative = sum_norm;
  }

}

Current_goal setTaskObjective(float rand)
{
  Current_goal aux_objective;
  uint8_t id;
  float x, y, yaw;
  float prob = 0.0;
  
  for(int i = 0 ; i < probabilities.size() ; i ++)  // set a task for the robot from available ones
  {
    if(rand <= probabilities[i].accumulative){
      id = missions[i].id_task;
      x = missions[i].x;
      y = missions[i].y;
      yaw = 0.0;  // TODO es ports per a cada missio
      //ROS_INFO("Kobuki_%i set task: %i", kobuki_id, id);
      break; // out of loop
    }
  }

  /*for(int i = 0 ; i < probabilities.size() ; i++)
  {
    ROS_INFO("ID: %i , D_STIM:%f, U_STIM:%f, STIMULOUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f", probabilities[i].id_task, probabilities[i].d_stimulous, probabilities[i].u_stimulous, probabilities[i].stimulous,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
  }*/

  aux_objective = {id, x, y, yaw};
  return aux_objective;

}

void publishGoal(ros::Publisher* goal_pub, Current_goal goal)
{
  geometry_msgs::PoseStamped goal_msg;

  goal_msg.header.frame_id = "map";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.pose.position.x = goal.x;
  goal_msg.pose.position.y = goal.y;
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
  //aux_task.id_task = current_goal.id_task; // TODO millorar aquesta chapuzilla !!!!!!
  aux_task.sec = init_task.sec;
  aux_task.nsec = init_task.nsec;
  if(kobuki_state == Start_task)
  {
    aux_task.active = true;
    aux_task.id_task = current_goal.id_task;  // TODO millorar aquesta chapuzilla !!!!!!
    ROS_INFO("kobuki_%i starts task %i at sec:%i nsec: %i", aux_task.id_kobuki, aux_task.id_task, aux_task.sec, aux_task.nsec);
  } else {
    aux_task.active = false;
    aux_task.id_task = last_task_id;  // TODO millorar aquesta chapuzilla !!!!!!
    ROS_INFO("kobuki_%i leave task %i at sec:%i nsec: %i", aux_task.id_kobuki, aux_task.id_task, aux_task.sec, aux_task.nsec);
  }

  last_task_id = current_goal.id_task;

  //ROS_DEBUG("kobuki_%i starts task %i at sec:%i nsec: %i", aux_task.id_kobuki, aux_task.id_task, aux_task.sec, aux_task.nsec);

  task_pub->publish(aux_task);
}