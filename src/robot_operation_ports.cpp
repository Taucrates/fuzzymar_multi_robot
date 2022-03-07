/**
 * @Author: Toni Tauler
 * @File: robot_operation_ports.cpp
 * @Description: TODO
 * @Date: February 2022
 */

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fuzzymar_multi_robot/task_w_ports.h>
#include <fuzzymar_multi_robot/task_w_portsArray.h>
#include <fuzzymar_multi_robot/action_task_w_ports.h>
#include <fuzzymar_multi_robot/taskObjective.h>

#include <tf2/LinearMath/Quaternion.h> // from rpy to quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>
#include <random>

#define DEBUGGING 0
#define CLEAR_COSTMAP 0
#define TOTAL_ROBOTS 5

// STRUCTS

struct Task {
  uint8_t id_task;
  float deadline;
  float weight;
  float x;
  float y;
  uint8_t doing_task;
  bool enable;
  int utility;
  std::vector<fuzzymar_multi_robot::port> ports;
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

std::vector<Task> missions; // vector where the tasks are saved {id_task, deadline, weight, x, y, doing_task, enable}
std::vector<Probability> probabilities; // vector where probabilities are saved
std::pair<float, float> current_position; // current position of the robot
Current_goal current_goal; // current task declared as objective {task_id, x, y, yaw}

fuzzymar_multi_robot::port current_port;

uint8_t status = 255; // objective status of the robot -> 1 == in the way to goal // 3 == goal reached // 255 == no status 

ros::Time init_task; // time at which the robot begins to work in the current task 
ros::Time leave_task; // time at which the robot leaves the current task


Kobuki_State kobuki_state = Computing;

uint8_t last_task_id;

std::vector<uint8_t> ports_priority;

float min_port_stim = 0.8; // min_value at which portstimulous begin

int count_vel = 0;
bool may_unstuck = false;

// FLAGS
bool new_mission_state = false;
bool end_mission = false;
bool port_assigned = false;
bool calculated_task = false;

// FUNCTIONS DECLARATION
float distance(float x1, float y1, float x2, float y2); // calculate the distance between 2 points in 2D
bool disponiblePorts(); // disponible ports comprovation
int totalNumPorts(); // counts the total num of ports assigned, by task_ports node, in the mission
float getThreshold(); // get the threshold == max_dist(between tasks)/2
float getStimulous(int task); // get the current robot stimulous for a given task
float getUtilityStimulous(int task); // get the utility stimulous
void setProbabilities(float thres); // build the probabilities vector {id_task, stimulous, probability}

Current_goal setTaskObjective(float rand); // return the global goal {task_id, x, y, yaw} to publish to /kobuki_x/move_base_simple/goal

void setPortPriority(); //sets the priority of the ports for the current task (use distance as priority)

void publishGoal(ros::Publisher* goal_pub, Current_goal goal);
void publishPortGoal(ros::Publisher* goal_pub, fuzzymar_multi_robot::port goal);
void publishTask(ros::Publisher* task_pub);  
void publishTaskObjective(ros::Publisher* task_ports_pub);

void publishRotation(ros::Publisher* rotate_pub);

bool floatSimilarTo(float a, float b, float diff);

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void missionStateCallback(const fuzzymar_multi_robot::task_w_portsArray::ConstPtr& mission)
{
  //ROS_INFO("missionStateCallback");
  missions.clear();

  //printf("\nmissionsStateCallback\n");

  Task task_data;
  fuzzymar_multi_robot::port aux_port;
  
  for(int i = 0 ; i < mission->vector_size ; i++)
  {
    //printf("\ninit_firstLoop\n");
    task_data.ports.clear();

    task_data.id_task = mission->missions[i].id_task;
    task_data.deadline = mission->missions[i].deadline_task;
    task_data.weight = mission->missions[i].weight_task;
    task_data.x = mission->missions[i].x;
    task_data.y = mission->missions[i].y;
    task_data.doing_task = mission->missions[i].doing_task;
    if(mission->missions[i].doing_task <= 4){task_data.enable = true;}
    task_data.utility = mission->missions[i].utility;

    //printf("\n\n\nTask %i has the following ports:\n", task_data.id_task);

    for(int j = 0 ; j < mission->missions[i].ports.size() ; j++)
    {

      aux_port.id_port = mission->missions[i].ports[j].id_port;
      aux_port.x = mission->missions[i].ports[j].x;
      aux_port.y = mission->missions[i].ports[j].y;
      aux_port.yaw = mission->missions[i].ports[j].yaw;
      aux_port.id_kobuki = mission->missions[i].ports[j].id_kobuki;

      if(mission->missions[i].ports[j].id_kobuki == kobuki_id) //means that port is assigned to the robot
      {
        if((current_port.x != aux_port.x || current_port.y != aux_port.y)) // when a new port is assigned activate flag to publish new port/goal
        {

          current_port.id_port = aux_port.id_port;
          current_port.x = aux_port.x;
          current_port.y = aux_port.y;
          current_port.yaw = aux_port.yaw;

          port_assigned = true;
          printf("Kobuki_%i setted port %i of task %i\n", kobuki_id, aux_port.id_port, current_goal.id_task);
        }
        
      }

      task_data.ports.push_back(aux_port);

      //printf("      Port %i -> x: %5.2f   y: %5.2f\n", aux_port.id_port, aux_port.x, aux_port.y);
      
    }

    missions.push_back(task_data);
    //printf("______________________ Task %i has %i ports.\n", missions[i].id_task, missions[i].ports.size());
    
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

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  /*float lin_x = vel->linear.x;
  float ang_z = vel->angular.z;
  if(floatSimilarTo(lin_x, 0.0, 0.001) && floatSimilarTo(ang_z, 0.0, 0.001) && kobuki_state == Navigating)
  {
    printf("Kobuki_%i is stuck, count: %i\n", kobuki_id, count_vel);
    count_vel++;
    may_unstuck = false;*/

}
/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_operation_ports");

  ros::NodeHandle n;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0,1.0);

  // Subscribers
  ros::Subscriber mission_sub = n.subscribe(state_sub_topic, 1000, missionStateCallback);
  ros::Subscriber pose_sub = n.subscribe(pose_sub_topic, 1000, actualPoseCallback);
  ros::Subscriber goal_status_sub = n.subscribe(goal_status_sub_topic, 1000, statusGoalCallback);
  ros::Subscriber end_mission_sub = n.subscribe("/mission_accomplished", 1000, endMissionCallback);
  ros::Subscriber cmd_vel_sub = n.subscribe("mobile_base/commands/raw_velocity", 1, cmdvelCallback);

  // Publishers
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  ros::Publisher task_pub = n.advertise<fuzzymar_multi_robot::action_task_w_ports>("/treball", 10);
  ros::Publisher cancel_navigation_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);
  ros::Publisher task_ports_pub = n.advertise<fuzzymar_multi_robot::taskObjective>("/decidedTask", 10);
  ros::Publisher rotate_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);

  // Params
  n.getParam("robot_operation_ports/kobuki_id", kobuki_id);   // "/node_name(indicated inside this code)/param_name" --> the first '/' depends if groupns is used or not
  n.getParam("robot_operation_ports/alpha_utility", alpha_utility);

  ros::Rate loop_rate(10);

  ros::service::waitForService("move_base/clear_costmaps");
  ros::ServiceClient clearClient = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  std_srvs::Empty srv;

  int loop_counter = 0;

  bool first_loop = true; 

  uint8_t count_no_port_assigned_msg = 0;
  bool recalculate = false;
  bool disponible_ports = true;

  uint8_t aux_current_goal; 
  uint8_t aux_current_port;
  uint8_t aux_task_number; // how much tasks are in missions vector
  float threshold;
  float random;
  bool has_to_rotate = false;
  int rotate_publications = 0;

  printf("\nKobuki_%i has an alpha_utility of %i\n", kobuki_id, alpha_utility);

  while (ros::ok())
  {

    // KOBUKI_STATE UPDATE
    if(status == 3 && kobuki_state == Navigating) { // TODO es possible que haguem d'afegir status == 3 && kobuki_state == Computing per si sa tasca està a la posició actual del robot (no s'ha de moure per anar a la tasca)
      kobuki_state = Start_task;
      init_task = ros::Time::now();
      //ROS_INFO("Start_task");
    } else if((status == 1 && kobuki_state == Working) || (status == 1 && kobuki_state == Start_task)) {
      kobuki_state = Leave_task;
      leave_task = ros::Time::now();
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

      if(!disponible_ports)
      {
        printf("Kobuki_%i has not any disponible port. The navigation is stopped waiting for possible ports.\n", kobuki_id);
        actionlib_msgs::GoalID cancel_msg;
        cancel_navigation_pub.publish(cancel_msg);
        recalculate = false;
        //disponible_ports = true;
        
      }

      if(end_mission || (!disponible_ports && totalNumPorts() < TOTAL_ROBOTS)) // mission is accomplished
      {
        printf("Kobuki_%i finish their mission\n", kobuki_id);
        actionlib_msgs::GoalID cancel_msg;
        cancel_navigation_pub.publish(cancel_msg);
        ros::shutdown();
        
      } else {

        if(first_loop || aux_task_number != missions.size() || recalculate){  // NOW ONLY RECALCULATE A POSSIBLE NEW GOAL WHEN A TASK IS END ¿¿¿¿¿¿ HOW TO DO ??????

          setProbabilities(threshold); // the probabilities for each task are obtained

          random = dis(gen);

          current_goal = setTaskObjective(random); // set de current objective (task) for the robot

          setPortPriority();

          publishTaskObjective(&task_ports_pub);
          if(recalculate){printf("Kobuki_%i ***recalculate*** the task and port.\n", kobuki_id);}

          disponible_ports = disponiblePorts();
          recalculate = false;

        }


        if((first_loop || current_goal.id_task != aux_current_goal))  // only publish new goal if is the first task or a new task assignment (avoiding republish the same goal/task)
        {
          //publishGoal(&goal_pub, current_goal); // the current_goal is published on /kobuki_x/move_base_simple/goal topic
          calculated_task = true;  
          //port_assigned = false;  
        } else if(port_assigned)  // only publish new goal if is the first task or a new task assignment
        {
          printf("Kobuki_%i publish port %i of task %i localized on -> x: %5.2f y: %5.2f yaw: %5.2f\n", kobuki_id, current_port.id_port, current_goal.id_task, current_port.x, current_port.y, current_port.yaw);
          publishPortGoal(&goal_pub, current_port); // the current_goal is published on /kobuki_x/move_base_simple/goal topic

          port_assigned = false;
          calculated_task = false;
          //count_recalculate = 0;

        } else if (calculated_task && !port_assigned) // to avoid the robot try to go to a task without available ports (robot doesn't has a port assigned)
        {
          count_no_port_assigned_msg++;

          if(count_no_port_assigned_msg > 5) // in that point we should have been received a port assignation, if not robot has to recalculate
          {
            printf("Kobuki_%i has to recalculate the task and port.\n", kobuki_id);
            recalculate = true;
            count_no_port_assigned_msg = 0;
            //count_recalculate++;
          }
          
          
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
          ROS_INFO("ID: %i , D_STIM:%f, U_STIM:%f, STIMULOUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f", probabilities[i].id_task, probabilities[i].d_stimulous, probabilities[i].u_stimulous, probabilities[i].stimulous,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
        }

        // Current_goal vector
        ROS_DEBUG("Current task -> ID: %i , X: %f , Y: %f , YAW: %f", current_goal.id_task, current_goal.x,current_goal.y, current_goal.yaw);

        // Current position
        ROS_DEBUG("Current position -> X: %f, Y: %f", current_position.first, current_position.second);
      }

      aux_current_goal = current_goal.id_task;
      aux_current_port = current_port.id_port;
      aux_task_number = (uint8_t)missions.size();
      new_mission_state = false;

      first_loop = false;

    }

    if(kobuki_state == Start_task || kobuki_state == Leave_task) // send when robot arrives to the current task (goal reached) or leave a task
    {
      // publish
      publishTask(&task_pub); // publish {id_kobuki, id_task, sec, nsec, active} --> active indicates if is starting (true) or leaving (false) the task
      
    }

    if(loop_counter == 100 || kobuki_state == Stuck || count_vel > 50){ // if the robot abort the mission or each 10 secs, clear the costmaps and publish again the objective
      

      //if(kobuki_state == Stuck){publishGoal(&goal_pub, current_goal);} // republish the objective
      if(kobuki_state == Stuck || count_vel > 50)
      {
        printf("Kobuki_%i executing unstuck behavior.\n", kobuki_id);
        //for(int i = 0 ; i < 50 ; i++){
          publishRotation(&rotate_pub);
        //}
        rotate_publications++;
        if(rotate_publications > (1.5708*10))
        {
          publishPortGoal(&goal_pub, current_port);
          rotate_publications = 0;
          count_vel = 0;
        }
        loop_counter = 0;
        
      } else {

        clearClient.call(srv);

      }
      
      loop_counter = 0;
    }

    ros::spinOnce();
   
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

bool disponiblePorts()
{
  for(int i = 0 ; i < missions.size() ; i++)
  {
    for(int j = 0 ; j < missions[i].ports.size() ; j++)
    {
      if(missions[i].ports[j].id_kobuki == 0 || missions[i].ports[j].id_kobuki == kobuki_id) // can consider the port occupied by the own robot as a free port
      {
        return true;
      }
    }
  }

  return false;
}

int totalNumPorts()
{
  int num_ports = 0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    for(int j = 0 ; j < missions[i].ports.size() ; j++)
    {
      num_ports++;
    }
  }

  return num_ports;
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

float getPortsStimulous(int task)
{
  float stim = 0.0;
  uint8_t free_ports = 0;
  int tot_ports = missions[task-1].ports.size();

  for(int i = 0 ; i < tot_ports ; i++)
  {
    if(missions[task-1].ports[i].id_kobuki == 0 || missions[task-1].ports[i].id_kobuki == kobuki_id) // can be considered if port is occupied by the own robot, the port is free
    {
      free_ports++;
    }
  }

  /*if(missions[task-1].ports.size() - occupied_ports == 3 && missions[task-1].ports.size() > 3)
  {
    stim = 0.95;
  } else if(missions[task-1].ports.size() - occupied_ports == 2 && missions[task-1].ports.size() > 2)
  {
    stim = 0.9;
  } else if(missions[task-1].ports.size() - occupied_ports == 1 && missions[task-1].ports.size() > 1)
  {
    stim = 0.8;
  } else {
    stim = 1.0;
  }
  if(missions[task-1].ports.size() == occupied_ports)
  {
    stim = 0.0;
  }*/

  stim = ((free_ports * (1.0 - min_port_stim)) / tot_ports) + min_port_stim;
  if(free_ports == 0){stim = 0.0;}

  return stim;
}

void setProbabilities(float thres)
{
  probabilities.clear();

  uint8_t id;
  float stim, prob, norm_prob, accumul;
  float distance_stim = 0.0;
  float utility_stim = 0.0;
  float ports_stim = 0.0;
  float sum_prob, sum_norm;
  sum_prob = 0.0;
  sum_norm = 0.0;

  for(int i = 1 ; i <= missions.size() ; i++)
  {
    id = missions[i-1].id_task;
    ports_stim = getPortsStimulous(i); // using ports stimulous
    // OLD SIMPLY VERSION --> stim = getStimulous(i); // i+1 because we give the id task not the vector position
    //                        prob = (stim*stim) / ((stim*stim) + (thres*thres));
    distance_stim = 1/getStimulous(i);
    utility_stim = getUtilityStimulous(i);
    stim = (alpha_utility * ((thres*2.0)/(getUmax()))) * utility_stim + distance_stim; // thres*2.0 == d_max
    prob = ports_stim * ((thres*thres) / ((stim*stim) + (thres*thres))); // using ports stimulous
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
      
      break; // out of loop
    }
  }

  aux_objective = {id, x, y, yaw};
  return aux_objective;

}

bool sortbysec(const std::pair<int,float> &a, const std::pair<int,float> &b)
{
  return (a.second < b.second);
}

void setPortPriority()
{

  std::vector<std::pair<int,float>> distance_ports;
  std::pair<int,float> aux_ports;
  ports_priority.clear();

  for(int i = 0 ; i < missions.size() ; i++) // loop to get the positon of the task in the missions vector
  {
    if(current_goal.id_task == missions[i].id_task)
    {
      
      for(int j = 0 ; j < missions[i].ports.size() ; j++)
      {
        aux_ports.first = missions[i].ports[j].id_port;
        aux_ports.second = distance(current_position.first, current_position.second, missions[i].ports[j].x, missions[i].ports[j].y);
        distance_ports.push_back(aux_ports);
      }
      break;
    }
  }
  /*printf("\n\n\nCurrent task is %i and has distance ports of:\n", current_goal.id_task);
  for(int n = 0 ; n < distance_ports.size() ; n++)
  {
    printf("      Port %i , distance: %6.3f\n", distance_ports[n].first, distance_ports[n].second);
  }*/

  sort(distance_ports.begin(), distance_ports.end(), sortbysec);

  //printf("\n\n\nCurrent task is %i and has distance ports of(SORTED):\n", current_goal.id_task);
  for(int n = 0 ; n < distance_ports.size() ; n++)
  {
    uint8_t id_priority = distance_ports[n].first;
    ports_priority.push_back(id_priority);
    //printf("      Port %i , distance: %6.3f\n", distance_ports[n].first, distance_ports[n].second);
  }

  /*for(int n = 0 ; n < ports_priority.size() ; n++)
  {
    printf("          %i\n", ports_priority[n]);
  }*/

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
  fuzzymar_multi_robot::action_task_w_ports aux_task;

  aux_task.id_kobuki = kobuki_id;
  
  if(kobuki_state == Start_task)
  {
    aux_task.sec = init_task.sec;
    aux_task.nsec = init_task.nsec;
    aux_task.active = true;
    aux_task.id_task = current_goal.id_task;  // TODO millorar aquesta chapuzilla !!!!!!
    printf("\nKobuki_%i starts task %i at sec:%i nsec: %i\n", aux_task.id_kobuki, aux_task.id_task, aux_task.sec, aux_task.nsec);
  } else {
    aux_task.sec = leave_task.sec;
    aux_task.nsec = leave_task.nsec;
    aux_task.active = false;
    aux_task.id_task = last_task_id;  // TODO millorar aquesta chapuzilla !!!!!!
    printf("\nKobuki_%i leave task %i at sec:%i nsec: %i\n", aux_task.id_kobuki, aux_task.id_task, aux_task.sec, aux_task.nsec);
  }

  last_task_id = current_goal.id_task;

  task_pub->publish(aux_task);
}

void publishTaskObjective(ros::Publisher* task_ports_pub)
{
  fuzzymar_multi_robot::taskObjective aux_msg;

  aux_msg.id_kobuki = kobuki_id;
  aux_msg.id_task = current_goal.id_task;
  //printf("Port priority of robot %i for task %i:\n", aux_msg.id_kobuki, aux_msg.id_task);
  for(int i = 0 ; i < ports_priority.size() ; i++)
  {
    aux_msg.port_priority.push_back(ports_priority[i]);
    //printf("      %i \n", ports_priority[i]); 
  }

  task_ports_pub->publish(aux_msg);
}

void publishPortGoal(ros::Publisher* goal_pub, fuzzymar_multi_robot::port goal)
{
  geometry_msgs::PoseStamped goal_msg;

  goal_msg.header.frame_id = "map";
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.pose.position.x = goal.x;
  goal_msg.pose.position.y = goal.y;
  goal_msg.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY( 0.0, 0.0, goal.yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
  quaternion = quaternion.normalize();
  goal_msg.pose.orientation = tf2::toMsg(quaternion);

  /*goal_msg.pose.orientation.x = 0.0;
  goal_msg.pose.orientation.y = 0.0;    // TODO fer l'orientació com toca 
  goal_msg.pose.orientation.z = 0.0;
  goal_msg.pose.orientation.w = 1.0; */ 

  goal_pub->publish(goal_msg);
}

void publishRotation(ros::Publisher* rotate_pub)
{
  geometry_msgs::Twist vel;

  vel.linear.x = 0.0;
  vel.linear.y = 0.0;
  vel.linear.z = 0.0;

  vel.angular.x = 0.0;
  vel.angular.y = 0.0;
  vel.angular.z = 1.0;

  rotate_pub->publish(vel);
}

bool floatSimilarTo(float a, float b, float diff)
{
  float diference = a - b;

  return (diference < diff) && (-diference < diff);
}