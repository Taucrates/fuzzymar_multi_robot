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
#include <fuzzymar_multi_robot/robotParameters.h>

#include <dynamic_reconfigure/server.h>
#include <fuzzymar_multi_robot/kobukisConfig.h>

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
  float utility;
  std::vector<fuzzymar_multi_robot::port> ports;
};

struct Probability {
  uint8_t id_task;
  float d_stimulus;
  float u_stimulus;
  float p_stimulus;
  float stimulus;
  float probability;
  float norm_probability;
  float accumulative;
};

struct Current_goal {
  uint8_t id_task;
  float x;
  float y;
  float yaw;
  float utility;
};


// ENUMS

enum Kobuki_State {
  Computing, Start_task, Navigating, Working, Leave_task, Stuck
};

// GLOBAL VARIABLES
int kobuki_id;

bool possibilistic = false;

float alpha_utility = 2.0;
float beta_distance = 1.0;
float gamma_ports = 0.5;

float UDD_factor = 0.07;
float max_vel = 0.2;
float max_vel_aux = 0.2;

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

ros::Time mission_time; // the time at wich the mission starts


Kobuki_State kobuki_state = Computing;

uint8_t last_task_id;

std::vector<uint8_t> ports_priority;

float min_port_stim = 0.8; // min_value at which portstimulous begin

int count_vel = 0;
bool may_unstuck = false;

float threshold;
float max_dist;
float utility_max;

// FLAGS
bool new_mission_state = false;
bool end_mission = false;
bool port_assigned = false;
bool calculated_task = false;

// FUNCTIONS DECLARATION
float distance(float x1, float y1, float x2, float y2); // calculate the distance between 2 points in 2D
bool disponiblePorts(); // disponible ports comprovation
int totalNumPorts(); // counts the total num of ports assigned, by task_ports node, in the mission
float getMaxDist(); // get the threshold == 2/max_dist(between tasks)
float getUmax(); // get the max dif between all the tasks
float getDistanceStimulus(int task); // get the current robot stimulus for a given task
float getUtilityStimulus(int task); // get the utility stimulus
float getUtility_w_DeadlineStimulus(int task); // get the utility stimulus taking into account the deadline
void setProbabilities(float thres, float u_max, float d_max); // build the probabilities vector {id_task, stimulus, probability}

Current_goal setTaskObjectivePossibilistic(float rand); // return the global goal {task_id, x, y, yaw} to publish to /kobuki_x/move_base_simple/goal
Current_goal setTaskObjectiveDeterministic();

void setPortPriority(); //sets the priority of the ports for the current task (use distance as priority)

void publishGoal(ros::Publisher* goal_pub, Current_goal goal);
void publishPortGoal(ros::Publisher* goal_pub, fuzzymar_multi_robot::port goal);
void publishTask(ros::Publisher* task_pub);  
void publishTaskObjective(ros::Publisher* task_ports_pub);
void publishRobParamInfo(ros::Publisher& rob_param_pub);

void publishRotation(ros::Publisher* rotate_pub);

bool floatSimilarTo(float a, float b, float diff);

/********************************************************************************************
*************************************** CALLBACKS *******************************************
********************************************************************************************/

void callback(fuzzymar_multi_robot::kobukisConfig &config, uint32_t level) {

  if(config.apply_changes)
  {
    max_vel = config.Max_vel;
    UDD_factor = config.UDD_factor;
    alpha_utility = config.Alpha_utility;
    gamma_ports = config.Alpha_ports;

    ros::param::set("velocity_smoother/speed_lim_v", max_vel);
    ros::param::get("velocity_smoother/speed_lim_v", max_vel_aux);

    printf("Kobuki_%i -> Max_vel: %f\n", kobuki_id, max_vel_aux);
  }
  
}

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

/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_operation_ports");

  dynamic_reconfigure::Server<fuzzymar_multi_robot::kobukisConfig> server;
  dynamic_reconfigure::Server<fuzzymar_multi_robot::kobukisConfig>::CallbackType f;

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
  ros::Publisher task_pub = n.advertise<fuzzymar_multi_robot::action_task_w_ports>("/treball", 10);
  ros::Publisher cancel_navigation_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);
  ros::Publisher task_ports_pub = n.advertise<fuzzymar_multi_robot::taskObjective>("/decidedTask", 10);
  ros::Publisher rotate_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
  ros::Publisher rob_param_pub = n.advertise<fuzzymar_multi_robot::robotParameters>("/rob_parameters_info", 10);

  // Params
  n.getParam("robot_operation_ports/kobuki_id", kobuki_id);   // "/node_name(indicated inside this code)/param_name" --> the first '/' depends if groupns is used or not
  n.getParam("robot_operation_ports/alpha_utility", alpha_utility);
  n.getParam("robot_operation_ports/beta_distance", beta_distance);
  n.getParam("robot_operation_ports/gamma_ports", gamma_ports);
  n.getParam("robot_operation_ports/UDD_factor", UDD_factor);
  n.getParam("robot_operation_ports/possibilistic", possibilistic);

  ros::param::get("velocity_smoother/speed_lim_v", max_vel);
  max_vel_aux = max_vel;

  ros::Rate loop_rate(10);

  ros::service::waitForService("move_base/clear_costmaps");
  ros::ServiceClient clearClient = n.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  std_srvs::Empty srv;

  int loop_counter = 0;

  bool first_loop = true; 

  current_goal.id_task = 0; // means not decided task

  uint8_t count_no_port_assigned_msg = 0;
  bool recalculate = false;
  bool disponible_ports = true;

  uint8_t aux_current_goal; 
  uint8_t aux_current_port;
  uint8_t aux_task_number; // how much tasks are in missions vector

  float random;
  bool has_to_rotate = false;
  int rotate_publications = 0;

  printf("\nKobuki_%i has an alpha_utility: %5.2f , beta_distance: %5.2f, gamma_ports: %5.2f , UDD_factor: %6.4f, max_vel: %5.2f ", kobuki_id, alpha_utility, beta_distance, gamma_ports, UDD_factor, max_vel);
  
  if(possibilistic){printf(" POSSIBILISTIC\n");} 
  else{printf(" DETERMINISTIC\n");}

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
        max_dist = getMaxDist();
        threshold =  4 / max_dist;// the threshold is gotten
        utility_max = getUmax();

        publishRobParamInfo(rob_param_pub);

        //printf("\nd_max: %f threshold: %f Umax: %f\n",max_dist, threshold, utility_max);
        mission_time = ros::Time::now();
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
        printf("Kobuki_%i finish its mission\n", kobuki_id);
        actionlib_msgs::GoalID cancel_msg;
        cancel_navigation_pub.publish(cancel_msg);
        ros::shutdown();
        
      } else {

        if(first_loop || aux_task_number != missions.size() || recalculate){  // NOW ONLY RECALCULATE A POSSIBLE NEW GOAL WHEN A TASK IS END ¿¿¿¿¿¿ HOW TO DO ??????

          setProbabilities(threshold, utility_max, max_dist); // the probabilities for each task are obtained

          if(possibilistic)
          {
            random = dis(gen);
            current_goal = setTaskObjectivePossibilistic(random); // set de current task (task objective) for the robot

          } else {

            current_goal = setTaskObjectiveDeterministic();

          }
          

          setPortPriority(); // the port priority for the current task (using distance) is setted 

          publishTaskObjective(&task_ports_pub); // the task objective and its own port priority is published
          if(recalculate){printf("Kobuki_%i ***recalculate*** the task and port.\n", kobuki_id);}

          disponible_ports = disponiblePorts(); // set to true if there are free_ports in the current task
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

        } else if (calculated_task && !port_assigned) // to avoid the robot try to go to a task without available ports (robot hasn't got a port assigned)
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
          ROS_INFO("ID: %i , D_STIM:%5.3f, U_STIM:%f, STIMULUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].stimulus,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
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

    if(loop_counter == 100 || kobuki_state == Stuck){ // if the robot abort the mission or each 10 secs, clear the costmaps and publish again the objective
      

      //if(kobuki_state == Stuck){publishGoal(&goal_pub, current_goal);} // republish the objective
      if(kobuki_state == Stuck)
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
          
          clearClient.call(srv);
        }
        loop_counter = 0;
        
      } else {

        clearClient.call(srv);

      }
      
      loop_counter = 0;
    }

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

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

float getMaxDist()
{
  float aux_dist = 0.0;
  float dist = 0.0;
  bool first_aux = true;

  for(int i = 0 ; i < missions.size()  ; i++)
  {
    
    for (int j = 0 ; j < missions.size() ; j++)
    {
      if(first_aux){
        
        dist = distance(current_position.first, current_position.second, missions[j].x, missions[j].y);
        
      } else {
        
        dist = distance(missions[i].x, missions[i].y, missions[j].x, missions[j].y);
        
      }

      if(aux_dist < dist)
      {
        aux_dist = dist;
      }
    }
    first_aux = false;
  }
  
  return aux_dist;
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

float getDistanceStimulus(int task)
{
  float stim = distance(current_position.first, current_position.second, missions[task-1].x, missions[task-1].y); // TODO change 0.0's to /kobuki_x/amcl_pose x and y

  return stim;
}

float getUtilityStimulus(int task)
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

int robotsAssigned(int task)
{
  int num_robots = 0;

  for(int i = 0 ; i < missions[task].ports.size() ; i++)
  {
    if(missions[task].ports[i].id_kobuki != 0)
    {
      num_robots++;
    }
  }

  return num_robots;
}

float getNeededTime(int task) // fTj
{
  float needed_time = 0.0;

  needed_time = (ros::Time::now().toSec() - mission_time.toSec()) + (missions[task].weight / (robotsAssigned(task) + 1));

  return needed_time;
}

float getUtilityDD(int id_task)
{
  float utility = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    if(missions[i].id_task == id_task)
    {
      if(missions[i].deadline >= getNeededTime(i))
      {

        return utility = missions[i].utility;

      } else {

        return utility = missions[i].utility * ((UDD_factor * missions[i].deadline)/((getNeededTime(i) - missions[i].deadline) + UDD_factor * missions[i].deadline));

      }
    } 
  }

  return utility_max ;
}

float getUtility_w_DeadlineStimulus(int task)
{
  float stim = 0.0;
  int task_aux = task-1;

  /*if(current_goal.id_task == 0) // first time robot plan (not consider U stim)
  {
    //printf("INITIAL  Task_%i --> U: %f\n", missions[task_aux].id_task, 0.0);
    return stim = 0.0;

  } else {*/ // otherwise

    // ******************  MAY I HAVE TO CALCULATE Ui AS getUtilityDD(current_goal.id_task) *******************
    //printf("Task %i Ui: %f , Uj: %f , Ui-Uj = %f ||", missions[task_aux].id_task, getUtilityDD(current_goal.id_task), getUtilityDD(missions[task_aux].id_task), getUtilityDD(current_goal.id_task) - getUtilityDD(missions[task_aux].id_task));
    if(getUtilityDD(current_goal.id_task) - getUtilityDD(missions[task_aux].id_task) > 0.0)  // Ui > Uj
    {
      //printf("NORMAL  Task_%i --> U: %f\n", missions[task_aux].id_task, (current_goal.utility - getUtilityDD(missions[task_aux].id_task)));
      return stim = getUtilityDD(current_goal.id_task) - getUtilityDD(missions[task_aux].id_task);
      
    } else {                                                                   // Ui < Uj --> U = 0.0 
      //printf("0.0__Task_%i --> U: %f\n", missions[task_aux].id_task, 0.0);
      return stim = 0.0;
    }

    
  //} 

  
}

float getPortsStimulus(int task)
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

  if(free_ports == 0)
  {
    //printf("Task: %i No free ports\n", missions[task-1].id_task);
    return stim = -1.0;
  }
  //stim = ((free_ports * (1.0 - min_port_stim)) / tot_ports) + min_port_stim;
  
  //stim = (1 / (free_ports/float(tot_ports))) - 1;
  stim = free_ports/float(tot_ports);
  //printf("Task: %i Port_stim: %f\n", missions[task-1].id_task, stim);

  return stim;
}

void setProbabilities(float thres, float u_max, float d_max)
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

  printf("\n\n");

  for(int i = 1 ; i <= missions.size() ; i++)
  {
    id = missions[i-1].id_task;

    // OLD SIMPLY VERSION --> stim = getDistanceStimulus(i); // i+1 because we give the id task not the vector position
    //                        prob = (stim*stim) / ((stim*stim) + (thres*thres));

    //printf("Kobuki_%i -> Task_%i has Ui: %f and Uj: %f, Ujmax: %f  Result: ", kobuki_id, id, current_goal.utility, getUtilityDD(id), missions[i-1].utility);

    distance_stim = getDistanceStimulus(i) / max_vel;
    utility_stim = getUtility_w_DeadlineStimulus(i);
    ports_stim = getPortsStimulus(i); // using ports stimulus
                                    
    //stim = (alpha_utility * (d_max/u_max)) * utility_stim + beta_distance * distance_stim + gamma_ports * ports_stim; // 
    stim = (alpha_utility/u_max) * utility_stim + beta_distance * (max_vel/d_max) * distance_stim + gamma_ports * ports_stim; 
    //prob = ports_stim * ((thres*thres) / ((stim*stim) + (thres*thres))); // using ports stimulus
    //printf("Su: %f Sd: %f Sp: %f\n", (1/u_max) * utility_stim, (max_vel/d_max) * distance_stim, ports_stim);
    if(ports_stim < -0.1) // if task has not free ports prob has to be 0
    {

      prob = 0.0;

    } else {

      prob = 1 / (1 + (thres*thres)*(stim*stim));

    }
    
    norm_prob = 0.0;
    accumul = 0.0;
    sum_prob += prob;

    Probability aux_prob = {id, distance_stim, utility_stim, ports_stim, stim, prob, norm_prob, accumul};

    probabilities.push_back(aux_prob);
  }
  
  for(int j = 0 ; j < probabilities.size() ; j++) { // fill the normalized probability and the accumulative normalized probability
    probabilities[j].norm_probability = probabilities[j].probability / sum_prob;
    sum_norm += probabilities[j].norm_probability;
    probabilities[j].accumulative = sum_norm;
  }

  for(int i = 0 ; i < probabilities.size() ; i++)
  {
    ROS_INFO("ID: %i , D_STIM: %5.3f, U_STIM: %f, P_STIM: %f, STIMULUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].p_stimulus, probabilities[i].stimulus,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
  }

}

Current_goal setTaskObjectivePossibilistic(float rand)
{
  Current_goal aux_objective;
  uint8_t id;
  float x, y, yaw, utility;
  float prob = 0.0;
  
  for(int i = 0 ; i < probabilities.size() ; i ++)  // set a task for the robot from available ones
  {
    if(rand <= probabilities[i].accumulative){
      id = missions[i].id_task;
      x = missions[i].x;
      y = missions[i].y;
      yaw = 0.0;  
      utility = missions[i].utility;
      
      break; // out of loop
    }
  }

  aux_objective = {id, x, y, yaw, utility};
  return aux_objective;

}

Current_goal setTaskObjectiveDeterministic()
{
  Current_goal aux_objective;
  uint8_t id;
  float x, y, yaw, utility;
  float prob = 0.0;
  
  for(int i = 0 ; i < probabilities.size() ; i ++)  // set a task for the robot from available ones
  {
    if(prob <= probabilities[i].norm_probability){

      prob = probabilities[i].norm_probability;

      id = missions[i].id_task;
      x = missions[i].x;
      y = missions[i].y;
      yaw = 0.0;  
      utility = missions[i].utility;
      
    }
  }

  aux_objective = {id, x, y, yaw, utility};
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
    printf("\nKobuki_%i starts task %i at sec:%6.3f\n", aux_task.id_kobuki, aux_task.id_task, (ros::Time::now().toSec() - mission_time.toSec()));
  } else {
    aux_task.sec = leave_task.sec;
    aux_task.nsec = leave_task.nsec;
    aux_task.active = false;
    aux_task.id_task = last_task_id;  // TODO millorar aquesta chapuzilla !!!!!!
    printf("\nKobuki_%i leave task %i at sec:%6.3f\n", aux_task.id_kobuki, aux_task.id_task, (ros::Time::now().toSec() - mission_time.toSec()));
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

void publishRobParamInfo(ros::Publisher& rob_param_pub)
{
  fuzzymar_multi_robot::robotParameters aux;

  aux.id_robot = kobuki_id;
  aux.max_vel = max_vel;
  aux.UDD_factor = UDD_factor;
  aux.alpha_utility = alpha_utility;
  aux.beta_distance = beta_distance;
  aux.gamma_ports = gamma_ports;
  if(possibilistic){aux.selection_task = "possibilistic";}
  else{aux.selection_task = "deterministic";}

  rob_param_pub.publish(aux);
}


bool floatSimilarTo(float a, float b, float diff)
{
  float diference = a - b;

  return (diference < diff) && (-diference < diff);
}