/**
 * @Author: Toni Tauler
 * @File: pos_robot_operation_ports.cpp
 * @Description: TODO
 * @Date: February 2022
 */

#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalStatusArray.h"

#include <dynamic_reconfigure/server.h>
#include <fuzzymar_multi_robot/kobukisConfig.h>

#include <tf2/LinearMath/Quaternion.h> // from rpy to quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include "fuzzymar_multi_robot/defs.h"
#include "fuzzymar_multi_robot/stimulus.h"
//#include "fuzzymar_multi_robot/probabilities.h"

#define DEBUGGING 0
#define CLEAR_COSTMAP 0
#define TOTAL_ROBOTS 5
#define LOOP_RATE 20

// ENUMS

enum Kobuki_State {
  Computing, Start_task, Navigating, Working, Leave_task, Stuck
};

// Global Variables

std::string state_sub_topic = "/mission_state";
std::string pose_sub_topic = "amcl_pose";
std::string goal_status_sub_topic = "move_base/status";

//std::vector<Task> missions; // vector where the tasks are saved {id_task, deadline, weight, x, y, doing_task, enable}
//std::pair<float, float> current_position; // current position of the robot
//Current_goal current_goal; // current task declared as objective {task_id, x, y, yaw}

uint8_t status = 255; // objective status of the robot -> 1 == in the way to goal // 3 == goal reached // 255 == no status 

ros::Time init_task; // time at which the robot begins to work in the current task 
ros::Time leave_task; // time at which the robot leaves the current task

Kobuki_State kobuki_state = Computing;

uint8_t last_task_id;

std::vector<uint8_t> ports_priority;

float threshold;
float max_dist;
float utility_max;

// FLAGS
bool new_mission_state = false;
bool end_mission = false;
bool port_assigned = false;
bool calculated_task = false;

// FUNCTIONS DECLARATION
//float distance(float x1, float y1, float x2, float y2); // calculate the distance between 2 points in 2D
bool disponiblePorts(); // disponible ports comprovation
//int totalNumPorts(); // counts the total num of ports assigned, by task_ports node, in the mission
//float getMaxDist(); // Get the max distance between tasks and between robot and tasks
//float getMaxDistVAR(); // get the max distance between robot and tasks
//float getUtilityDD(int id_task); // get the Utility of a task
//float getUmax(); // get the max U of all the tasks
//float getUmaxDL(); // get the max U of all the tasks taking account the U variable depending on DL
//float getDistanceStimulus(int task); // get the current robot stimulus for a given task
//float getUtilityStimulus(int task); // get the utility stimulus
//float getUtility_w_DeadlineStimulus(int task); // get the utility stimulus taking into account the deadline
//void setProbabilities(float thres, float d_max, float u_max); // build the probabilities vector {id_task, stimulus, probability}

Current_goal setTaskObjectivePossibilistic(float rand); // return the global goal {task_id, x, y, yaw} to publish to /kobuki_x/move_base_simple/goal
Current_goal setTaskObjectiveDeterministic();

void setPortPriority(); //sets the priority of the ports for the current task (use distance as priority)

void publishGoal(ros::Publisher* goal_pub, Current_goal goal);
void publishPortGoal(ros::Publisher* goal_pub, fuzzymar_multi_robot::port goal);
void publishTask(ros::Publisher* task_pub);  
void publishTaskObjective(ros::Publisher* task_ports_pub);
void publishRobParamInfo(ros::Publisher& rob_param_pub);

void publishRotation(ros::Publisher* rotate_pub);

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

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  //ROS_INFO("odomMSG");
  if(doubleSimilarTo((double)odom_msg->twist.twist.linear.x, (double)0.0, 0.0001) && doubleSimilarTo((double)odom_msg->twist.twist.angular.z, (double)0.0, 0.0001) && kobuki_state == Navigating)
  {
    
    count_stuck++;
    //ROS_INFO("*************************** MAY STUCK ****************************************");
    if(count_stuck > 2*LOOP_RATE){sure_stuck = true; /*printf("***************************** ROBOT STUCKED *******************************\n");*/}
  }

}

/********************************************************************************************
***************************************** MAIN **********************************************
********************************************************************************************/

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pos_robot_operation_ports");

  dynamic_reconfigure::Server<fuzzymar_multi_robot::kobukisConfig> server;
  dynamic_reconfigure::Server<fuzzymar_multi_robot::kobukisConfig>::CallbackType f;

  ros::NodeHandle n;

  /*std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0,1.0);*/
  

  // Subscribers
  ros::Subscriber mission_sub = n.subscribe(state_sub_topic, 1000, missionStateCallback);
  ros::Subscriber pose_sub = n.subscribe(pose_sub_topic, 1000, actualPoseCallback);
  ros::Subscriber goal_status_sub = n.subscribe(goal_status_sub_topic, 1000, statusGoalCallback);
  ros::Subscriber end_mission_sub = n.subscribe("/mission_accomplished", 1000, endMissionCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 1, odomCallback);

  // Publishers
  ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  ros::Publisher task_pub = n.advertise<fuzzymar_multi_robot::action_task_w_ports>("/treball", 10);
  ros::Publisher cancel_navigation_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 10);
  ros::Publisher task_ports_pub = n.advertise<fuzzymar_multi_robot::taskObjective>("/decidedTask", 10);
  ros::Publisher rotate_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
  ros::Publisher rob_param_pub = n.advertise<fuzzymar_multi_robot::robotParameters>("/rob_parameters_info", 10);

  // Params
  n.getParam("pos_robot_operation_ports/kobuki_id", kobuki_id);   // "/node_name(indicated inside this code)/param_name" --> the first '/' depends if groupns is used or not
  n.getParam("pos_robot_operation_ports/alpha_utility", alpha_utility);
  n.getParam("pos_robot_operation_ports/beta_distance", beta_distance);
  n.getParam("pos_robot_operation_ports/gamma_ports", gamma_ports);
  n.getParam("pos_robot_operation_ports/UDD_factor", UDD_factor);
  //n.getParam("pos_robot_operation_ports/possibilistic", possibilistic);
  //n.getParam("pos_robot_operation_ports/dynamic_max", dynamic_max);


  srand(kobuki_id); // define the seed of the random number using the kobuki_id

  ros::param::get("velocity_smoother/speed_lim_v", max_vel);
  max_vel_aux = max_vel;

  ros::Rate loop_rate(LOOP_RATE);

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
  int rotated_360 = 0;

  printf("\nKobuki_%i has an alpha_utility: %5.2f , beta_distance: %5.2f, gamma_ports: %5.2f , UDD_factor: %6.4f, max_vel: %5.2f, ", kobuki_id, alpha_utility, beta_distance, gamma_ports, UDD_factor, max_vel);
  
  printf(" POSSIBILISTIC\n");

  while (ros::ok())
  {

    // KOBUKI_STATE UPDATE
    if(status == 3 && kobuki_state == Navigating) { // TODO es possible que haguem d'afegir status == 3 && kobuki_state == Computing per si sa tasca està a la posició actual del robot (no s'ha de moure per anar a la tasca)
      kobuki_state = Start_task;
      has_worked = true;
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

    if(kobuki_state == Start_task || kobuki_state == Leave_task) // send when robot arrives to the current task (goal reached) or leave a task
    {
      // publish
      publishTask(&task_pub); // publish {id_kobuki, id_task, sec, nsec, active} --> active indicates if is starting (true) or leaving (false) the task
      
    }

    if(new_mission_state) // when missions is updated
    {
      
      //totalNumPorts(missions);

      if(first_loop){ 
        utility_max = getUmax();
        threshold =  4 / getMaxDist();// the threshold is gotten
        max_dist = getMaxDist();

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

          setProbabilities(threshold, max_dist, utility_max, mission_time, utility_max); // the probabilities for each task are obtained

          //random = dis(gen);
          random = float((float)rand() / (float)RAND_MAX);
          current_goal = setTaskObjectivePossibilistic(random); // set de current task (task objective) for the robot

          
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

          // IF NEW TASK OBJECTIVE IS CALCULATED HAS TO CANCEL THE LAST OBJECTIVE
          actionlib_msgs::GoalID cancel_msg;
          cancel_navigation_pub.publish(cancel_msg);
          
          //port_assigned = false;  
        } else if(port_assigned)  // only publish new goal if is the first task or a new task assignment
        {
          printf("Kobuki_%i publish port %i of task %i localized on -> x: %5.2f y: %5.2f yaw: %5.2f\n", kobuki_id, current_port.id_port, current_goal.id_task, current_port.x, current_port.y, current_port.yaw);
          publishPortGoal(&goal_pub, current_port); // the current_goal is published on /kobuki_x/move_base_simple/goal topic
          clearClient.call(srv);

          port_assigned = false;
          calculated_task = false;
          //count_recalculate = 0;

        } else if (calculated_task && !port_assigned) // to avoid the robot try to go to a task without available ports (robot hasn't got a port assigned)
        {
          count_no_port_assigned_msg++;

          if(count_no_port_assigned_msg > LOOP_RATE/2) // in that point we should have been received a port assignation, if not robot has to recalculate
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


    // UNSTUCK BEHAVIOR

    if(loop_counter >= 10*LOOP_RATE || kobuki_state == Stuck || sure_stuck){ // if the robot abort the mission or each 10 secs, clear the costmaps and publish again the objective
      

      //if(kobuki_state == Stuck){publishGoal(&goal_pub, current_goal);} // republish the objective
      if(kobuki_state == Stuck || sure_stuck)
      {
        
        //for(int i = 0 ; i < 50 ; i++){
          actionlib_msgs::GoalID cancel_msg;
          cancel_navigation_pub.publish(cancel_msg);
          publishRotation(&rotate_pub);
        //}
        rotate_publications++;
        if(rotate_publications > (1.5708*LOOP_RATE))
        {
          
          rotate_publications = 0;
          sure_stuck = false;
          //may_stuck = false;
          count_stuck = 0; 
          publishPortGoal(&goal_pub, current_port);
          printf("Kobuki_%i has executed unstuck behavior.\n", kobuki_id);
          // when robot rotates 360º clear costmaps
          if(rotated_360 > 4) 
          {
            clearClient.call(srv);
            printf("Kobuki_%i clears costmaps due to unstuck behavior.\n", kobuki_id);
            rotated_360 = 0;
          } 
          rotated_360++;
          //
        }
        loop_counter = 0;
        
      } else {
        
        //printf("Kobuki_%i clear costmap.\n", kobuki_id);
        clearClient.call(srv);
        loop_counter = 0;
        rotated_360 = 0;
      }
      
      
    }
    // this if structure to be sure the goal is republished after UNSTUCK BEHAVIOR
    if(rotate_publications == 0 && status == 2)
    {
      publishPortGoal(&goal_pub, current_port);
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
  aux.selection_task = "possibilistic";
  aux.calc_stim_time = -1.0;
  aux.calc_stim_way = false;
  aux.sdl_method = -1;
  aux.agregation_type = -1;
  aux.w1 = -1.0;
  aux.w2 = -1.0;
  aux.w3 = -1.0;

  rob_param_pub.publish(aux);
}
