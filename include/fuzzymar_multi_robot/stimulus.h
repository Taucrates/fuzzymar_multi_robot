/**
 * @Author: Toni Tauler
 * @File: stimulus.h
 * @Description: TODO
 * @Date: April 2022
*/

#ifndef STIMULUS_H
#define STIMULUS_H

/*#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>
#include <random>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fuzzymar_multi_robot/task_w_ports.h>
#include <fuzzymar_multi_robot/task_w_portsArray.h>
#include <fuzzymar_multi_robot/action_task_w_ports.h>
#include <fuzzymar_multi_robot/taskObjective.h>
#include <fuzzymar_multi_robot/robotParameters.h> */
#include "fuzzymar_multi_robot/defs.h"

float distance(float x1, float y1, float x2, float y2); // calculate the distance between 2 points in 2D
int totalNumPorts(); // counts the total num of ports assigned, by task_ports node, in the mission
int robotsAssigned(int task);
bool doubleSimilarTo(double a, double b, double diff);

float getMaxDist(); // Get the max distance between tasks and between robot and tasks
//float getMaxDistVAR(std::vector<Task> missions, std::pair<float, float> current_position); // get the max distance between robot and tasks
float getUtilityDD(int id_task,ros::Time mission_time, float utility_max); // get the Utility of a task
float getUmax(); // get the max U of all the tasks
float getUmaxDL(ros::Time mission_time,float utility_max); // get the max U of all the tasks taking account the U variable depending on DL

// STIMULUS
float getDistanceStimulus(int task); // get the current robot stimulus for a given task
float getUtilityStimulus(int task); // get the utility stimulus
float getUtility_w_DeadlineStimulus(int task, ros::Time mission_time, float utility_max); // get the utility stimulus taking into account the deadline
float getPortsStimulus(int task);

// NEW STIMULUS
float sdl_V1(int task);   // utility stimulus version 1 (sdl1(t))
float sdl_V2(int task);   // utility stimulus version 2 (sdl2(t))

float sds(int task, Current_goal current_task);      // distance stimulus (sds(t))

float iL(int task);       // ports stimulus (iL(occupied(t)))


// PROBABILITIES TABLE
void setProbabilities(float thres, float d_max, float u_max, ros::Time mission_time, float utility_max); // build the probabilities vector {id_task, stimulus, probability}

// STIMULUS TABLE
void setStimulusDet(Current_goal current_task, int sdl_method, int agregation_type, float w1, float w2, float w3);   // build the vector probabilities

#endif // STIMULUS_H 