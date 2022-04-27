/**
 * @Author: Toni Tauler
 * @File: stimulus.h
 * @Description: TODO
 * @Date: April 2022
*/ 

#ifndef PROBABILITIES_H
#define PROBABILITIES_H

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
#include "fuzzymar_multi_robot/stimulus.h"

// STRUCTS
struct Probability {
  uint8_t id_task;
  double d_stimulus;
  double u_stimulus;
  double p_stimulus;
  double stimulus;
  double probability;
  double norm_probability;
  double accumulative;
};

std::vector<Probability> probabilities; // vector where probabilities are saved

bool doubleSimilarTo(double a, double b, double diff);

// PROBABILITIES TABLE
void setProbabilities(float thres, float d_max, float u_max, ros::Time mission_time, float utility_max); // build the probabilities vector {id_task, stimulus, probability}

// STIMULUS TABLE
void setStimulusDet(Current_goal current_task, int sdl_method, int agregation_type, float w1, float w2, float w3);   // build the vector probabilities
#endif // PROBABILITIES_H 