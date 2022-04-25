/**
 * @Author: Toni Tauler
 * @File: stimulus.h
 * @Description: TODO
 * @Date: April 2022
 
#include <stdio.h>
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
#include "fuzzymar_multi_robot/probabilities.h"

void setProbabilities(float thres, float d_max, float u_max, ros::Time mission_time, float utility_max)
{
  probabilities.clear();

  uint8_t id;
  float stim, prob, norm_prob, accumul;
  float distance_stim, d_stim_norm = 0.0;
  float utility_stim, u_stim_norm = 0.0;
  float ports_stim, p_stim_norm = 0.0;
  float sum_prob, sum_norm;
  sum_prob = 0.0;
  sum_norm = 0.0;

  printf("\n\n");

  for(int i = 1 ; i <= missions.size() ; i++)
  {
    id = missions[i-1].id_task;

    utility_stim = getUtility_w_DeadlineStimulus(i, mission_time, utility_max);
    distance_stim = getDistanceStimulus(i) / max_vel;
    ports_stim = getPortsStimulus(i); // using ports stimulus

    // VARIABLE D_MAX && U_MAX 
    /*
    
    */
    // STATIC D_MAX && U_MAX
    if(dynamic_max)
    {
      u_stim_norm = (1/getUmaxDL(mission_time, utility_max)) * utility_stim;
      d_stim_norm = (max_vel/getMaxDist()) * distance_stim;
      p_stim_norm = ports_stim;
    } else {
      u_stim_norm = (1/u_max) * utility_stim;
      d_stim_norm = (max_vel/d_max) * distance_stim;
      p_stim_norm = ports_stim;
    }
    

    //stim = (alpha_utility * (d_max/u_max)) * utility_stim + beta_distance * distance_stim + gamma_ports * ports_stim; // 
    stim = alpha_utility * u_stim_norm + beta_distance * d_stim_norm + gamma_ports * p_stim_norm; 
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

    Probability aux_prob = {id, d_stim_norm, u_stim_norm, p_stim_norm, stim, prob, norm_prob, accumul};

    probabilities.push_back(aux_prob);
  }
  
  for(int j = 0 ; j < probabilities.size() ; j++) { // fill the normalized probability and the accumulative normalized probability
    probabilities[j].norm_probability = probabilities[j].probability / sum_prob;
    sum_norm += probabilities[j].norm_probability;
    probabilities[j].accumulative = sum_norm;
  }

  /*for(int i = 0 ; i < probabilities.size() ; i++)
  {
    ROS_INFO("ID: %i , D_STIM: %5.3f, U_STIM: %f, P_STIM: %f, STIMULUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].p_stimulus, probabilities[i].stimulus,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
  }*/

}
