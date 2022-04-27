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
  float distance_stim = 0.0, d_stim_norm = 0.0;
  float utility_stim = 0.0, u_stim_norm = 0.0;
  float ports_stim = 0.0, p_stim_norm = 0.0;
  float sum_prob = 0.0, sum_norm = 0.0;

  printf("\n\n");

  for(int i = 1 ; i <= missions.size() ; i++)
  {
    id = missions[i-1].id_task;

    utility_stim = getUtility_w_DeadlineStimulus(i, mission_time, utility_max);
    distance_stim = getDistanceStimulus(i) / max_vel;
    ports_stim = getPortsStimulus(i); // using ports stimulus

    // VARIABLE D_MAX && U_MAX 
    
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

  /*printf("KOBUKI_%i:\n", kobuki_id);
  for(int i = 0 ; i < probabilities.size() ; i++)
  {
    printf("ID: %i , D_STIM: %f, U_STIM: %f, P_STIM: %f, STIMULUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f\n", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].p_stimulus, probabilities[i].stimulus,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
  }*/

}

bool doubleSimilarTo(double a, double b, double diff)
{
  double difference = a - b;

  return (difference < diff) && (-difference < diff);
}

double middleValue(double u_stim, double d_stim, double p_stim) /// S'HA DE MIRAR BÉ!!!
{
  double aux_max = std::max({u_stim, d_stim, p_stim});
  double aux_min = std::min({u_stim, d_stim, p_stim});
  int num_max = 0;
  
  if(!doubleSimilarTo(u_stim, aux_max, 0.0000000000001) && !doubleSimilarTo(u_stim, aux_min, 0.0000000000001))
  {
    return u_stim;
  }

  if(!doubleSimilarTo(d_stim, aux_max, 0.0000000000001) && !doubleSimilarTo(d_stim, aux_min, 0.0000000000001))
  {
    return d_stim;
  }

  if(!doubleSimilarTo(p_stim, aux_max, 0.0000000000001) && !doubleSimilarTo(p_stim, aux_min, 0.0000000000001))
  {
    return p_stim;
  }
  
  if(doubleSimilarTo(u_stim, aux_max, 0.0000001)){num_max++;}
  if(doubleSimilarTo(d_stim, aux_max, 0.0000001)){num_max++;}
  if(doubleSimilarTo(p_stim, aux_max, 0.0000001)){num_max++;}

  if(num_max == 2)
  {
    return aux_max;
  } else {
    return aux_min;
  }

}

void setStimulusDet(Current_goal current_task, int sdl_method, int agregation_type, float w1, float w2, float w3)
{
  probabilities.clear();

  //printf("************************************ Ti: %i ********************************\n", current_task.id_task);

  uint8_t id;
  double stim, prob, norm_prob, accumul;
  double distance_stim, d_stim_norm = 0.0;
  double utility_stim, u_stim_norm = 0.0;
  double ports_stim, p_stim_norm = 0.0;
  double sum_prob, sum_norm;
  sum_prob = 0.0;
  sum_norm = 0.0;

  printf("\n\n");

  for(int i = 0; i < missions.size() ; i++)
  {
    id = missions[i].id_task;

    switch(sdl_method)  // way to calculate the utility stimulus
    {
      case(1):    // version 1
        utility_stim = sdl_V1(i);
        break;
      case(2):    // version 2
        utility_stim = sdl_V2(i);
        break;
      default:
        utility_stim = 0.0;
        break;
    }
    
    //distance_stim = sds(i, current_goal);   
    distance_stim = sds(i, current_task);   // OJO així fa que Ti ho sigui quan ja s'hi ha estat (no canvi continu de Ti)
    ports_stim = iL(i);

    u_stim_norm = utility_stim;
    d_stim_norm = distance_stim;
    p_stim_norm = ports_stim;
 
    
    if((1.0 - ((double)robotsAssigned(i) / (double)missions[i].ports.size())) < 0.0000000001) // if task has not free ports prob has to be 0
    {
  
      prob = 0.0;

    } else {

      switch(agregation_type)  // TOTAL STIM CALCULATION (agregation type)
      {
        case(0): // t-norma mínimo -> A = min{x, y, z}
          stim = std::min({u_stim_norm, d_stim_norm, p_stim_norm});
          break;

        case(1): // t-norma producto -> A = x · y · z
          stim = u_stim_norm * d_stim_norm * p_stim_norm;
          break;

        case(2): // OWA modified -> A = w1 · b + w2 · c + w3 · d (si b == 0 -> A = 0)  // b = max{x, y, z}, c = min{x, y, z}, d = (x OR y OR z) != (b AND c)
          if((std::min({u_stim_norm, d_stim_norm, p_stim_norm})) == 0.0)
          {

            stim = 0.0;

          } else {
            //printf("%f · %f   +   %f · %f   +   %f · %f\n", w1, std::min({u_stim_norm, d_stim_norm, p_stim_norm}), w2, std::max({u_stim_norm, d_stim_norm, p_stim_norm}), w3, middleValue(u_stim_norm, d_stim_norm, p_stim_norm));
            stim = w1 * (std::max({u_stim_norm, d_stim_norm, p_stim_norm}))   +   w2 * std::min({u_stim_norm, d_stim_norm, p_stim_norm})   +   w3 * middleValue(u_stim_norm, d_stim_norm, p_stim_norm);
            //printf("MiddleValue: %20.16f\n", middleValue(u_stim_norm, d_stim_norm, p_stim_norm));
          }
          break;


        default:
          stim = 0.0;
          break;
      }

      prob = stim;

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

  /*printf("KOBUKI_%i:\n", kobuki_id);
  for(int i = 0 ; i < probabilities.size() ; i++)
  {
    printf("ID: %i , D_STIM: %f, U_STIM: %f, P_STIM: %f, STIMULUS: %25.24f\n", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].p_stimulus, probabilities[i].probability);
  }*/

}
