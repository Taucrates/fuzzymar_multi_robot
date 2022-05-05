/**
 * @Author: Toni Tauler
 * @File: stimulus.cpp
 * @Description: TODO
 * @Date: April 2022
 */


#include <fuzzymar_multi_robot/stimulus.h>


float distance(float x1, float y1, float x2, float y2)
{
  float dist = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  return dist;
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

    printf("Numero de puertos: %i \n", num_ports);
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
      if(has_worked) // when robot has worked in some task max_dist is just calculed using tasks location (for dynamic_max == true)
      {

        dist = distance(missions[i].x, missions[i].y, missions[j].x, missions[j].y);

      } else {

        if(first_aux){
          
          dist = distance(current_position.first, current_position.second, missions[j].x, missions[j].y);
          
        } else {
          
          dist = distance(missions[i].x, missions[i].y, missions[j].x, missions[j].y);
          
        }

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

float getMaxDistVAR()
{
  float aux_dist = 0.0;
  float dist = 0.0;

  for(int i = 0 ; i < missions.size()  ; i++)
  {
    
    for (int j = 0 ; j < missions.size() ; j++)
    {
     
      dist = distance(current_position.first, current_position.second, missions[j].x, missions[j].y);

      if(aux_dist < dist)
      {
        aux_dist = dist;
      }
    }
    
  }
  
  return aux_dist;
}

int robotsAssigned(int task)
{
  int num_robots = 0;

  for(int i = 0 ; i < missions[task].ports.size() ; i++)
  {
    if(missions[task].ports[i].id_kobuki != 0 && missions[task].ports[i].id_kobuki != kobuki_id)
    {
      num_robots++;
    }
  }

  return num_robots;
}

float getNeededTime(int task, ros::Time mission_time) // fTj
{
  float needed_time = 0.0;

  needed_time = (ros::Time::now().toSec() - mission_time.toSec()) + (missions[task].weight / (robotsAssigned(task) + 1));

  return needed_time;
}

float getUtilityDD(int id_task, ros::Time mission_time, float utility_max)
{
  float utility = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    if(missions[i].id_task == id_task)
    {
      if(missions[i].deadline >= getNeededTime(i, mission_time))
      {

        return utility = missions[i].utility;

      } else {

        return utility = missions[i].utility * ((UDD_factor * missions[i].deadline)/((getNeededTime(i, mission_time) - missions[i].deadline) + UDD_factor * missions[i].deadline));

      }
    } 
  }

  if(dynamic_max)
    {
      return getUmaxDL(mission_time, utility_max);

    } else {

      return utility_max;
    }
  
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

float getUmaxDL(ros::Time mission_time, float utility_max)
{
  float u_max = 0.0;

  for(int i = 0 ; i < missions.size() ; i++)
  {
    if(getUtilityDD(missions[i].id_task, mission_time, utility_max) > u_max)
    {
      u_max = getUtilityDD(missions[i].id_task, mission_time, utility_max);
    }
  }
  
  return u_max;
}

/*********************************************************************************************************************************************************
********************************************************************     STIMULUS     ********************************************************************
*********************************************************************************************************************************************************/

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

float getUtility_w_DeadlineStimulus(int task, ros::Time mission_time, float utility_max)
{
  float stim = 0.0;
  int task_aux = task-1;

  // ******************  MAY I HAVE TO CALCULATE Ui AS getUtilityDD(current_goal.id_task) *******************
  //printf("Task %i Ui: %f , Uj: %f , Ui-Uj = %f ||", missions[task_aux].id_task, getUtilityDD(current_goal.id_task), getUtilityDD(missions[task_aux].id_task), getUtilityDD(current_goal.id_task) - getUtilityDD(missions[task_aux].id_task));
  if(getUtilityDD(current_goal.id_task, mission_time, utility_max) - getUtilityDD(missions[task_aux].id_task, mission_time, utility_max) > 0.0)  // Ui > Uj
  {
    //printf("NORMAL  Task_%i --> U: %f\n", missions[task_aux].id_task, (current_goal.utility - getUtilityDD(missions[task_aux].id_task)));
    return stim = getUtilityDD(current_goal.id_task, mission_time, utility_max) - getUtilityDD(missions[task_aux].id_task, mission_time, utility_max);
    
  } else {                                                                   // Ui < Uj --> U = 0.0 
    
    return stim = 0.0;
  }

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

/*********************************************************************************************************************************************************
*****************************************************************    PROBABILITIES    ********************************************************************
*********************************************************************************************************************************************************/

void setProbabilities(float thres, float d_max, float u_max, ros::Time mission_time, float utility_max)
{
  probabilities.clear();

  uint8_t id;
  float stim, prob, norm_prob, accumul;
  float distance_stim = 0.0, d_stim_norm = 0.0;
  float utility_stim = 0.0, u_stim_norm = 0.0;
  float ports_stim = 0.0, p_stim_norm = 0.0;
  float sum_prob = 0.0, sum_norm = 0.0;

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

  /*printf("\n\n");
  printf("KOBUKI_%i:\n", kobuki_id);
  for(int i = 0 ; i < probabilities.size() ; i++)
  {
    printf("ID: %i , D_STIM: %f, U_STIM: %f, P_STIM: %f, STIMULUS: %f , PROBABILITY: %f NORMALIZED: %f ACCUMULATIVE: %f\n", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].p_stimulus, probabilities[i].stimulus,  probabilities[i].probability,  probabilities[i].norm_probability, probabilities[i].accumulative);
  }*/

} 


/*********************************************************************************************************************************************************
*****************************************************************    NEW STIMULUS    *********************************************************************
*********************************************************************************************************************************************************/

float actualTime(ros::Time mission_time)
{
  return (ros::Time::now().toSec() - mission_time.toSec());
}

float sdl_V1(int task)
{
  float stim = 0.0;

  if((actualTime(mission_time) + (missions[task].weight/workload)) > missions[task].deadline)
  {

    stim = missions[task].utility * ((0.07 * missions[task].deadline) / (((actualTime(mission_time) + (missions[task].weight/workload)) - missions[task].deadline) + (0.07 * missions[task].deadline))); 

  } else {

    stim = missions[task].utility;
  }

  return stim;
}

float sdl_V2(int task)
{
  float stim = 0.0;

  if((actualTime(mission_time) + (missions[task].weight/workload)) > missions[task].deadline)
  {

    stim = missions[task].utility * ((0.07 * missions[task].deadline) / (((actualTime(mission_time) + (missions[task].weight/workload)) - missions[task].deadline) + (0.07 * missions[task].deadline))); 

  } else {

    stim = missions[task].utility * ( missions[task].deadline / ( (missions[task].deadline - (actualTime(mission_time) + (missions[task].weight/workload))) + missions[task].deadline ) );
  }

  return stim;
}

float sds(int task, Current_goal current_task)
{
  float stim = 0.0;
  float aux = 0.0;

  // IN THE START THE ROBOT USE THE START POSITION AS Ti (to calculat the distance)
  /*if(current_task.id_task != 0)   
  {

    aux = (distance(current_task.x, current_task.y, missions[task].x, missions[task].y)) / (max_vel * missions[task].weight);

  } else {

    aux = (distance(current_position.first, current_position.second, missions[task].x, missions[task].y)) / (max_vel * missions[task].weight);

  }*/

  // WHEN ROBOT IS WORKING IS SUPOSED THE DISTANCE TO THIS TASK IS 0
  if(current_task.id_task == missions[task].id_task && working)
  {

    aux = 0.0;

  } else {

    aux = (distance(current_position.first, current_position.second, missions[task].x, missions[task].y)) / (max_vel * missions[task].weight);

  }
  
  if(aux < 1)
  {

    stim = std::max((double)(1 - aux), EPSILON);

  } else {

    stim = EPSILON;
    //printf("Kobuki_%i on task %i -> EPSILON\n", kobuki_id, missions[task].id_task);
  }

  return stim;
}

float iL(int task)
{
  float stim = 0.0;

  // TAKING ACCOUNT THE ROBOTS ASSIGNED TO THE TASK (no itself)
  stim = (1.0 - ((double)robotsAssigned(task) / (double)missions[task].ports.size()));

  // TAKING ACCOUNT THE ROBOTS WORKING IN THE TASK
  // This 'if' structure to avoid the robot take account of itself when is working in a task
  /*if(working && missions[task].id_task == task_Ti.id_task)
  {
    stim = (1.0 - (((double)missions[task].doing_task - 1) / (double)missions[task].ports.size()));
  } else {
    stim = (1.0 - ((double)missions[task].doing_task / (double)missions[task].ports.size()));
  }*/
  
  
  return stim;
}


/*********************************************************************************************************************************************************
***************************************************************    NEW PROBABILITIES    ******************************************************************
*********************************************************************************************************************************************************/

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
  double sum_prob = 0.0, sum_norm = 0.0;

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
       
    distance_stim = sds(i, current_task);   
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

  /*printf("\n\n");
  printf("KOBUKI_%i:\n", kobuki_id);
  for(int i = 0 ; i < probabilities.size() ; i++)
  {
    printf("ID: %i , D_STIM: %f, U_STIM: %f, P_STIM: %f, STIMULUS: %25.24f\n", probabilities[i].id_task, probabilities[i].d_stimulus, probabilities[i].u_stimulus, probabilities[i].p_stimulus, probabilities[i].probability);
  }*/

} 