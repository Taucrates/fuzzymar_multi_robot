/**
 * @Author: Toni Tauler
 * @File: defs.h
 * @Description: TODO
 * @Date: April 2022
 */



#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <iomanip>
#include <random>
#include <chrono>
#include <thread>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fuzzymar_multi_robot/task_w_ports.h>
#include <fuzzymar_multi_robot/task_w_portsArray.h>
#include <fuzzymar_multi_robot/action_task_w_ports.h>
#include <fuzzymar_multi_robot/taskObjective.h>
#include <fuzzymar_multi_robot/robotParameters.h>


#pragma once

#define EPSILON 2.22044604925031 / 10000000000000000

// STRUCTS

struct Current_goal {
  uint8_t id_task;
  float x;
  float y;
  float yaw;
  float utility;
};

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
  double d_stimulus;
  double u_stimulus;
  double p_stimulus;
  double stimulus;
  double probability;
  double norm_probability;
  double accumulative;
};


// GLOBAL VARIABLES
int kobuki_id;

  // POSSIBILISTIC
bool possibilistic = true; // default -> possibilistic if false -> deterministic
bool dynamic_max = false; // if true d_max && u_max will be variable
bool has_worked = false; // robot has worked

float alpha_utility = 2.0; // default 2.0
float beta_distance = 3.0; // default 3.0
float gamma_ports = 1.5;   // default 1.5

float UDD_factor = 0.07;   // default 0.07
float max_vel = 0.2;       // default 0.2
float max_vel_aux = 0.2;

  // DETERMINISTIC

int sdl_method = 1;        // default method 1 -> 1: method 1, 2: method 2
int agregation_type = 0;   // default t-norma mínimo -> 0: t-norma mínimo  1: t-norma producto 2: OWA-modified 

float w1 = 1.0/3.0;        // default 1/3
float w2 = 1.0/3.0;        // default 1/3
float w3 = 1.0/3.0;        // default 1/3

float workload = 1.0;      // weight reduced per second

float calc_stim_time = 1.0;     // how often the robot calculate the stimulus
bool calc_stim_way = true;      // true if the robot have to calculate running through tasks, false otherwise

// GENERAL

ros::Time mission_time; // the time at wich the mission starts

std::vector<Task> missions;
std::pair<float, float> current_position; // current position of the robot
Current_goal current_goal; // current task declared as objective {task_id, x, y, yaw}
Current_goal task_Ti;      // current task where robot has already arrived
std::vector<Probability> probabilities; // vector where probabilities are saved

// FLAG
//bool may_stuck = false;   // indicates if the robot is stopped (maybe stucked)
bool sure_stuck = false;  // indicates if the robot is stucked (may_stuck && !Working)
int count_stuck = 0;