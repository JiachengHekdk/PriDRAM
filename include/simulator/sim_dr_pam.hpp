#pragma once
#include <fstream>
#include <vector>
using namespace std;
#include <chrono>
#include <sstream>
#include "DR_PAM/algorithm/optim_dr_pam.hpp"

class Simulator {
 public:
  bool success;
  Simulator(int cf_num, bool read_cf, int num_drones, bool use_model,
            std::vector<float> noise);
  void runSimulation();

  void saveMetrics(vector<float> &c, vector<float> &v, vector<float> &w);

 private:
  std ::ofstream save_data, save_data_2, save_data_3, save_data_4, save_data_5;

  std ::string path;
  YAML ::Node params;

  int config_num;
  int VERBOSE;
  int num;
  int num_drone;
  int sim_iter;
  bool read_config;

  bool free_space, free_dynamic;

  float a_drone;
  float b_drone;
  float c_drone;

  float max_time;
  float t_plan;
  float dist_stop;
  float dt;

  bool out_space;
  float mission_time;
  bool collision_agent, collision_obstacle;
  std ::chrono ::duration<double, std::milli> total_time;

  Optim ::probData *prob_data;

  Eigen ::ArrayXXf agents_x, agents_y, agents_z;
  Eigen ::ArrayXXf agents_x_goal, agents_y_goal, agents_z_goal;
  Eigen ::ArrayXXf ori_agents_x_goal, ori_agents_y_goal, ori_agents_z_goal;
  Eigen ::ArrayXXf agents_xdot, agents_ydot, agents_zdot;
  Eigen ::ArrayXXf agents_xddot, agents_yddot, agents_zddot;
  Eigen ::ArrayXf smoothness, arc_length, dist_to_goal;
  Eigen ::ArrayXf isdynamic;
  std ::stringstream folder_name;
  std ::vector<std ::vector<float>> _init_drone;
  std ::vector<std ::vector<float>> _goal_drone;

  std ::vector<std ::vector<float>> _pos_static_obs;
  std ::vector<std ::vector<float>> _dim_static_obs;

  std ::vector<std ::vector<float>> _pos_dynamic_obs;
  std ::vector<std ::vector<float>> _dim_dynamic_obs;
  std ::vector<std ::vector<float>> _dynamic_obs_v;

  std ::vector<float> smoothness_agent, traj_length_agent, comp_time_agent,
      inter_agent_dist, agent_obs_dist;
  int num_obs, num_obs_2;
  int num_dynamic_obs;

  void shareInformation();
  void runAlgorithm();
  void checkCollision();
  void checkAtGoal();
  void checkViolation();
  void calculateDistances();
};
