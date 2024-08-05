#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
using namespace std;
#include <ros/package.h>
#include <ros/ros.h>

#include "DR_PAM/simulator/sim_dr_pam.hpp"

int main() {
  std ::string path = ros ::package::getPath("DR_PAM");
  YAML ::Node params = YAML ::LoadFile(path + "/params/config_sim.yaml");

  int start_config = params["start_config"].as<int>();
  int end_config = params["end_config"].as<int>();
  bool read_config = params["read_config"].as<bool>();
  bool use_model = params["use_model"].as<bool>();
  std ::vector<float> noise = params["noise"].as<std::vector<float>>();
  std ::vector<float> num_drones =
      params["num_drones"].as<std::vector<float>>();

  for (int j = 0; j < num_drones.size(); j++) {
    if (read_config)
      ROS_INFO_STREAM("Agent size = " << num_drones[j]
                                      << " Configuration numbers = "
                                      << end_config - start_config + 1);
    int success_trials = 0;
    std::vector<float> compu_time1;
    std::vector<float> flight_time2;
    std::vector<float> smooth1;
    for (int i = start_config; i < end_config; i++) {
      Simulator sim =
          Simulator(i, read_config, num_drones[j], use_model, noise);
      sim.runSimulation();

      sim.saveMetrics(compu_time1, flight_time2, smooth1);

      if (sim.success) success_trials += 1;
      // if(!read_config)
      //   break;
      // ROS_INFO_STREAM("Success Trials = " << success_trials << " out of " <<
      // i+1);
      ROS_INFO_STREAM("Success Trials = " << success_trials << " out of "
                                          << end_config);
    }
    // for(int i =0;i<compu_time1.size();i++)
    // {
    //     cout<<compu_time1[i]<<endl;
    //}
    double sum = 0;
    for (const float value : compu_time1) {
      sum += value;
    }
    float mean = (float)sum / compu_time1.size();
    cout << mean << endl;
    double diff1 = 0;
    for (const float value : compu_time1) {
      double diff = value - mean;
      diff1 += diff * diff;
    }
    cout << (double)diff1 / compu_time1.size() << endl;
    //***********************************************
    double sum1 = 0;
    for (const float value : flight_time2) {
      sum1 += value;
    }
    float mean1 = (float)sum1 / flight_time2.size();
    cout << mean1 << endl;
    double difff1 = 0;
    for (const float value : flight_time2) {
      double difff = value - mean1;
      difff1 += difff * difff;
    }
    cout << (double)difff1 / flight_time2.size() << endl;
    //***********************************************
    double sum2 = 0;
    for (const float value : smooth1) {
      sum2 += value;
    }
    float mean2 = (float)sum2 / smooth1.size();
    cout << mean2 << endl;
    double diffff1 = 0;
    for (const float value : smooth1) {
      double diffff = value - mean2;
      diffff1 += diffff * diffff;
    }
    cout << (double)diffff1 / smooth1.size() << endl;
    // ROS_INFO_STREAM("Success Trials = " << success_trials << " out of " <<
    // end_config);
    if (!read_config) break;
  }
  return 0;
}
