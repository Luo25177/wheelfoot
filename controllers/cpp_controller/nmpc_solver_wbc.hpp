#pragma once

#include <casadi/casadi.hpp>
#include <functional>
#include <map>
#include <vector>
#include <string>

using namespace std;
using namespace casadi;

#define balanced_state_n 6
#define leg_length_state_n 2
#define turn_state_n 2

#define balanced_control_n 2
#define leg_length_control_n 1
#define turn_control_n 2

#define balanced_predict_n 30
#define leg_length_predict_n 30
#define turn_predict_n 30

#define balanced_predict_step 0.03
#define leg_length_predict_step 0.03
#define turn_predict_step 0.03

#define balanced_weight 1
#define leg_length_weight 0.1
#define turn_weight 0.1

class NmpcSolverWbc {
  Function solver;
  map<string, DM> args;
  vector<double> res;
public:
  NmpcSolverWbc() {}
  ~NmpcSolverWbc() {}
  void setup();
  DM solve();
};


