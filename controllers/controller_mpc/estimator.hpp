#pragma once

#include "wheel_legged.hpp"
#include "low_pass_filter.hpp"

class Estimator {
public:
  Estimator(WheelLegged* robot_model, LowLevelState* low_level_state, VecInt2* contact, double dt);
  ~Estimator() = default;

  Vec8 get_x();
  void run();
  void measure();

private:
  void init_system();

  Vec14 x_hat;
  Vec6 u;
  Vec14 y;
  Vec14 y_hat;
  Mat14x14 A;
  Mat14x6 B;
  Mat14x14 C;

  Mat14x14 P; // 估计值协方差
  Mat14x14 P_priori;
  Mat14x14 Q; // 观测噪声协方差
  Mat14x14 R; // 测量噪声协方差
  Mat6x6 C_u; // 输入值的协方差

  // 过程参数
  Mat14x14 S;
  PartialPivLU<Matrix<double, 14, 14>> S_lu;
  Vec14 S_y;
  Mat14x14 S_C;
  Mat14x14 S_R;
  Mat14x14 ST_C;
  Mat14x14 I_KC;

  double dt;

  Vec3 g;

  LowLevelState* low_level_state;
  WheelLegged* robot_model;
  VecInt2* contact;

  LowPassFilter* vx_filter;
  LowPassFilter* w_filter;

  AvgCov* R_check;
  AvgCov* u_check;

  string est_name;
};


