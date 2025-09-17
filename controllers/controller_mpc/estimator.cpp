#include "estimator.hpp"
  
Estimator::Estimator(WheelLegged* robot_model, LowLevelState* low_level_state, VecInt2* contact, double dt)
  : robot_model(robot_model), low_level_state(low_level_state), contact(contact), dt(dt) {
  est_name = "current";
}

void Estimator::init_system() {
  g << 0, 0, -9.81;
}

Vec8 Estimator::get_x() {
  Vec8 x;
  return x;
}

void Estimator::run() {
  x_hat = A * x_hat + B * u;
  y_hat = C * x_hat;
  //y << feet_pos_to_body, feet_vel_to_body, feet_h;

  P_priori = A * P * A.transpose() + Q;
  S = R + C * P_priori * C.transpose();
  S_lu = S.lu();
  S_y = S_lu.solve(y - y_hat);
  S_C = S_lu.solve(C);
  S_R = S_lu.solve(R);
  ST_C = (S.transpose()).lu().solve(C);
  I_KC = I14 - P_priori * C.transpose() * S_C;

  x_hat += P_priori * C.transpose() * S_y;
  P =
  I_KC * P_priori * I_KC.transpose() + P_priori * C.transpose() * S_R * ST_C * P_priori.transpose();
}

void Estimator::measure() {
}

