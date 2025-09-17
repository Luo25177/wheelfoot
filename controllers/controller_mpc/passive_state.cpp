#include "passive_state.hpp"

void PassiveState::enter() {
  for (int i = 0; i < 12; ++i) low_level_cmd->motor_cmd[i].mode = MotorMode::TORQUE;
}
void PassiveState::run() {
  for (int i = 0; i < 12; ++i)
    low_level_cmd->motor_cmd[i].t = -kd * low_level_state->motor_state[i].dq;
}
void PassiveState::exit() {
}
 
FSMStateName PassiveState::check_change() {
  if (low_level_state->ext_cmd.cmd == StateSwitchCmd::BALANCE)
    return FSMStateName::BALANCE;
  return FSMStateName::PASSIVE;
}

