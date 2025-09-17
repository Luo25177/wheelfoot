#pragma once

#include "fsm_state_base.hpp"

class PassiveState : public FSMStateBase {
public:
  // TODO:
  // PassiveState()
  ~PassiveState() = default;
  void         enter() override;
  void         run() override;
  void         exit() override;
  FSMStateName check_change() override;

private:
  double kd;
};
