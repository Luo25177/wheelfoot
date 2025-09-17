#pragma once

#include <string>
#include "low_level_cmd.hpp"
#include "low_level_state.hpp"

using namespace std;

enum class FSMStateName {
  INVALID,
  PASSIVE,
  BALANCE
};

class FSMStateBase {
public:
  // TODO:
  //FSMStateBase(ControlComponent* _control_component, const FSMStateName &_fsm_state,
  //             const string &_state_name_str);

  virtual void enter() = 0;
  virtual void run() = 0;
  virtual void exit() = 0;

  virtual FSMStateName check_change() { return FSMStateName::INVALID; };

  FSMStateName fsm_state;
  string       state_name_str;

protected:
  // TODO:
  // control_component
  FSMStateName      next_state_name;

  LowLevelCmd* low_level_cmd;
  LowLevelState*   low_level_state;

  ExternalCmd ext_cmd;
};