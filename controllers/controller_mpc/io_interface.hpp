#pragma once

#include "low_level_cmd.hpp"
#include "low_level_state.hpp"
#include "cmd_panel.hpp"

class IOInterface {
public:
  IOInterface() = default;
  ~IOInterface() = default;
  virtual void   send_recv(LowLevelState* low_level_state, LowLevelCmd* low_level_cmd) = 0;
  virtual double get_time() = 0;
  void reset_cmd() {
    cmd_panel->reset();
  }

protected:
  CmdPanel* cmd_panel;
};

