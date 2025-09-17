#pragma once

#include "types.hpp"
#include "enums.hpp"

class CmdPanel {
public:
  CmdPanel() {}
  ~CmdPanel() {};
  ExternalCmd get_ext_cmd() { return ext_cmd; }
  void set_passive() { ext_cmd.cmd = StateSwitchCmd::PASSIVE; }
  void reset() { ext_cmd.reset(); }
  virtual void run() = 0;

protected:
  ExternalCmd ext_cmd;
};

