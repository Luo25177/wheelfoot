#pragma once

#include "cmd_panel.hpp"
#include <webots/Keyboard.hpp>

using namespace webots;

class IOKeyBoard : public CmdPanel {
public:
  IOKeyBoard();
  ~IOKeyBoard() = default;
  void run() override;

private:
  Keyboard* key_board;
  double left_sensitivity;
  double right_sensitivity;
  char c;
};
