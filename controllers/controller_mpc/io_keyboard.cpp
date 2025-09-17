#include "io_keyboard.hpp"

#include "utils.hpp"

IOKeyBoard::IOKeyBoard() {
  key_board = new Keyboard();
  key_board->enable(TIME_STEP);
  left_sensitivity = 1;
  right_sensitivity = 1;
}
void IOKeyBoard::run() {
  c = key_board->getKey();
  if (c <= 0) return;
  switch (c) {
    case 'w':case 'W':
      ext_cmd.ly = left_sensitivity;
      break;
    case 's':case 'S':
      ext_cmd.ly = -left_sensitivity;
      break;
    case 'd':case 'D':
      ext_cmd.lx = left_sensitivity;
      break;
    case 'a':case 'A':
      ext_cmd.lx = -left_sensitivity;
      break;
    case 'i':case 'I':
      ext_cmd.ry = right_sensitivity;
      break;
    case 'k':case 'K':
      ext_cmd.ry = -right_sensitivity;
      break;
    case 'l':case 'L':    
      ext_cmd.rx = right_sensitivity;
      break;
    case 'j':case 'J':
      ext_cmd.rx = -right_sensitivity;
      break;
    case '1':
      ext_cmd.cmd = StateSwitchCmd::PASSIVE;
      break;
    case '2':
      ext_cmd.cmd = StateSwitchCmd::BALANCE;
      break;
  }
}

