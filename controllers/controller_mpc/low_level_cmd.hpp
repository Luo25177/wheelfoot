#pragma once

#include "enums.hpp"
#include "types.hpp"

struct MotorCmd {
  MotorMode mode;
  double    q;
  double    dq;
  double    t;

  MotorCmd() {
    q = 0;
    dq = 0;
    t = 0;
    mode = MotorMode::POSITION;
  }
};

class LowLevelCmd{
public:
  MotorCmd motor_cmd[6];

  void set_q(const Vec6& q) {
    for (int i = 0; i < 6; ++i)
      motor_cmd[i].q = q(i);
  }
  void set_q(const LegIDName& id, const Vec3& q) {
    motor_cmd[(int) id * 3 + 0].q = q(0);
    motor_cmd[(int) id * 3 + 1].q = q(1);
    motor_cmd[(int) id * 3 + 2].q = q(2);
  }
  void set_dq(const Vec6& dq) {
    for (int i = 0; i < 6; ++i)
      motor_cmd[i].dq = dq(i);
  }
  void set_dq(const LegIDName& id, const Vec3& dq) {
    motor_cmd[(int) id * 3 + 0].dq = dq(0);
    motor_cmd[(int) id * 3 + 1].dq = dq(1);
    motor_cmd[(int) id * 3 + 2].dq = dq(2);
  }
  void set_t(const Vec6& t) {
    for (int i = 0; i < 6; ++i)
      motor_cmd[i].t = t(i);
  }
  void set_t(const LegIDName& id, const Vec6& t) {
    motor_cmd[(int) id * 3 + 0].t = t(0);
    motor_cmd[(int) id * 3 + 1].t = t(1);
    motor_cmd[(int) id * 3 + 2].t = t(2);
  }
};
