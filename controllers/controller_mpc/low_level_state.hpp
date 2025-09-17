#pragma once

#include "enums.hpp"
#include "types.hpp"
#include "utils.hpp"

struct MotorState {
  MotorMode mode;
  double    q;
  double    dq;
  double    t;

  MotorState() {
    q = 0;
    dq = 0;
    t = 0;
    mode = MotorMode::POSITION;
  }
};

struct SensorState {
  Vec3 imu_rpy;
  Quat imu_quat;
  Vec3 gyro_data;
  Vec3 accel_data;

  SensorState() {
    imu_rpy = { 0, 0, 0 };
    imu_quat = { 0, 0, 0, 0 };
    gyro_data = { 0, 0, 0 };
    accel_data = { 0, 0, 0 };
  }
};

class LowLevelState {
public:
  MotorState motor_state[6];
  SensorState sensor_state;
  ExternalCmd ext_cmd;

  LowLevelState() { }

  Mat3x2 get_q() const {
    Mat3x2 q;
    for (int i = 0; i < 2; ++i) {
      q(0, i) = motor_state[i * 3 + 0].q;
      q(1, i) = motor_state[i * 3 + 1].q;
      q(2, i) = motor_state[i * 3 + 2].q;
    }
    return q;
  }

  Vec3 get_q(const LegIDName& id) const {
    Vec3 q;
    q(0) = motor_state[(int) id * 3 + 0].q;
    q(1) = motor_state[(int) id * 3 + 1].q;
    q(2) = motor_state[(int) id * 3 + 2].q;
    return q;
  }

  Mat3x2 get_dq() const {
    Mat3x2 dq;
    for (int i = 0; i < 2; ++i) {
      dq(0, i) = motor_state[i * 3 + 0].dq;
      dq(1, i) = motor_state[i * 3 + 1].dq;
      dq(2, i) = motor_state[i * 3 + 2].dq;
    }
    return dq;
  }

  Vec3 get_dq(const LegIDName& id) const {
    Vec3 dq;
    dq(0) = motor_state[(int) id * 3 + 0].dq;
    dq(1) = motor_state[(int) id * 3 + 1].dq;
    dq(2) = motor_state[(int) id * 3 + 2].dq;
    return dq;
  }

  Quat get_quat() const { return sensor_state.imu_quat; }

  Vec3 get_rpy() const { return sensor_state.imu_rpy; }

  Vec3 get_gyro() const { return sensor_state.gyro_data; }

  Vec3 get_accel() const { return sensor_state.accel_data; }

  RotMat get_rotmat() const { return rpy_to_rotmat(sensor_state.imu_rpy[0], sensor_state.imu_rpy[1], sensor_state.imu_rpy[2]); }

  Vec3 get_accel_global() const { return get_rotmat() * get_accel(); }

  Vec3 get_gyro_global() const { return get_rotmat() * get_gyro(); }
};
