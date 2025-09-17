#pragma once

#include "io_interface.hpp"
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

using namespace webots;

class IOWebots : public IOInterface {
public:
  IOWebots();
  ~IOWebots();
  void   send_recv(LowLevelState* low_level_state, LowLevelCmd* low_level_cmd) override;
  double get_time() override;

private:
  Motor*          motor[6];
  PositionSensor* encoder[6];
  InertialUnit*   imu;
  Accelerometer*  accel;
  Gyro*           gyro;
  Robot*          robot;
  double          timer;

  void send_cmd(LowLevelCmd* low_level_cmd);
  void recv(LowLevelState* low_level_state);
  void run_step();
};

