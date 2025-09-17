#include "io_webots.hpp"
#include "io_keyboard.hpp"

IOWebots::IOWebots() {
  robot = new Robot();

  motor[0] = new Motor("left_motor1");
  motor[1] = new Motor("left_motor2");
  motor[2] = new Motor("left_motor3");
  motor[3] = new Motor("right_motor1");
  motor[4] = new Motor("right_motor2");
  motor[5] = new Motor("right_motor3");

  encoder[0] = new PositionSensor("left_sensor1");
  encoder[1] = new PositionSensor("left_sensor2");
  encoder[2] = new PositionSensor("left_sensor3");
  encoder[3] = new PositionSensor("right_sensor1");
  encoder[4] = new PositionSensor("right_sensor2");
  encoder[5] = new PositionSensor("right_sensor3");

  imu = new InertialUnit("imu");
  accel = new Accelerometer("accel");
  gyro = new Gyro("gyro");
  timer = 0;

  for (int i = 0; i < 6; ++i) {
    motor[i]->enableTorqueFeedback(TIME_STEP);
    encoder[i]->enable(TIME_STEP);
  }
  imu->enable(TIME_STEP);
  accel->enable(TIME_STEP);
  gyro->enable(TIME_STEP);
  cmd_panel = new IOKeyBoard();
}

IOWebots::~IOWebots() {
  for (int i = 0; i < 6; ++i) {
    delete motor[i];
    delete encoder[i];
  }
  delete imu;
  delete accel;
  delete gyro;
  delete robot;
}

void IOWebots::send_recv(LowLevelState* low_level_state, LowLevelCmd* low_level_cmd) {
  run_step();
  send_cmd(low_level_cmd);
  recv(low_level_state);
  cmd_panel->run();
  low_level_state->ext_cmd = cmd_panel->get_ext_cmd();
}

double IOWebots::get_time() {
  return timer;
}

void IOWebots::send_cmd(LowLevelCmd* low_level_cmd) {
  for (int i = 0; i < 6; ++i) {
    if (low_level_cmd->motor_cmd[i].mode == MotorMode::POSITION)
      motor[i]->setPosition(low_level_cmd->motor_cmd[i].q);
    else if (low_level_cmd->motor_cmd[i].mode == MotorMode::VELOCITY)
      motor[i]->setVelocity(low_level_cmd->motor_cmd[i].dq);
    else if (low_level_cmd->motor_cmd[i].mode == MotorMode::TORQUE)
      motor[i]->setTorque(low_level_cmd->motor_cmd[i].t);
  }
}

void IOWebots::recv(LowLevelState* low_level_state) {
  for (int i = 0; i < 6; ++i) {
    double tmp_q = low_level_state->motor_state[i].q;
    low_level_state->motor_state[i].q = encoder[i]->getValue();
    low_level_state->motor_state[i].dq = (low_level_state->motor_state[i].q - tmp_q) / DELTA_TIME;
    low_level_state->motor_state[i].t = motor[i]->getTorqueFeedback();
  }
  low_level_state->sensor_state.imu_rpy << imu->getRollPitchYaw()[0], imu->getRollPitchYaw()[1],
  imu->getRollPitchYaw()[2];

  low_level_state->sensor_state.imu_quat << imu->getQuaternion()[0], imu->getQuaternion()[1],
  imu->getQuaternion()[2], imu->getQuaternion()[3];

  low_level_state->sensor_state.gyro_data << gyro->getValues()[0], gyro->getValues()[1],
  gyro->getValues()[2];

  low_level_state->sensor_state.accel_data << accel->getValues()[0], accel->getValues()[1],
  accel->getValues()[2];

}
void IOWebots::run_step() {
  assert_exit(robot->step(TIME_STEP) != -1);
  timer += DELTA_TIME;
}

