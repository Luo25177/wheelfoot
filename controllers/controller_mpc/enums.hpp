#pragma once

enum class LegIDName {
  LEG_L = 0,
  LEG_R = 1,
};

enum class FrameType {
  BODY,
  HIP,
  GLOBAL
};

enum class MotorMode {
  POSITION,
  VELOCITY,
  TORQUE
};

enum class StateSwitchCmd {
  NONE,
  PASSIVE,
  BALANCE,
};

