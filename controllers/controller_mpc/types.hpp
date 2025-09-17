#pragma once

#include <Eigen>
#include "enums.hpp"

using namespace Eigen;

using Vec2 = Matrix<double, 2, 1>;
using Vec3 = Matrix<double, 3, 1>;
using Vec6 = Matrix<double, 6, 1>;
using Vec8 = Matrix<double, 8, 1>;
using Vec12 = Matrix<double, 12, 1>;
using Vec14 = Matrix<double, 14, 1>;
using VecX = Matrix<double, Dynamic, 1>;

using VecInt2 = Matrix<int, 2, 1>;

using Mat2x2 = Matrix<double, 2, 2>;
using Mat3x2 = Matrix<double, 3, 2>;
using Mat3x3 = Matrix<double, 3, 3>;
using Mat6x6 = Matrix<double, 6, 6>;
using Mat12x2 = Matrix<double, 12, 2>;
using Mat12x12 = Matrix<double, 12, 12>;
using Mat14x6 = Matrix<double, 14, 6>;
using Mat14x14 = Matrix<double, 14, 14>;
using MatX = Matrix<double, Dynamic, Dynamic>;

using Quat = Matrix<double, 4, 1>;
using RotMat = Matrix<double, 3, 3>;

#define I14 MatrixXd::Identity(14, 14);

struct ExternalCmd {
  float lx;
  float ly;
  float rx;
  float ry;
  StateSwitchCmd cmd;

  ExternalCmd () {
    reset();
  }
  void reset() {
    lx = 0;
    ly = 0;
    rx = 0;
    ry = 0;
    cmd = StateSwitchCmd::NONE;
  }
};

