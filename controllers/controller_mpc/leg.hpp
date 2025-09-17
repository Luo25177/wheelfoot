#pragma once

#include "enums.hpp"
#include "types.hpp"

class Leg {
  LegIDName id;
  Vec3 hip_to_body;
  const double thigh_len;
  const double calf_len;
  const Vec2 init_q;
  double side_sign;
  
  Vec3 calc_endpos_to_body(const Vec2& q);
  Vec3 calc_endpos_to_hip(const Vec2& q);

public:
  Leg(const LegIDName &id, const Vec3& hip_to_body, const double &thigh_len, const double& calf_len, const Vec2& init_q);
  Vec2 calc_torque(const Vec2& q, const Vec3& f);
  Vec3 calc_force(const Vec2& q, const Vec2& t);
  Vec3 calc_endpos_to_frame(const Vec2 &q, const FrameType& frame);
  Mat3x2 calc_jacobian(const Vec2 &q);
};
