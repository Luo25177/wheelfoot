#include "leg.hpp"

Vec3 Leg::calc_endpos_to_body(const Vec2& q) {
  return calc_endpos_to_hip(q) + hip_to_body;
}

Vec3 Leg::calc_endpos_to_hip(const Vec2& q) {
  Vec2 q_ = q * side_sign + init_q;
  double t1 = q_(0); 
  double t2 = q_(1) + q_(0);
  return { -thigh_len * sin(t1) - calf_len * sin(t2), 
    0, 
    -thigh_len * cos(t1) - calf_len * cos(t2)};
}
Leg::Leg(const LegIDName& id, const Vec3& hip_to_body, const double& thigh_len, const double& calf_len, const Vec2& init_q) :
  id(id), hip_to_body(hip_to_body), thigh_len(thigh_len), calf_len(calf_len), init_q(init_q) {
  if (id == LegIDName::LEG_L) side_sign = 1;
  else side_sign = -1;
}
Vec2 Leg::calc_torque(const Vec2& q, const Vec3& f) {
  return side_sign * calc_jacobian(q).transpose() * f;
}

Vec3 Leg::calc_force(const Vec2& q, const Vec2& t) {
  return side_sign * calc_jacobian(q) * t;
}

Vec3 Leg::calc_endpos_to_frame(const Vec2& q, const FrameType& frame) {
  if (frame == FrameType::HIP) return calc_endpos_to_hip(q);
  else if (frame == FrameType::BODY) return calc_endpos_to_body(q);
  return { 0, 0, 0 };
}

Mat3x2 Leg::calc_jacobian(const Vec2& q) {
// TODO:
  Mat3x2 res;
  res << 0, 0, 0, 0, 0, 0;
  return res;
}
