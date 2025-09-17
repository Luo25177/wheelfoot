#include "wheel_legged.hpp"

WheelLegged::WheelLegged() {
  legs[0] = new Leg(LegIDName::LEG_L, { 0, 0.16, 0 }, 0.18, 0.18, {1 * PI / 3, -2 * PI / 3});
  legs[1] = new Leg(LegIDName::LEG_R, { 0, -0.16, 0 }, 0.18, 0.18, {1 * PI / 3, -2 * PI / 3});
  vel_lim_x << -0.4, 0.4;
  vel_lim_yaw << -0.5, 0.5;
  Ib = Vec3(0.13135586, 0.07941512, 0.13135586).asDiagonal();
}
WheelLegged::~WheelLegged() {
  delete legs[0];
  delete legs[1];
}
Vec3 WheelLegged::calc_footend_pos(const LowLevelState& low_level_state, const LegIDName& leg_id, const FrameType& frame) {
  int i = static_cast<int>(leg_id);
  Vec3 q = low_level_state.get_q().col(i);
  return legs[i]->calc_endpos_to_frame(q, frame);
}
Mat2x2 WheelLegged::calc_t(const Mat2x2& q, const Mat3x2& force) {
  Mat2x2 t;
  t.col(0) = legs[0]->calc_torque(q.col(0), force.col(0));
  t.col(1) = legs[1]->calc_torque(q.col(1), force.col(1));
  return t;
}
Mat3x2 WheelLegged::calc_feetend_pos_to_body(const LowLevelState& state, const FrameType& frame) {
  Mat3x2 pos;
  if (frame == FrameType::GLOBAL) {
    pos.col(0) = calc_footend_pos(state, static_cast<LegIDName>(0), FrameType::BODY);
    pos.col(1) = calc_footend_pos(state, static_cast<LegIDName>(1), FrameType::BODY);
    pos = state.get_rotmat() * pos;
  } else {
    pos.col(0) = calc_footend_pos(state, static_cast<LegIDName>(0), frame);
    pos.col(1) = calc_footend_pos(state, static_cast<LegIDName>(1), frame);
  }
  return pos;
}
Mat3x2 WheelLegged::get_jacobian(const LowLevelState& state, const LegIDName& leg_id) {
  int i = static_cast<int>(leg_id);
  return legs[i]->calc_jacobian(state.get_q().col(i));
}


