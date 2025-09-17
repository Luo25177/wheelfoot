#pragma once

#include "leg.hpp"
#include "low_level_state.hpp"

class WheelLegged {
  Leg* legs[2];
  Vec2 vel_lim_x;
  Vec2 vel_lim_yaw;
  Vec3 calc_footend_pos(const LowLevelState &low_level_state, const LegIDName &leg_id, const FrameType &frame);
  Mat3x3 Ib;

public:
  WheelLegged();
  ~WheelLegged();
  Mat2x2 calc_t(const Mat2x2& q, const Mat3x2 &force);
  Mat3x2 calc_feetend_pos_to_body(const LowLevelState& state, const FrameType& frame);
  Mat3x2 get_jacobian(const LowLevelState& state, const LegIDName& leg_id);
};
