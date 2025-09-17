#include "low_pass_filter.hpp"

#include "utils.hpp"

LowPassFilter::LowPassFilter(const double &sample_period, const double &cut_frequency) {
  weight = 1.f / (1.f + 1.f / (2.f * PI * sample_period * cut_frequency));
  start = false;
}

void LowPassFilter::add_value(const double &value) {
  if (!start) {
    start = true;
    past_value = value;
  }
  past_value = weight * value + (1 - weight) * past_value;
}

double LowPassFilter::get_value() const {
  return past_value;
}

void LowPassFilter::clear() {
  start = false;
}
