#pragma once

class LowPassFilter {
public:
  LowPassFilter(const double &sample_period, const double &cut_frequency);
  ~LowPassFilter() = default;
  void   add_value(const double &value);
  double get_value() const;
  void   clear();

private:
  double weight;
  double past_value;
  bool   start;
};
