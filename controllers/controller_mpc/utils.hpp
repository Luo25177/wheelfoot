#pragma once

#include "types.hpp"
#include <iostream>

using namespace std;

inline void assert_panic_exit(const char* filename, const int &line, const char* f,
                              const char* con) {
  cout << "\x1b[31merror: \x1b[0m"
       << "at line " << line << " of file " << filename << ": " << con << endl;
  exit(-1);
}

#define panic_exit(...) assert_panic_exit(__FILE__, __LINE__, __func__, __VA_ARGS__)
#define assert_exit(con) \
  if (!(con)) panic_exit(#con);

#define PI 3.14159265358979323846
#define TIME_STEP 2
#define DELTA_TIME 0.002

inline RotMat rpy_to_rotmat(const double &r, const double &p, const double &y) {
  RotMat mat;
  mat << cos(y) * cos(p), cos(y) * sin(p) * sin(r) - sin(y) * cos(r),
  cos(y) * sin(p) * cos(r) + sin(y) * sin(r), sin(y) * cos(p),
  sin(y) * sin(p) * sin(r) + cos(y) * cos(r), sin(y) * sin(p) * cos(r) - cos(y) * sin(r), -sin(p),
  cos(p) * sin(r), cos(p) * cos(r);
  return mat;
}

inline Vec3 rotmat_to_rpy(const RotMat &m) {
  Vec3 rpy;
  rpy(0) = atan2(m(2, 1), m(2, 2));
  rpy(1) = asin(-m(2, 0));
  rpy(2) = atan2(m(1, 0), m(0, 0));
  return rpy;
}

class AvgCov {
public:
  AvgCov(const unsigned int &_size, const std::string &_name, const bool &_avg_only = false,
         const unsigned int &_show_period = 2000, const unsigned int &_wait_count = 5000,
         const double &_zoom_factor = 10000) :
  size(_size),
  name(_name),
  avg_only(_avg_only),
  show_period(_show_period),
  wait_count(_wait_count),
  zoom_factor(_zoom_factor) {
    exp.resize(size);
    exp.setZero();
    cov.resize(size, size);
    default_weight.resize(size, size);
    default_weight.setIdentity();
    measure_count = 0;
  }

  void measure(const VecX &v) {
    ++measure_count;
    if (measure_count <= wait_count) return;
    update_avg_cov(v, measure_count - wait_count);
    if (measure_count % show_period == 0) {
      cout << "\x1b[34mavarage and covariance measure: \x1b[0m" << endl;
      cout << zoom_factor << " times of avarage: " << (zoom_factor * exp).transpose() << endl;
      if (!avg_only) cout << zoom_factor << " times of covariance: " << (zoom_factor * cov) << endl;
    }
  }

private:
  VecX         exp;
  MatX         cov;
  MatX         default_weight;
  bool         avg_only;
  unsigned int size;
  unsigned int measure_count;
  unsigned int show_period;
  unsigned int wait_count;
  double       zoom_factor;
  std::string  name;

  void update_avg_cov(const VecX &v, const double &n) {
    assert_exit(cov.rows() == cov.cols());
    assert_exit(cov.rows() == exp.rows());
    assert_exit(exp.rows() == v.rows());
    cov = cov * (n - 1) / n + (v - exp) * (v - exp).transpose() * (n - 1) / (n * n);
    exp = exp + (v - exp) / n;
  }
};

