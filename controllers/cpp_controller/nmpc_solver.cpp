#include "nmpc_solver.hpp"

nmpc_solver_typedef::nmpc_solver_typedef() {
}
nmpc_solver_typedef::~nmpc_solver_typedef() {
}

void nmpc_solver_typedef::setup(
  const int& _predict_n,
  const double& _predict_step,
  const int& _state_n,
  const int& _control_n,
  Function _dynamics,
  const DM& _Q, 
  const DM& _R, 
  const DM& _u_min, 
  const DM& _u_max) {

  state_n = _state_n;
  control_n = _control_n;
  predict_n = _predict_n;
  predict_step = _predict_step;

  SX J = 0;
  int i;
  SX U = SX::sym("u", control_n, predict_n);
  SX X = SX::sym("x", state_n, predict_n + 1);
  SX x0 = SX::sym("x0", state_n);
  SX x_ref = SX::sym("x_ref", state_n);
  SX l = SX::sym("l");
  SX mask = SX::sym("mask", state_n, state_n);

  X(Slice(), 0) = x0;

  for (i = 0; i < predict_n; ++i) {
    SX k1 = _dynamics(SXDict{ {"x", X(Slice(), i)}, {"u", U(Slice(), i)}, {"l", l} }).at("xdot");
    SX k2 = _dynamics(SXDict{ {"x", X(Slice(), i) + 0.5 * predict_step * k1}, {"u", U(Slice(), i)}, {"l", l}  }).at("xdot");
    SX k3 = _dynamics(SXDict{ {"x", X(Slice(), i) + 0.5 * predict_step * k2}, {"u", U(Slice(), i)}, {"l", l}  }).at("xdot");
    SX k4 = _dynamics(SXDict{ {"x", X(Slice(), i) + 0.5 * predict_step * k3}, {"u", U(Slice(), i)}, {"l", l}  }).at("xdot");
    X(Slice(), i + 1) = X(Slice(), i) + (predict_step / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
  }

  for (i = 0; i < predict_n; ++i)
    J += SX::dot((X(Slice(), i) - x_ref), SX::mtimes(_Q, (X(Slice(), i) - x_ref))) + 
      SX::dot(U(Slice(), i), SX::mtimes(_R, U(Slice(), i)));
  J += SX::mtimes((X(Slice(), i) - x_ref).T(), SX::mtimes(_Q, (X(Slice(), i) - x_ref))); 

  SXDict nlp = {
    {"x", reshape(U, -1, 1)},
    {"f", J},
    {"p", vertcat(x0, x_ref, l)}
  };

  Dict opts;
  opts["ipopt.max_iter"] = 5000;
  opts["print_time"] = 0;
  opts["ipopt.print_level"] = 0;
  opts["ipopt.acceptable_tol"] =  1e-6;
  opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

  solver = nlpsol("solver", "ipopt", nlp, opts);

  SX lbu = repmat(_u_min, predict_n, 1);
  SX ubu = repmat(_u_max, predict_n, 1);

  args["lbx"] = lbu;
  args["ubx"] = ubu;
}

DM nmpc_solver_typedef::solve(const DM &x0, const DM &x_ref, const DM &l) {
  args["p"] = vertcat(x0, x_ref, l);
  args["x0"] = DM::zeros(control_n * predict_n, 1);
  DMDict res = solver(args);

  return reshape(res.at("x"), control_n, predict_n);
}

