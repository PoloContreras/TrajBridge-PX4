
#include "types.h"

// map user-defined to OSQP-accepted parameters
extern void canonicalize_OSQP_P();
extern void canonicalize_OSQP_q();
extern void canonicalize_OSQP_d();
extern void canonicalize_OSQP_A();
extern void canonicalize_OSQP_l();
extern void canonicalize_OSQP_u();

// retrieve user-defined objective function value
extern void retrieve_value();

// retrieve solution in terms of user-defined variables
extern void retrieve_solution();

// perform one ASA sequence to solve a problem instance
extern void solve();

// update user-defined parameter values
extern void update_P_sqrt(c_int idx, c_float val);
extern void update_Q_sqrt(c_int idx, c_float val);
extern void update_R_sqrt(c_int idx, c_float val);
extern void update_S_sqrt(c_int idx, c_float val);
extern void update_A(c_int idx, c_float val);
extern void update_B(c_int idx, c_float val);
extern void update_x_init(c_int idx, c_float val);
extern void update_u_prev(c_int idx, c_float val);
extern void update_fv_min(c_float val);
extern void update_fv_max(c_float val);
extern void update_gamma(c_float val);
extern void update_G(c_int idx, c_float val);

// update OSQP settings
extern void set_OSQP_default_settings();
extern void set_OSQP_rho(c_float rho_new);
extern void set_OSQP_max_iter(c_int max_iter_new);
extern void set_OSQP_eps_abs(c_float eps_abs_new);
extern void set_OSQP_eps_rel(c_float eps_rel_new);
extern void set_OSQP_eps_prim_inf(c_float eps_prim_inf_new);
extern void set_OSQP_eps_dual_inf(c_float eps_dual_inf_new);
extern void set_OSQP_alpha(c_float alpha_new);
extern void set_OSQP_scaled_termination(c_int scaled_termination_new);
extern void set_OSQP_check_termination(c_int check_termination_new);
extern void set_OSQP_warm_start(c_int warm_start_new);
