
#include "types.h"

// map user-defined to canonical parameters
extern void canonicalize_Canon_P();
extern void canonicalize_Canon_q();
extern void canonicalize_Canon_d();
extern void canonicalize_Canon_A();
extern void canonicalize_Canon_l();
extern void canonicalize_Canon_u();

// retrieve solution in terms of user-defined variables
extern void retrieve_solution();

// retrieve solver info
extern void retrieve_info();

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

// update solver settings
extern void set_solver_default_settings();
extern void set_solver_rho(c_float rho_new);
extern void set_solver_max_iter(c_int max_iter_new);
extern void set_solver_eps_abs(c_float eps_abs_new);
extern void set_solver_eps_rel(c_float eps_rel_new);
extern void set_solver_eps_prim_inf(c_float eps_prim_inf_new);
extern void set_solver_eps_dual_inf(c_float eps_dual_inf_new);
extern void set_solver_alpha(c_float alpha_new);
extern void set_solver_scaled_termination(c_int scaled_termination_new);
extern void set_solver_check_termination(c_int check_termination_new);
extern void set_solver_warm_start(c_int warm_start_new);
