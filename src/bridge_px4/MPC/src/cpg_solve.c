
#include "cpg_solve.h"
#include "cpg_workspace.h"
#include "workspace.h"
#include "osqp.h"
#include <time.h>

static c_int i;
static c_int j;


// update user-defined parameters
void update_P_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+0] = val;
Canon_Outdated.A = 1;
}
void update_Q_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+36] = val;
Canon_Outdated.A = 1;
}
void update_R_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+72] = val;
Canon_Outdated.A = 1;
}
void update_S_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+81] = val;
Canon_Outdated.A = 1;
}
void update_A(c_int idx, c_float val){
CPG_Params_Vec[idx+90] = val;
Canon_Outdated.A = 1;
}
void update_B(c_int idx, c_float val){
CPG_Params_Vec[idx+126] = val;
Canon_Outdated.A = 1;
}
void update_x_init(c_int idx, c_float val){
CPG_Params_Vec[idx+144] = val;
Canon_Outdated.l = 1;
Canon_Outdated.u = 1;
}
void update_u_prev(c_int idx, c_float val){
CPG_Params_Vec[idx+150] = val;
Canon_Outdated.l = 1;
Canon_Outdated.u = 1;
}
void update_fv_min(c_float val){
CPG_Params_Vec[153] = val;
Canon_Outdated.u = 1;
}
void update_fv_max(c_float val){
CPG_Params_Vec[154] = val;
Canon_Outdated.u = 1;
}
void update_gamma(c_float val){
CPG_Params_Vec[155] = val;
Canon_Outdated.A = 1;
}
void update_G(c_int idx, c_float val){
CPG_Params_Vec[idx+156] = val;
Canon_Outdated.u = 1;
}

// map user-defined to canonical parameters
void canonicalize_Canon_P(){
// reset values to zero
for(i=0; i<366; i++){
Canon_Params.P->x[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<366; i++){
for(j=Canon_P_map.p[i]; j<Canon_P_map.p[i+1]; j++){
Canon_Params.P->x[i] += Canon_P_map.x[j]*CPG_Params_Vec[Canon_P_map.i[j]];
}
}
}
void canonicalize_Canon_A(){
// reset values to zero
for(i=0; i<4521; i++){
Canon_Params.A->x[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<4521; i++){
for(j=Canon_A_map.p[i]; j<Canon_A_map.p[i+1]; j++){
Canon_Params.A->x[i] += Canon_A_map.x[j]*CPG_Params_Vec[Canon_A_map.i[j]];
}
}
}
void canonicalize_Canon_l(){
// reset values to zero
for(i=0; i<555; i++){
Canon_Params.l[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<555; i++){
for(j=Canon_l_map.p[i]; j<Canon_l_map.p[i+1]; j++){
Canon_Params.l[i] += Canon_l_map.x[j]*CPG_Params_Vec[Canon_l_map.i[j]];
}
}
}
void canonicalize_Canon_u(){
// reset values to zero
for(i=0; i<795; i++){
Canon_Params.u[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<795; i++){
for(j=Canon_u_map.p[i]; j<Canon_u_map.p[i+1]; j++){
Canon_Params.u[i] += Canon_u_map.x[j]*CPG_Params_Vec[Canon_u_map.i[j]];
}
}
}

// retrieve user-defined objective function value
void retrieve_value(){
objective_value = workspace.info->obj_val + *Canon_Params.d;
}

// retrieve solution in terms of user-defined variables
void retrieve_solution(){
}

// perform one ASA sequence to solve a problem instance
void solve(){
clock_t start_ASA = clock();
if (Canon_Outdated.A) {
canonicalize_Canon_A();
osqp_update_A(&workspace, Canon_Params.A->x, 0, 0);
}
if (Canon_Outdated.l && Canon_Outdated.u) {
canonicalize_Canon_l();
canonicalize_Canon_u();
osqp_update_bounds(&workspace, Canon_Params.l, Canon_Params.u);
} else if (Canon_Outdated.l) {
canonicalize_Canon_l();
osqp_update_lower_bound(&workspace, Canon_Params.l);
} else if (Canon_Outdated.u) {
canonicalize_Canon_u();
osqp_update_upper_bound(&workspace, Canon_Params.u);
}
clock_t start = clock();
osqp_solve(&workspace);
clock_t end = clock();
CPG_Result.osqp_solve_time = (c_float)(end - start) / CLOCKS_PER_SEC;
retrieve_value();
retrieve_solution();
Canon_Outdated.P = 0;
Canon_Outdated.q = 0;
Canon_Outdated.d = 0;
Canon_Outdated.A = 0;
Canon_Outdated.l = 0;
Canon_Outdated.u = 0;
clock_t end_ASA = clock();
CPG_Result.cpg_solve_time = (c_float)(end_ASA - start_ASA) / CLOCKS_PER_SEC;
}

// update solver settings
void set_solver_default_settings(){
osqp_set_default_settings(&settings);
}
void set_solver_rho(c_float rho_new){
osqp_update_rho(&workspace, rho_new);
}
void set_solver_max_iter(c_int max_iter_new){
osqp_update_max_iter(&workspace, max_iter_new);
}
void set_solver_eps_abs(c_float eps_abs_new){
osqp_update_eps_abs(&workspace, eps_abs_new);
}
void set_solver_eps_rel(c_float eps_rel_new){
osqp_update_eps_rel(&workspace, eps_rel_new);
}
void set_solver_eps_prim_inf(c_float eps_prim_inf_new){
osqp_update_eps_prim_inf(&workspace, eps_prim_inf_new);
}
void set_solver_eps_dual_inf(c_float eps_dual_inf_new){
osqp_update_eps_dual_inf(&workspace, eps_dual_inf_new);
}
void set_solver_alpha(c_float alpha_new){
osqp_update_alpha(&workspace, alpha_new);
}
void set_solver_scaled_termination(c_int scaled_termination_new){
osqp_update_scaled_termination(&workspace, scaled_termination_new);
}
void set_solver_check_termination(c_int check_termination_new){
osqp_update_check_termination(&workspace, check_termination_new);
}
void set_solver_warm_start(c_int warm_start_new){
osqp_update_warm_start(&workspace, warm_start_new);
}
