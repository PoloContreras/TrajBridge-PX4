
#include "cpg_workspace.h"
#include "workspace.h"
#include "osqp.h"
#include <time.h>

static c_int i;
static c_int j;

// update user-defined parameters
void update_P_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+0] = val;
OSQP_Outdated.A = 1;
}
void update_Q_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+36] = val;
OSQP_Outdated.A = 1;
}
void update_R_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+72] = val;
OSQP_Outdated.A = 1;
}
void update_S_sqrt(c_int idx, c_float val){
CPG_Params_Vec[idx+81] = val;
OSQP_Outdated.A = 1;
}
void update_A(c_int idx, c_float val){
CPG_Params_Vec[idx+90] = val;
OSQP_Outdated.A = 1;
}
void update_B(c_int idx, c_float val){
CPG_Params_Vec[idx+126] = val;
OSQP_Outdated.A = 1;
}
void update_x_init(c_int idx, c_float val){
CPG_Params_Vec[idx+144] = val;
OSQP_Outdated.l = 1;
OSQP_Outdated.u = 1;
}
void update_u_prev(c_int idx, c_float val){
CPG_Params_Vec[idx+150] = val;
OSQP_Outdated.l = 1;
OSQP_Outdated.u = 1;
}
void update_fv_min(c_float val){
CPG_Params_Vec[153] = val;
OSQP_Outdated.u = 1;
}
void update_fv_max(c_float val){
CPG_Params_Vec[154] = val;
OSQP_Outdated.u = 1;
}
void update_gamma(c_float val){
CPG_Params_Vec[155] = val;
OSQP_Outdated.A = 1;
}
void update_G(c_int idx, c_float val){
CPG_Params_Vec[idx+156] = val;
OSQP_Outdated.u = 1;
}

// map user-defined to OSQP-accepted parameters
void canonicalize_OSQP_P(){
// reset values to zero
for(i=0; i<366; i++){
OSQP_Params.P->x[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<366; i++){
for(j=OSQP_P_map.p[i]; j<OSQP_P_map.p[i+1]; j++){
OSQP_Params.P->x[i] += OSQP_P_map.x[j]*CPG_Params_Vec[OSQP_P_map.i[j]];
}
}
}
void canonicalize_OSQP_A(){
// reset values to zero
for(i=0; i<4521; i++){
OSQP_Params.A->x[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<4521; i++){
for(j=OSQP_A_map.p[i]; j<OSQP_A_map.p[i+1]; j++){
OSQP_Params.A->x[i] += OSQP_A_map.x[j]*CPG_Params_Vec[OSQP_A_map.i[j]];
}
}
}
void canonicalize_OSQP_l(){
// reset values to zero
for(i=0; i<555; i++){
OSQP_Params.l[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<555; i++){
for(j=OSQP_l_map.p[i]; j<OSQP_l_map.p[i+1]; j++){
OSQP_Params.l[i] += OSQP_l_map.x[j]*CPG_Params_Vec[OSQP_l_map.i[j]];
}
}
}
void canonicalize_OSQP_u(){
// reset values to zero
for(i=0; i<795; i++){
OSQP_Params.u[i] = 0;
}
// compute sparse matrix multiplication
for(i=0; i<795; i++){
for(j=OSQP_u_map.p[i]; j<OSQP_u_map.p[i+1]; j++){
OSQP_Params.u[i] += OSQP_u_map.x[j]*CPG_Params_Vec[OSQP_u_map.i[j]];
}
}
}

// retrieve user-defined objective function value
void retrieve_value(){
objective_value = workspace.info->obj_val + *OSQP_Params.d;
}

// retrieve solution in terms of user-defined variables
void retrieve_solution(){
}

// perform one ASA sequence to solve a problem instance
void solve(){
if (OSQP_Outdated.P && OSQP_Outdated.A) {
canonicalize_OSQP_P();
canonicalize_OSQP_A();
osqp_update_P_A(&workspace, OSQP_Params.P->x, 0, 0, OSQP_Params.A->x, 0, 0);
} else if (OSQP_Outdated.P) {
canonicalize_OSQP_P();
osqp_update_P(&workspace, OSQP_Params.P->x, 0, 0);
} else if (OSQP_Outdated.A) {
canonicalize_OSQP_A();
osqp_update_A(&workspace, OSQP_Params.A->x, 0, 0);
}
if (OSQP_Outdated.l && OSQP_Outdated.u) {
canonicalize_OSQP_l();
canonicalize_OSQP_u();
osqp_update_bounds(&workspace, OSQP_Params.l, OSQP_Params.u);
} else if (OSQP_Outdated.l) {
canonicalize_OSQP_l();
osqp_update_lower_bound(&workspace, OSQP_Params.l);
} else if (OSQP_Outdated.u) {
canonicalize_OSQP_u();
osqp_update_upper_bound(&workspace, OSQP_Params.u);
}
clock_t start = clock();
osqp_solve(&workspace);
clock_t end = clock();
CPG_Result.osqp_solve_time = (c_float)(end - start) / CLOCKS_PER_SEC;
retrieve_value();
retrieve_solution();
OSQP_Outdated.P = 0;
OSQP_Outdated.q = 0;
OSQP_Outdated.d = 0;
OSQP_Outdated.A = 0;
OSQP_Outdated.l = 0;
OSQP_Outdated.u = 0;
}

// update OSQP settings
void set_OSQP_default_settings(){
osqp_set_default_settings(&settings);
}
void set_OSQP_rho(c_float rho_new){
osqp_update_rho(&workspace, rho_new);
}
void set_OSQP_max_iter(c_int max_iter_new){
osqp_update_max_iter(&workspace, max_iter_new);
}
void set_OSQP_eps_abs(c_float eps_abs_new){
osqp_update_eps_abs(&workspace, eps_abs_new);
}
void set_OSQP_eps_rel(c_float eps_rel_new){
osqp_update_eps_rel(&workspace, eps_rel_new);
}
void set_OSQP_eps_prim_inf(c_float eps_prim_inf_new){
osqp_update_eps_prim_inf(&workspace, eps_prim_inf_new);
}
void set_OSQP_eps_dual_inf(c_float eps_dual_inf_new){
osqp_update_eps_dual_inf(&workspace, eps_dual_inf_new);
}
void set_OSQP_alpha(c_float alpha_new){
osqp_update_alpha(&workspace, alpha_new);
}
void set_OSQP_scaled_termination(c_int scaled_termination_new){
osqp_update_scaled_termination(&workspace, scaled_termination_new);
}
void set_OSQP_check_termination(c_int check_termination_new){
osqp_update_check_termination(&workspace, check_termination_new);
}
void set_OSQP_warm_start(c_int warm_start_new){
osqp_update_warm_start(&workspace, warm_start_new);
}
