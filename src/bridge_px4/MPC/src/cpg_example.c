
#include <stdio.h>
#include "cpg_workspace.h"
#include "cpg_solve.h"

static c_int i;

int main(int argc, char *argv[]){

// initialize user-defined parameter values
update_P_sqrt(0, 3.16227766016837952279);
update_P_sqrt(1, 0.00000000000000000000);
update_P_sqrt(2, 0.00000000000000000000);
update_P_sqrt(3, 0.00000000000000000000);
update_P_sqrt(4, 0.00000000000000000000);
update_P_sqrt(5, 0.00000000000000000000);
update_P_sqrt(6, 0.00000000000000000000);
update_P_sqrt(7, 3.16227766016837952279);
update_P_sqrt(8, 0.00000000000000000000);
update_P_sqrt(9, 0.00000000000000000000);
update_P_sqrt(10, 0.00000000000000000000);
update_P_sqrt(11, 0.00000000000000000000);
update_P_sqrt(12, 0.00000000000000000000);
update_P_sqrt(13, 0.00000000000000000000);
update_P_sqrt(14, 3.16227766016837952279);
update_P_sqrt(15, 0.00000000000000000000);
update_P_sqrt(16, 0.00000000000000000000);
update_P_sqrt(17, 0.00000000000000000000);
update_P_sqrt(18, 0.00000000000000000000);
update_P_sqrt(19, 0.00000000000000000000);
update_P_sqrt(20, 0.00000000000000000000);
update_P_sqrt(21, 3.16227766016837952279);
update_P_sqrt(22, 0.00000000000000000000);
update_P_sqrt(23, 0.00000000000000000000);
update_P_sqrt(24, 0.00000000000000000000);
update_P_sqrt(25, 0.00000000000000000000);
update_P_sqrt(26, 0.00000000000000000000);
update_P_sqrt(27, 0.00000000000000000000);
update_P_sqrt(28, 3.16227766016837952279);
update_P_sqrt(29, 0.00000000000000000000);
update_P_sqrt(30, 0.00000000000000000000);
update_P_sqrt(31, 0.00000000000000000000);
update_P_sqrt(32, 0.00000000000000000000);
update_P_sqrt(33, 0.00000000000000000000);
update_P_sqrt(34, 0.00000000000000000000);
update_P_sqrt(35, 3.16227766016837952279);
update_Q_sqrt(0, 1.00000000000000000000);
update_Q_sqrt(1, 0.00000000000000000000);
update_Q_sqrt(2, 0.00000000000000000000);
update_Q_sqrt(3, 0.00000000000000000000);
update_Q_sqrt(4, 0.00000000000000000000);
update_Q_sqrt(5, 0.00000000000000000000);
update_Q_sqrt(6, 0.00000000000000000000);
update_Q_sqrt(7, 1.00000000000000000000);
update_Q_sqrt(8, 0.00000000000000000000);
update_Q_sqrt(9, 0.00000000000000000000);
update_Q_sqrt(10, 0.00000000000000000000);
update_Q_sqrt(11, 0.00000000000000000000);
update_Q_sqrt(12, 0.00000000000000000000);
update_Q_sqrt(13, 0.00000000000000000000);
update_Q_sqrt(14, 1.00000000000000000000);
update_Q_sqrt(15, 0.00000000000000000000);
update_Q_sqrt(16, 0.00000000000000000000);
update_Q_sqrt(17, 0.00000000000000000000);
update_Q_sqrt(18, 0.00000000000000000000);
update_Q_sqrt(19, 0.00000000000000000000);
update_Q_sqrt(20, 0.00000000000000000000);
update_Q_sqrt(21, 1.00000000000000000000);
update_Q_sqrt(22, 0.00000000000000000000);
update_Q_sqrt(23, 0.00000000000000000000);
update_Q_sqrt(24, 0.00000000000000000000);
update_Q_sqrt(25, 0.00000000000000000000);
update_Q_sqrt(26, 0.00000000000000000000);
update_Q_sqrt(27, 0.00000000000000000000);
update_Q_sqrt(28, 1.00000000000000000000);
update_Q_sqrt(29, 0.00000000000000000000);
update_Q_sqrt(30, 0.00000000000000000000);
update_Q_sqrt(31, 0.00000000000000000000);
update_Q_sqrt(32, 0.00000000000000000000);
update_Q_sqrt(33, 0.00000000000000000000);
update_Q_sqrt(34, 0.00000000000000000000);
update_Q_sqrt(35, 1.00000000000000000000);
update_R_sqrt(0, 0.10000000000000000555);
update_R_sqrt(1, 0.00000000000000000000);
update_R_sqrt(2, 0.00000000000000000000);
update_R_sqrt(3, 0.00000000000000000000);
update_R_sqrt(4, 0.10000000000000000555);
update_R_sqrt(5, 0.00000000000000000000);
update_R_sqrt(6, 0.00000000000000000000);
update_R_sqrt(7, 0.00000000000000000000);
update_R_sqrt(8, 0.10000000000000000555);
update_S_sqrt(0, 0.31622776601683794118);
update_S_sqrt(1, 0.00000000000000000000);
update_S_sqrt(2, 0.00000000000000000000);
update_S_sqrt(3, 0.00000000000000000000);
update_S_sqrt(4, 0.31622776601683794118);
update_S_sqrt(5, 0.00000000000000000000);
update_S_sqrt(6, 0.00000000000000000000);
update_S_sqrt(7, 0.00000000000000000000);
update_S_sqrt(8, 0.31622776601683794118);
update_A(0, 1.00000000000000000000);
update_A(1, 0.00000000000000000000);
update_A(2, 0.00000000000000000000);
update_A(3, 0.00000000000000000000);
update_A(4, 0.00000000000000000000);
update_A(5, 0.00000000000000000000);
update_A(6, 0.00000000000000000000);
update_A(7, 1.00000000000000000000);
update_A(8, 0.00000000000000000000);
update_A(9, 0.00000000000000000000);
update_A(10, 0.00000000000000000000);
update_A(11, 0.00000000000000000000);
update_A(12, 0.00000000000000000000);
update_A(13, 0.00000000000000000000);
update_A(14, 1.00000000000000000000);
update_A(15, 0.00000000000000000000);
update_A(16, 0.00000000000000000000);
update_A(17, 0.00000000000000000000);
update_A(18, 0.03333333333333333287);
update_A(19, 0.00000000000000000000);
update_A(20, 0.00000000000000000000);
update_A(21, 1.00000000000000000000);
update_A(22, 0.00000000000000000000);
update_A(23, 0.00000000000000000000);
update_A(24, 0.00000000000000000000);
update_A(25, 0.03333333333333333287);
update_A(26, 0.00000000000000000000);
update_A(27, 0.00000000000000000000);
update_A(28, 1.00000000000000000000);
update_A(29, 0.00000000000000000000);
update_A(30, 0.00000000000000000000);
update_A(31, 0.00000000000000000000);
update_A(32, 0.03333333333333333287);
update_A(33, 0.00000000000000000000);
update_A(34, 0.00000000000000000000);
update_A(35, 1.00000000000000000000);
update_B(0, 0.00104821802935010466);
update_B(1, 0.00000000000000000000);
update_B(2, 0.00000000000000000000);
update_B(3, 0.06289308176100627534);
update_B(4, 0.00000000000000000000);
update_B(5, 0.00000000000000000000);
update_B(6, 0.00000000000000000000);
update_B(7, 0.00104821802935010466);
update_B(8, 0.00000000000000000000);
update_B(9, 0.00000000000000000000);
update_B(10, 0.06289308176100627534);
update_B(11, 0.00000000000000000000);
update_B(12, 0.00000000000000000000);
update_B(13, 0.00000000000000000000);
update_B(14, 0.00104821802935010466);
update_B(15, 0.00000000000000000000);
update_B(16, 0.00000000000000000000);
update_B(17, 0.06289308176100627534);
update_x_init(0, -0.75000000000000000000);
update_x_init(1, -0.50000000000000000000);
update_x_init(2, -0.25000000000000000000);
update_x_init(3, -0.10000000000000000555);
update_x_init(4, -0.10000000000000000555);
update_x_init(5, -0.10000000000000000555);
update_u_prev(0, 0.00000000000000000000);
update_u_prev(1, 0.00000000000000000000);
update_u_prev(2, 0.00000000000000000000);
update_fv_min(-2.59965000000000046043);
update_fv_max(4.64877753012207861616);
update_gamma(0.12468200376510511773);
update_G(0, 0.64825914217591107391);
update_G(1, 0.64825914217591107391);
update_G(2, 0.64825914217591107391);
update_G(3, 0.64825914217591107391);
update_G(4, 0.64825914217591107391);
update_G(5, 0.64825914217591107391);
update_G(6, 0.64825914217591107391);
update_G(7, 0.64825914217591107391);
update_G(8, 0.64825914217591107391);
update_G(9, 0.64825914217591107391);
update_G(10, 0.64825914217591107391);
update_G(11, 0.64825914217591107391);
update_G(12, 0.64825914217591107391);
update_G(13, 0.64825914217591107391);
update_G(14, 0.64825914217591107391);
update_G(15, 0.64825914217591107391);
update_G(16, 0.64825914217591107391);
update_G(17, 0.64825914217591107391);
update_G(18, 0.64825914217591107391);
update_G(19, 0.64825914217591107391);
update_G(20, 0.64825914217591107391);
update_G(21, 0.64825914217591107391);
update_G(22, 0.64825914217591107391);
update_G(23, 0.64825914217591107391);
update_G(24, 0.64825914217591107391);
update_G(25, 0.64825914217591107391);
update_G(26, 0.64825914217591107391);
update_G(27, 0.64825914217591107391);
update_G(28, 0.64825914217591107391);
update_G(29, 0.64825914217591107391);

// solve the problem instance
solve();

// printing objective function value for demonstration purpose
printf("f = %f \n", objective_value);

// printing solution for demonstration purpose
for(i = 0; i < 186; i++) {
printf("X[%lld] = %f \n", i, CPG_Result.X[i]);
}
for(i = 0; i < 93; i++) {
printf("U[%lld] = %f \n", i, CPG_Result.U[i]);
}
return 0;
}
