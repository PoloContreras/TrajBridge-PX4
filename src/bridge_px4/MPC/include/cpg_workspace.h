
#include "types.h"

#ifndef CPG_TYPES_H
# define CPG_TYPES_H

typedef struct {
    int         P;              ///< bool, if OSQP parameter P outdated
    int         q;              ///< bool, if OSQP parameter q outdated
    int         d;              ///< bool, if OSQP parameter d outdated
    int         A;              ///< bool, if OSQP parameter A outdated
    int         l;              ///< bool, if OSQP parameter l outdated
    int         u;              ///< bool, if OSQP parameter u outdated
} OSQP_Outdated_t;

// Struct containing flags for outdated OSQP parameters
extern OSQP_Outdated_t OSQP_Outdated;

typedef struct {
    csc         *P;              ///< OSQP parameter P
    c_float     *q;              ///< OSQP parameter q
    c_float     *d;              ///< OSQP parameter d
    csc         *A;              ///< OSQP parameter A
    c_float     *l;              ///< OSQP parameter l
    c_float     *u;              ///< OSQP parameter u
} OSQP_Params_t;

typedef struct {
    c_float     *objective_value;     ///< Objective function value
    c_float     *X;              ///< Your variable X
    c_float     *U;              ///< Your variable U
    c_float     osqp_solve_time;
} CPG_Result_t;

#endif // ifndef CPG_TYPES_H

// Parameters accepted by OSQP
extern csc OSQP_P;
extern c_float OSQP_q[645];
extern c_float OSQP_d;
extern csc OSQP_A;
extern c_float OSQP_l[677];
extern c_float OSQP_u[677];

// Struct containing parameters accepted by OSQP
extern OSQP_Params_t OSQP_Params;

// Sparse mappings from user-defined to OSQP-accepted parameters
extern csc OSQP_P_map;
extern csc OSQP_A_map;
extern csc OSQP_l_map;
extern csc OSQP_u_map;

// Vector containing flattened user-defined parameters
extern c_float CPG_Params_Vec[187];

// Value of the objective function
extern c_float objective_value;

// User-defined variables

// Struct containing CPG objective value and solution
extern CPG_Result_t CPG_Result;
