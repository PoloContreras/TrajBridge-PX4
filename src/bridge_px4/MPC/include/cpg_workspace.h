
#include "types.h"

#ifndef CPG_TYPES_H
# define CPG_TYPES_H

typedef struct {
    int         P;       ///< bool, if canonical parameter P outdated
    int         q;       ///< bool, if canonical parameter q outdated
    int         d;       ///< bool, if canonical parameter d outdated
    int         A;       ///< bool, if canonical parameter A outdated
    int         l;       ///< bool, if canonical parameter l outdated
    int         u;       ///< bool, if canonical parameter u outdated
} Canon_Outdated_t;

typedef struct {
    csc         *P;      ///< bool, if canonical parameter P outdated
    c_float     *q;      ///< bool, if canonical parameter q outdated
    c_float     *d;      ///< bool, if canonical parameter d outdated
    csc         *A;      ///< bool, if canonical parameter A outdated
    c_float     *l;      ///< bool, if canonical parameter l outdated
    c_float     *u;      ///< bool, if canonical parameter u outdated
} Canon_Params_t;

typedef struct {
    c_float     *objective_value;     ///< Objective function value
    c_float     *X;              ///< Your variable X
    c_float     *U;              ///< Your variable U
    c_float     osqp_solve_time;
} CPG_Result_t;

#endif // ifndef CPG_TYPES_H

// Struct containing flags for outdated canonical parameters
extern Canon_Outdated_t Canon_Outdated;

// Canonical parameters
extern csc Canon_P;
extern csc Canon_P_ECOS;
extern c_float Canon_q[705];
extern c_float Canon_q_ECOS[705];
extern c_float Canon_d;
extern c_float Canon_d_ECOS;
extern csc Canon_A;
extern csc Canon_A_ECOS;
extern c_float Canon_l[795];
extern c_float Canon_l_ECOS[795];
extern c_float Canon_u[795];
extern c_float Canon_u_ECOS[795];

// Struct containing canonical parameters
extern Canon_Params_t Canon_Params;
extern Canon_Params_t Canon_Params_ECOS;

// Sparse mappings from user-defined to canonical parameters
extern csc Canon_P_map;
extern csc Canon_A_map;
extern csc Canon_l_map;
extern csc Canon_u_map;

// Vector containing flattened user-defined parameters
extern c_float CPG_Params_Vec[187];

// Value of the objective function
extern c_float objective_value;

// Struct containing CPG objective value and solution
extern CPG_Result_t CPG_Result;
