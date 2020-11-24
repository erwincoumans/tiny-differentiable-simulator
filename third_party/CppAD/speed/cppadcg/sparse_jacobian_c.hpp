# ifndef CPPAD_SPEED_CPPADCG_SPARSE_JACOBIAN_C_HPP
# define CPPAD_SPEED_CPPADCG_SPARSE_JACOBIAN_C_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
// BEGIN_SPARSE_JACOBIAN_C
extern "C" int sparse_jacobian_c(
    int           subgraph   ,
    int           optimized  ,
    int           seed       ,
    int           size       ,
    int           nnz        ,
    const double* x          ,
    double*       y
);
// END_SPARSE_JACOBIAN_C
# endif
