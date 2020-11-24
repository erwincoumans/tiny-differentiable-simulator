# ifndef CPPAD_SPEED_CPPADCG_DET_MINOR_GRAD_C_HPP
# define CPPAD_SPEED_CPPADCG_DET_MINOR_GRAD_C_HPP

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
// BEGIN_DET_MINOR_GRAD_C
extern "C" int det_minor_grad_c(
    int optimized, int size, const double* x, double* y
);
// END_DET_MINOR_GRAD_C

# endif
