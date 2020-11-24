/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin cppadcg_det_minor_grad.c$$
$spell
    Cppadcg
    det
$$

$section Cppadcg Speed: Gradient of Determinant by Minor Expansion$$

$head Syntax$$
$icode%flag% = det_minor_grad_c(%optimize%, %size%, %x%, %y%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_DET_MINOR_GRAD_C%// END_DET_MINOR_GRAD_C%1
%$$

$head Purpose$$
Computes the gradient of the determinant for a square matrix.

$head optimize$$
If it is zero (one), the non-optimized (optimized) version
of the computation is used.

$head size$$
Is the row and column dimension of the square matrix.
This function is only implemented for one value of $icode size$$.
You can create a version of this function for a different value of
$icode size$$ by calling $cref cppadcg_det_minor.cpp$$.

$head x$$
Is the values in the matrix at which we are evaluating the gradient
(stored in either row or column major order).
The length of this vector is the product of its row and column dimension;
i.e., $icode%size%*size%$$.

$head y$$
Is the gradient of the determinant function.
It size is the same as $icode x$$.
The length of this vector is the same as $icode x$$.
The input value of it's elements does not matter.
Upon return the $th j$$ component of $icode y$$
is the partial of the determinant
with respect to the $th j$$ component of $icode x$$.

$head flag$$
This value is zero if the value of $icode size$$ is passed in
can be handled by this source code; see $cref cppadcg_det_minor_cg.cpp$$.
Otherwise,
$icode y$$ is not used and the return value of flag is $code 1$$.

$head Implementation$$
The following is an implementation for the case when $icode size$$
is one:

$end
*/
// BEGIN_DET_MINOR_GRAD_C
int det_minor_grad_c(int optimized, int size, const double* x, double* y)
// END_DET_MINOR_GRAD_C
{   return 1; }
