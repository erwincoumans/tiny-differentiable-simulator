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
$begin cppadcg_sparse_jacobian.c$$
$spell
    Cppadcg
    Jacobian
    subgraph
    nnz
$$

$section Cppadcg Speed: Sparse Jacobian$$

$head Syntax$$
$icode%flag% = sparse_jacobian_c(
    %subgraph%, %optimize%, %seed%, %size%, %nnz%, %x%, %y%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_SPARSE_JACOBIAN_C%// END_SPARSE_JACOBIAN_C%1
%$$

$head Purpose$$
Computes a sparse Jacobian corresponding to
the function $cref sparse_jac_fun$$.

$head subgraph$$
If it is one (zero), the $cref subgraph_jac_rev$$
method is used (is not used) to compute the sparse Jacobian.
This function was only implemented for one value of $icode subgraph$$.
You can create a new version of this function by calling
$cref cppadcg_sparse_jacobian_cg.cpp$$.

$head optimize$$
If it is one (zero), the optimized (non-optimized) version
of the computation is used.
This function was only implemented for one value of $icode optimize$$.

$head seed$$
is the random number seed used to choose the row and column
vectors.
This function was only implemented for one value of $icode seed$$.

$head size$$
Is the dimension of the argument space for the function.
This function is only implemented few values of $icode size$$.

$head nnz$$
is the number of non-zeros in the sparsity pattern for the Jacobian.

$head x$$
This is a vector of length $icode size$$ containing the
point at which to evaluate the derivative.

$head y$$
Is a vector with size $icode nnz$$ containing the non-zeros
in the sparse Jacobian.
The input value of it's elements does not matter.
Upon return,
for $icode%k% = 0 , %...%, %nnz%-1%$$.
the $th k$$ component of $icode y$$
is the $th k$$ non-zero in the sparse Jacobian.

$head flag$$
If the value of
$icode subgraph$$, $icode optimize$$, and $icode seed$$ are the same,
and $icode size$$ was one of the sizes,
when the current version of $code sparse_jacobian_c$$
was generated, $icode%flag% = 0%$$.
If the value of $icode subgraph$$ , $icode optimize$$, or $icode seed$$
is different, $icode%flag% = 1%$$.
If the value of $icode subgraph$$ , $icode optimize$$, and $icode seed$$
are the same and $icode size$$
is not one of the sizes, $icode%flag% = 2%$$.

$end
*/
// BEGIN_SPARSE_JACOBIAN_C
int sparse_jacobian_c(
    int           subgraph   ,
    int           optimized  ,
    int           seed       ,
    int           size       ,
    int           nnz        ,
    const double* x          ,
    double*       y          )
// END_SPARSE_JACOBIAN_C
{   // This version returns random seed error flag so that it gets replaced
    return 1;
}
