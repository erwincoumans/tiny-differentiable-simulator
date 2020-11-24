/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin sacado_det_minor.cpp$$
$spell
    onetape
    cppad
    det
    const
    CppAD
    typedef
    diff
    bool
    srand
    Sacado.hpp
    ADvar
    Gradcomp
$$

$section Sacado Speed: Gradient of Determinant by Minor Expansion$$


$head Specifications$$
See $cref link_det_minor$$.

$head Implementation$$

$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <Sacado.hpp>
# include <cppad/speed/det_by_minor.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/vector.hpp>

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

bool link_det_minor(
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>     &matrix   ,
    CppAD::vector<double>     &gradient )
{
    // speed test global option values
    if( global_option["atomic"] )
        return false;
    if( global_option["memory"] || global_option["onetape"] || global_option["optimize"] )
        return false;
    // -----------------------------------------------------
    // setup

    // object for computing determinant
    typedef Sacado::Rad::ADvar<double>    ADScalar;
    typedef CppAD::vector<ADScalar>       ADVector;
    CppAD::det_by_minor<ADScalar>         Det(size);

    size_t i;                // temporary index
    size_t n = size * size;  // number of independent variables
    ADScalar   detA;         // AD value of the determinant
    ADVector   A(n);         // AD version of matrix

    // ------------------------------------------------------
    while(repeat--)
    {   // get the next matrix
        CppAD::uniform_01(n, matrix);

        // set independent variable values
        for(i = 0; i < n; i++)
            A[i] = matrix[i];

        // compute the determinant
        detA = Det(A);

        // reverse mode compute gradient of last computed value; i.e., detA
        ADScalar::Gradcomp();

        // return gradient
        for(i =0; i < n; i++)
            gradient[i] = A[i].adj(); // partial detA w.r.t A[i]
    }
    // ---------------------------------------------------------
    return true;
}
/* %$$
$end
*/
