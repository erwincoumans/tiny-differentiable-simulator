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
$begin adolc_det_minor.cpp$$
$spell
    thread_alloc
    onetape
    cppad
    zos
    fos
    adouble
    CppAD
    typedef
    adolc
    Lu
    Adolc
    det
    hpp
    const
    bool
    srand
$$

$section Adolc Speed: Gradient of Determinant by Minor Expansion$$


$head Specifications$$
See $cref link_det_minor$$.

$head Implementation$$
$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <adolc/adolc.h>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/det_by_minor.hpp>
# include <cppad/speed/uniform_01.hpp>

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
    if( global_option["memory"] || global_option["optimize"] )
        return false;
    // -----------------------------------------------------
    // setup
    typedef adouble    ADScalar;
    typedef ADScalar*  ADVector;

    int tag  = 0;         // tape identifier
    int m    = 1;         // number of dependent variables
    int n    = size*size; // number of independent variables
    double f;             // function value
    int j;                // temporary index

    // set up for thread_alloc memory allocator (fast and checks for leaks)
    using CppAD::thread_alloc; // the allocator
    size_t capacity;           // capacity of an allocation

    // object for computing determinant
    CppAD::det_by_minor<ADScalar> Det(size);

    // AD value of determinant
    ADScalar   detA;

    // AD version of matrix
    ADVector A   = thread_alloc::create_array<ADScalar>(size_t(n), capacity);

    // vectors of reverse mode weights
    double* u    = thread_alloc::create_array<double>(size_t(m), capacity);
    u[0] = 1.;

    // vector with matrix value
    double* mat  = thread_alloc::create_array<double>(size_t(n), capacity);

    // vector to receive gradient result
    double* grad = thread_alloc::create_array<double>(size_t(n), capacity);

    // ----------------------------------------------------------------------
    if( ! global_option["onetape"] ) while(repeat--)
    {   // choose a matrix
        CppAD::uniform_01(n, mat);

        // declare independent variables
        int keep = 1; // keep forward mode results
        trace_on(tag, keep);
        for(j = 0; j < n; j++)
            A[j] <<= mat[j];

        // AD computation of the determinant
        detA = Det(A);

        // create function object f : A -> detA
        detA >>= f;
        trace_off();

        // evaluate and return gradient using reverse mode
        fos_reverse(tag, m, n, u, grad);
    }
    else
    {
        // choose a matrix
        CppAD::uniform_01(n, mat);

        // declare independent variables
        int keep = 0; // do not keep forward mode results in buffer
        trace_on(tag, keep);
        for(j = 0; j < n; j++)
            A[j] <<= mat[j];

        // AD computation of the determinant
        detA = Det(A);

        // create function object f : A -> detA
        detA >>= f;
        trace_off();

        while(repeat--)
        {   // get the next matrix
            CppAD::uniform_01(n, mat);

            // evaluate the determinant at the new matrix value
            keep = 1; // keep this forward mode result
            zos_forward(tag, m, n, keep, mat, &f);

            // evaluate and return gradient using reverse mode
            fos_reverse(tag, m, n, u, grad);
        }
    }
    // --------------------------------------------------------------------

    // return matrix and gradient
    for(j = 0; j < n; j++)
    {   matrix[j] = mat[j];
        gradient[j] = grad[j];
    }

    // tear down
    thread_alloc::delete_array(grad);
    thread_alloc::delete_array(mat);
    thread_alloc::delete_array(u);
    thread_alloc::delete_array(A);
    return true;
}
/* %$$
$end
*/
