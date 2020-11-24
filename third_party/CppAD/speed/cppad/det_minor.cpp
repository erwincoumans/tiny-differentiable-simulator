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
$begin cppad_det_minor.cpp$$
$spell
    onetape
    typedef
    cppad
    CppAD
    det
    hpp
    const
    bool
    std
$$

$section Cppad Speed: Gradient of Determinant by Minor Expansion$$


$head Specifications$$
See $cref link_det_minor$$.

$head Implementation$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
# include <cppad/speed/det_by_minor.hpp>
# include <cppad/speed/uniform_01.hpp>

// Note that CppAD uses global_option["memory"] at the main program level
# include <map>
extern std::map<std::string, bool> global_option;
// see comments in main program for this external
extern size_t global_cppad_thread_alloc_inuse;

bool link_det_minor(
    size_t                     size     ,
    size_t                     repeat   ,
    CppAD::vector<double>     &matrix   ,
    CppAD::vector<double>     &gradient )
{   global_cppad_thread_alloc_inuse = 0;

    // --------------------------------------------------------------------
    // check global options
    const char* valid[] = { "memory", "onetape", "optimize"};
    size_t n_valid = sizeof(valid) / sizeof(valid[0]);
    typedef std::map<std::string, bool>::iterator iterator;
    //
    for(iterator itr=global_option.begin(); itr!=global_option.end(); ++itr)
    {   if( itr->second )
        {   bool ok = false;
            for(size_t i = 0; i < n_valid; i++)
                ok |= itr->first == valid[i];
            if( ! ok )
                return false;
        }
    }
    // --------------------------------------------------------------------
    // optimization options: no conditional skips or compare operators
    std::string optimize_options =
        "no_conditional_skip no_compare_op no_print_for_op";
    // -----------------------------------------------------
    // setup

    // object for computing determinant
    typedef CppAD::AD<double>       a_double;
    typedef CppAD::vector<a_double> a_vector;
    CppAD::det_by_minor<a_double>   a_det(size);

    size_t m = 1;           // number of dependent variables
    size_t n = size * size; // number of independent variables
    a_vector   a_A(n);      // AD domain space vector
    a_vector   a_detA(m);   // AD range space vector

    // vectors of reverse mode weights
    CppAD::vector<double> w(1);
    w[0] = 1.;

    // the AD function object
    CppAD::ADFun<double> f;

    // do not even record comparison operators
    size_t abort_op_index = 0;
    bool record_compare   = false;

    // ---------------------------------------------------------------------
    if( ! global_option["onetape"] ) while(repeat--)
    {
        // choose a matrix
        CppAD::uniform_01(n, matrix);
        for(size_t i = 0; i < n; i++)
            a_A[i] = matrix[i];

        // declare independent variables
        Independent(a_A, abort_op_index, record_compare);

        // AD computation of the determinant
        a_detA[0] = a_det(a_A);

        // create function object f : A -> detA
        f.Dependent(a_A, a_detA);

        if( global_option["optimize"] )
            f.optimize(optimize_options);

        // skip comparison operators
        f.compare_change_count(0);

        // evaluate the determinant at the new matrix value
        f.Forward(0, matrix);

        // evaluate and return gradient using reverse mode
        gradient = f.Reverse(1, w);
    }
    else
    {
        // choose a matrix
        CppAD::uniform_01(n, matrix);
        for(size_t i = 0; i < n; i++)
            a_A[i] = matrix[i];

        // declare independent variables
        Independent(a_A, abort_op_index, record_compare);

        // AD computation of the determinant
        a_detA[0] = a_det(a_A);

        // create function object f : A -> detA
        f.Dependent(a_A, a_detA);

        if( global_option["optimize"] )
            f.optimize(optimize_options);

        // skip comparison operators
        f.compare_change_count(0);

        // ------------------------------------------------------
        while(repeat--)
        {   // get the next matrix
            CppAD::uniform_01(n, matrix);

            // evaluate the determinant at the new matrix value
            f.Forward(0, matrix);

            // evaluate and return gradient using reverse mode
            gradient = f.Reverse(1, w);
        }
    }
    size_t thread                   = CppAD::thread_alloc::thread_num();
    global_cppad_thread_alloc_inuse = CppAD::thread_alloc::inuse(thread);
    return true;
}
/* %$$
$end
*/
