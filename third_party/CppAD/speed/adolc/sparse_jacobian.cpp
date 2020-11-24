/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cstring>
# include <cppad/utility/vector.hpp>

/*
$begin adolc_sparse_jacobian.cpp$$
$spell
    const
    sparsedrivers.cpp
    colpack
    boolsparsity
    adouble
    int int_n
    cppad.hpp
    onetape
    typedef
    alloc
    jac
    nnz
    cind
    bool
    CppAD
    adolc
    sparse_jacobian
$$

$section Adolc Speed: Sparse Jacobian$$


$head Specifications$$
See $cref link_sparse_jacobian$$.

$head Implementation$$

$srccode%cpp% */
// suppress conversion warnings before other includes
# include <cppad/wno_conversion.hpp>
//
# include <adolc/adolc.h>
# include <adolc/adolc_sparse.h>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>

// list of possible options
# include <map>
extern std::map<std::string, bool> global_option;

bool link_sparse_jacobian(
    size_t                           size     ,
    size_t                           repeat   ,
    size_t                           m        ,
    const CppAD::vector<size_t>&     row      ,
    const CppAD::vector<size_t>&     col      ,
          CppAD::vector<double>&     x_return ,
          CppAD::vector<double>&     jacobian ,
          size_t&                    n_color  )
{
    if( global_option["atomic"] || (! global_option["colpack"]) )
        return false;
    if( global_option["memory"] || global_option["optimize"] )
        return false;
    // -----------------------------------------------------
    // setup
    typedef unsigned int*    SizeVector;
    typedef double*          DblVector;
    typedef adouble          ADScalar;
    typedef ADScalar*        ADVector;

    size_t i, j, k;            // temporary indices
    size_t n = size;           // number of independent variables
    size_t order = 0;          // derivative order corresponding to function

    // set up for thread_alloc memory allocator (fast and checks for leaks)
    using CppAD::thread_alloc; // the allocator
    size_t capacity;           // capacity of an allocation

    // tape identifier
    int tag  = 0;
    // AD domain space vector
    ADVector a_x = thread_alloc::create_array<ADScalar>(n, capacity);
    // AD range space vector
    ADVector a_y = thread_alloc::create_array<ADScalar>(m, capacity);
    // argument value in double
    DblVector x = thread_alloc::create_array<double>(n, capacity);
    // function value in double
    DblVector y = thread_alloc::create_array<double>(m, capacity);


    // options that control sparse_jac
    int        options[4];
    if( global_option["boolsparsity"] )
        options[0] = 1;  // sparsity by propagation of bit pattern
    else
        options[0] = 0;  // sparsity pattern by index domains
    options[1] = 0; // (0 = safe mode, 1 = tight mode)
    options[2] = 0; // see changing to -1 and back to 0 below
    options[3] = 0; // (0 = column compression, 1 = row compression)

    // structure that holds some of the work done by sparse_jac
    int        nnz;                   // number of non-zero values
    SizeVector rind   = CPPAD_NULL;   // row indices
    SizeVector cind   = CPPAD_NULL;   // column indices
    DblVector  values = CPPAD_NULL;   // Jacobian values

    // choose a value for x
    CppAD::uniform_01(n, x);

    // declare independent variables
    int keep = 0; // keep forward mode results
    trace_on(tag, keep);
    for(j = 0; j < n; j++)
        a_x[j] <<= x[j];

    // AD computation of f (x)
    CppAD::sparse_jac_fun<ADScalar>(m, n, a_x, row, col, order, a_y);

    // create function object f : x -> y
    for(i = 0; i < m; i++)
        a_y[i] >>= y[i];
    trace_off();

    // Retrieve n_color using undocumented feature of sparsedrivers.cpp
    int same_pattern = 0;
    options[2]       = -1;
    n_color = sparse_jac(tag, int(m), int(n),
        same_pattern, x, &nnz, &rind, &cind, &values, options
    );
    options[2]       = 0;
    // ----------------------------------------------------------------------
    if( ! global_option["onetape"] ) while(repeat--)
    {   // choose a value for x
        CppAD::uniform_01(n, x);

        // declare independent variables
        trace_on(tag, keep);
        for(j = 0; j < n; j++)
            a_x[j] <<= x[j];

        // AD computation of f (x)
        CppAD::sparse_jac_fun<ADScalar>(m, n, a_x, row, col, order, a_y);

        // create function object f : x -> y
        for(i = 0; i < m; i++)
            a_y[i] >>= y[i];
        trace_off();

        // is this a repeat call with the same sparsity pattern
        same_pattern = 0;

        // calculate the jacobian at this x
        rind   = CPPAD_NULL;
        cind   = CPPAD_NULL;
        values = CPPAD_NULL;
        sparse_jac(tag, int(m), int(n),
            same_pattern, x, &nnz, &rind, &cind, &values, options
        );
        // only needed last time through loop
        if( repeat == 0 )
        {   assert( size_t(nnz) == row.size() );
            for(int ell = 0; ell < nnz; ell++)
            {   assert( row[ell] == size_t(rind[ell]) );
                assert( col[ell] == size_t(cind[ell]) );
                jacobian[ell] = values[ell];
            }
        }

        // free raw memory allocated by sparse_jac
        free(rind);
        free(cind);
        free(values);
    }
    else
    {   while(repeat--)
        {   // choose a value for x
            CppAD::uniform_01(n, x);

            // calculate the jacobian at this x
            sparse_jac(tag, int(m), int(n),
                same_pattern, x, &nnz, &rind, &cind, &values, options
            );
            same_pattern = 1;
        }
        // check that acolc has the same sparsity pattern in row major order
        bool ok = size_t(nnz) == row.size();
        for(k = 0; k < row.size(); ++k)
        {   ok &= row[k] == size_t( rind[k] );
            ok &= col[k] == size_t( cind[k] );
            jacobian[k] = values[k];
        }
        // assert here in case adolc stops returning same pattern
        assert( ok );

        // free raw memory allocated by sparse_jac
        free(rind);
        free(cind);
        free(values);
    }
    // --------------------------------------------------------------------
    // return argument
    for(j = 0; j < n; j++)
        x_return[j] = x[j];

    // tear down
    thread_alloc::delete_array(a_x);
    thread_alloc::delete_array(a_y);
    thread_alloc::delete_array(x);
    thread_alloc::delete_array(y);
    return true;
}
/* %$$
$end
*/
