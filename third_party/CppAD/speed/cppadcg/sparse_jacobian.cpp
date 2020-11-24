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
$begin cppadcg_sparse_jacobian.cpp$$
$spell
    cppadcg
    jacobian
$$

$section Cppadcg Speed: sparse_jacobian$$

$head Specifications$$
$cref link_sparse_jacobian$$

$childtable%
    speed/cppadcg/sparse_jacobian.c%
    speed/cppadcg/sparse_jacobian_cg.cpp
%$$

$srccode%cpp% */
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/vector.hpp>

# include <map>
extern std::map<std::string, bool> global_option;

// routine created by sparse_jacobian_cg
# include "sparse_jacobian_c.hpp"

// random seed
extern size_t global_seed;

bool link_sparse_jacobian(
    size_t                           size     ,
    size_t                           repeat   ,
    size_t                           m        ,
    const CppAD::vector<size_t>&     row      ,
    const CppAD::vector<size_t>&     col      ,
    CppAD::vector<double>&           x        ,
    CppAD::vector<double>&           jacobian ,
    size_t&                          n_color  )
{   assert( x.size() == size );
    assert( jacobian.size() == row.size() );
    // --------------------------------------------------------------------
    // check global options
    const char* valid[] = { "onetape", "optimize", "subgraph"};
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
    if( ! global_option["onetape"] )
        return false;
    // -----------------------------------------------------
    int optimize_int = int( global_option["optimize"] );
    int subgraph_int = int( global_option["subgraph"] );
    int nnz_int      = int( row.size() );
    int seed_int     = int( global_seed );
    int size_int     = int( size );
    while( repeat-- )
    {   // choose argument vector
        CppAD::uniform_01(size, x);
        //
        // compute sparse Jacobian
        int flag = sparse_jacobian_c(
            subgraph_int,
            optimize_int,
            seed_int,
            size_int,
            nnz_int,
            x.data(),
            jacobian.data()
        );
        if( flag != 0 )
            return false;
    }
    n_color = 0;
    return true;
}
/* %$$
$end
*/
