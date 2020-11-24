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
$begin sparse_rc.cpp$$
$spell
    rc
$$

$section sparse_rc: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/

// BEGIN C++
# include <cppad/utility/sparse_rc.hpp>
# include <vector>

bool sparse_rc(void)
{   bool ok = true;
    typedef std::vector<size_t> SizeVector;

    // 3 by 3 identity matrix
    size_t nr  = 3;
    size_t nc  = 3;
    size_t nnz = 3;
    CppAD::sparse_rc<SizeVector> pattern(nr, nc, nnz);
    for(size_t k = 0; k < nnz; k++)
        pattern.set(k, k, k);

    // row and column vectors corresponding to pattern
    const SizeVector& row( pattern.row() );
    const SizeVector& col( pattern.row() );

    // check pattern
    ok &= pattern.nnz() == nnz;
    ok &= pattern.nr()  == nr;
    ok &= pattern.nc()  == nc;
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[k] == k;
        ok &= col[k] == k;
    }

    // change to sparsity pattern for a 5 by 5 diagonal matrix
    nr  = 5;
    nc  = 5;
    nnz = 5;
    pattern.resize(nr, nc, nnz);
    for(size_t k = 0; k < nnz; k++)
    {   size_t r = nnz - k - 1; // reverse or row-major order
        size_t c = nnz - k - 1;
        pattern.set(k, r, c);
    }
    SizeVector row_major = pattern.row_major();

    // check row and column
    for(size_t k = 0; k < nnz; k++)
    {   ok &= row[ row_major[k] ] == k;
        ok &= col[ row_major[k] ] == k;
    }

    // create an empty pattern
    CppAD::sparse_rc<SizeVector> target;
    ok &= target.nnz() == 0;
    ok &= target.nr()  == 0;
    ok &= target.nc()  == 0;

    // now use it as the target for an assignment statement
    target = pattern;
    ok    &= target.nr()  == pattern.nr();
    ok    &= target.nc()  == pattern.nc();
    ok    &= target.nnz() == pattern.nnz();
    for(size_t k = 0; k < nnz; k++)
    {   ok &= target.row()[k] == row[k];
        ok &= target.col()[k] == col[k];
    }
    return ok;
}

// END C++
