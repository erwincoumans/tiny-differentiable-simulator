/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/utility/near_equal.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/index_sort.hpp>

# include "link_sparse_jacobian.hpp"

/*
 ------------------------------------------------------------------------------
$begin choose_row_col_sparse_jacobian$$
$spell
    Jacobian
    CppAD
$$
$section Randomly Choose Row and Column Indices for Sparse Jacobian$$

$head Syntax$$
$codei%choose_row_col_sparse_jacobian(%seed%, %n%, %m%, %row%, %col%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_CHOOSE_ROW_COL%// END_CHOOSE_ROW_COL%1
%$$

$head seed$$
is used to initialize the random generator
at the beginning of this routine as follows:
$codei%
    CppAD::uniform_01(%seed%)
%$$
This makes sure that same rows and columns are chosen
by different calls with the same values of
$icode seed$$, $icode n$$ and $icode m$$.

$head n$$
is the dimension of the domain space for the function f(x).

$head m$$
is the dimension of the range space for the function f(x).

$head row$$
The input size and elements of $icode row$$ do not matter.
Upon return it is the chosen row indices.

$head col$$
The input size and elements of $icode col$$ do not matter.
Upon return it is the chosen column indices.

$end
*/
// BEGIN_CHOOSE_ROW_COL
void choose_row_col_sparse_jacobian(
    size_t                 seed ,
    size_t                 n    ,
    size_t                 m    ,
    CppAD::vector<size_t>& row  ,
    CppAD::vector<size_t>& col  )
// END_CHOOSE_ROW_COL
{   using CppAD::vector;
    //
    // reset random number generator
    CppAD::uniform_01(seed);
    //
    // get random numbers for row and column indices
    size_t K = 5 * std::max(m, n);
    vector<double>  random(2 * K);
    CppAD::uniform_01(2 * K, random);
    //
    // sort the temporary row and colunn choices
    vector<size_t> key(K);
    vector<size_t> ind(K);
    for(size_t k = 0; k < K; k++)
    {   // convert from [0,1] to row index
        // avoid warning when converting double to size_t
        size_t r = size_t( float( double(m) * random[k] ) );
        r        = std::min(m-1, r);
        // convert from [0,1] to column index
        size_t c = size_t( float( double(n) * random[k + K] ) );
        c        = std::min(n-1, c);
        //
        // key in row major order
        key[k] = c + n * r;
    }
    CppAD::index_sort(key, ind);
    //
    // remove duplicates and set row, col in row major order
    row.resize(0);
    col.resize(0);
    size_t k          = ind[0];
    size_t c_previous = key[k] % n;
    size_t r_previous = key[k] / n;
    CPPAD_ASSERT_UNKNOWN( r_previous < m && c_previous < n );
    CPPAD_ASSERT_UNKNOWN( key[k] == c_previous + n * r_previous );
    row.push_back(r_previous);
    col.push_back(c_previous);
    for(size_t ell = 1; ell < K; ell++)
    {   k        = ind[ell];
        size_t c = key[k] % n;
        size_t r = key[k] / n;
        CPPAD_ASSERT_UNKNOWN( key[k] == c + n * r );
        CPPAD_ASSERT_UNKNOWN( r < m && c < n );
        if( r != r_previous || c != c_previous)
        {   row.push_back(r);
            col.push_back(c);
        }
        r_previous = r;
        c_previous = c;
    }
}
extern size_t global_seed;
// ----------------------------------------------------------------------------
// available_sparse_jacobian
bool available_sparse_jacobian(void)
{   using CppAD::vector;
    // cppadcg assumes that that size = 10, m = 2 * size; see ../main.cpp
    size_t n      = 10;
    size_t m      = 2 * n;
    size_t repeat = 1;
    vector<size_t> row, col;
    choose_row_col_sparse_jacobian(global_seed, n, m, row, col);

    vector<double> x(n);
    size_t K = row.size();
    vector<double> jacobian(K);
    size_t         n_color;
    return link_sparse_jacobian(n, repeat, m, row, col, x, jacobian, n_color);
}
// ----------------------------------------------------------------------------
// correct_sparse_jacobian
bool correct_sparse_jacobian(bool is_package_double)
{   using CppAD::vector;
    // cppadcg assumes that that size = 10, m = 2 * size; see ../main.cpp
    size_t n      = 10;
    size_t m      = 2 * n;
    size_t repeat = 1;
    bool ok       = true;
    double eps    = 10. * CppAD::numeric_limits<double>::epsilon();
    vector<size_t> row, col;
    choose_row_col_sparse_jacobian(global_seed, n, m, row, col);

    size_t K = row.size();
    // The double package assumes jacobian.size() >= m
    CPPAD_ASSERT_UNKNOWN( K >= m );
    vector<double> x(n);
    vector<double> jacobian(K);
    size_t         n_color;
    link_sparse_jacobian(n, repeat, m, row, col, x, jacobian, n_color);

    if( is_package_double)
    {
        // check f(x)
        size_t order = 0;
        vector<double> check(m);
        CppAD::sparse_jac_fun<double>(m, n, x, row, col, order, check);
        for(size_t i = 0; i < m; i++)
            ok &= CppAD::NearEqual(check[i], jacobian[i], eps, eps);

        return ok;
    }
    // check f'(x)
    size_t order = 1;
    vector<double> check(K);
    CppAD::sparse_jac_fun<double>(m, n, x, row, col, order, check);
    for(size_t k = 0; k < K; k++)
        ok &= CppAD::NearEqual(check[k], jacobian[k], eps, eps);

    return ok;
}
// ----------------------------------------------------------------------------
// speed_sparse_jacobian
void speed_sparse_jacobian(size_t size, size_t repeat)
{   using CppAD::vector;
    // cppadcg assumes that m = 2 * size; see ../main.cpp
    size_t n   = size;
    size_t m   = 2 * n;
    //
    static size_t previous_size = 0;
    static vector<size_t> row, col;
    //
    // free statically allocated memory
    if( size == 0 && repeat == 0 )
    {   row.clear();
        col.clear();
        previous_size = size;
        return;
    }

    if( size != previous_size)
    {   choose_row_col_sparse_jacobian(global_seed, n, m, row, col);
        previous_size = size;
    }

    // note that cppad/sparse_jacobian.cpp assumes that x.size()
    // is the size corresponding to this test
    vector<double> x(n);
    size_t K = row.size();
    vector<double> jacobian(K);
    size_t         n_color;
    link_sparse_jacobian(n, repeat, m, row, col, x, jacobian, n_color);
    return;
}
// ----------------------------------------------------------------------------
// info_sparse_jacobian
void info_sparse_jacobian(size_t size, size_t& n_color)
{   using CppAD::vector;
    // cppadcg assumes that m = 2 * size; see ../main.cpp
    size_t n      = size;
    size_t m      = 2 * n;
    size_t repeat = 1;
    vector<size_t> row, col;
    choose_row_col_sparse_jacobian(global_seed, n, m, row, col);

    // note that cppad/sparse_jacobian.cpp assumes that x.size()
    // is the size corresponding to this test
    vector<double> x(n);
    size_t K = row.size();
    vector<double> jacobian(K);
    link_sparse_jacobian(n, repeat, m, row, col, x, jacobian, n_color);
    return;
}
