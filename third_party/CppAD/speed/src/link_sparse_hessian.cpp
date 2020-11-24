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
# include <cppad/speed/sparse_hes_fun.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/index_sort.hpp>

# include "link_sparse_hessian.hpp"

/*!
\{
\file link_sparse_hessian.cpp
Defines and implement sparse Hessian speed link to package specific code.
*/
namespace { // BEGIN_EMPTY_NAMESPACE

using CppAD::vector;

/*
$begin choose_row_col$$
$spell
    Namespace
$$

$section Randomly choose Hessian row and column indices$$

$head Namespace$$
This function is in the empty namespace; i.e., it can only be accessed
by functions in this file.

$head Prototype$$
$srcthisfile%
    0%// BEGIN_choose_row_col%// END_choose_row_col%1
%$$

$head n$$
is number of rows and columns in the Hessian.

$head row$$
The input size and elements of row do not matter.
Upon return it is the chosen row indices.

$head col$$
The input size and elements of col do not matter.
Upon return it is the chosen column indices.

$head Order$$
The return row and column values are in row major order.

$head Diagonal$$
The diagonal is included in the result; i.e., for each $icode i$$
between zero and $icode%n%-1%$$, there is a $icode k$$ such that
$codei%
    %row%[%k%] == %col%[%k%] == %i%
%$$.

$end
*/
// BEGIN_choose_row_col
void choose_row_col(
    size_t                 n   ,
    CppAD::vector<size_t>& row ,
    CppAD::vector<size_t>& col )
// END_choose_row_col
{   using CppAD::vector;

    // maximum number of entries per row
    size_t max_per_row = 5;

    // random choices for each row, and correspond sort order
    vector<double> random_01(max_per_row);
    vector<size_t> random_index(max_per_row), order(max_per_row);

    // generate the row and column indices
    row.resize(0);
    col.resize(0);
    for(size_t i = 0; i < n; i++)
    {   // generate max_per_row random values between 0 and 1
        CppAD::uniform_01(max_per_row, random_01);

        // make sure the diagnoal is in the result because
        // sparse_hes_fun requires it
        random_index[0] = i;

        // convert to column indices between 0 and i
        for(size_t k = 1; k < max_per_row; ++k)
        {   random_index[k] = size_t( random_01[k] * double(i) );
            random_index[k] = std::min(random_index[k], i);
        }

        // determine the sort order for the indices
        CppAD::index_sort(random_index, order);

        // set the indices for this row
        for(size_t k = 0; k < max_per_row; k++)
        {   size_t j = random_index[ order[k] ];
            bool ok = k == 0;
            if( ! ok )
                ok = random_index[ order[k-1] ] < j;
            if( ok )
            {   row.push_back(i);
                col.push_back(j);
            }
        }
    }
}
} // END_EMPTY_NAMESPACE


/*
$begin available_sparse_hessian$$
$spell
    Namespace
    CppAD
    bool
$$

$section Is Sparse Hessian Speed Test Available$$

$head Namespace$$
This function is in the global namespace, not the CppAD namespace.

$head Syntax$$
$icode%available% = available_sparse_hessian()%$$

$head available$$
If the spare Hessian speed test is available for this package,
the $code bool$$ value $icode available$$ is true.
Otherwise it is false.

$end
*/
bool available_sparse_hessian(void)
{
    size_t n      = 2;
    size_t repeat = 1;
    vector<double> x(n);
    vector<size_t> row, col;
    choose_row_col(n, row, col);
    size_t K = row.size();
    vector<double> hessian(K);

    size_t n_color;
    return link_sparse_hessian(n, repeat, row, col, x, hessian, n_color);
}
/*
$begin correct_sparse_hessian$$
$spell
    Namespace
    CppAD
    bool
$$

$section Does Sparse Hessian Pass Correctness Test$$

$head Namespace$$
This function is in the global namespace, not the CppAD namespace.

$head Syntax$$
$icode%ok% = correct_sparse_hessian(%is_package_double%)%$$

$head is_package_double$$
If the $code bool$$ value $code is_package_double$$ is true,
we are checking function values.
Otherwise, we are checking derivative values.

$head ok$$
If the spare Hessian correctness test passed,
the $code bool$$ value $icode ok$$ is true.
Otherwise it is false.

$end
*/
bool correct_sparse_hessian(bool is_package_double)
{
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    size_t n      = 10;
    size_t repeat = 1;
    vector<double> x(n);
    vector<size_t> row, col;
    choose_row_col(n, row, col);
    size_t K = row.size();
    vector<double> hessian(K);
# ifndef NDEBUG
    for(size_t k = 0; k < K; k++)
        CPPAD_ASSERT_UNKNOWN( col[k] <= row[k] );
# endif

    // The double package assumes hessian.size() >= 1
    CPPAD_ASSERT_UNKNOWN( K >= 1 );
    size_t n_color;
    link_sparse_hessian(n, repeat, row, col, x, hessian, n_color);

    size_t order, size;
    if( is_package_double)
    {   order = 0;  // check function value
        size  = 1;
    }
    else
    {   order = 2;     // check hessian value
        size  = K;
    }
    CppAD::vector<double> check(size);
    CppAD::sparse_hes_fun<double>(n, x, row, col, order, check);
    bool ok = true;
    size_t k;
    for(k = 0; k < size; k++)
        ok &= CppAD::NearEqual(check[k], hessian[k], eps99, eps99);

    return ok;
}
/*
$begin speed_sparse_hessian$$
$spell
    Namespace
    CppAD
$$

$section Drive Sparse Hessian for Speed Testing$$

$head Namespace$$
This function is in the global namespace, not the CppAD namespace.

$head Syntax$$
$codei%speed_sparse_hessian(%size%, %repeat%)%$$

$head size$$
This $code size_t$$ value
is the dimension of the argument space for function we are taking
the Hessian of.

$head repeat$$
This $code size_t$$ value
is the number of times to repeat the speed test.

$end
*/
void speed_sparse_hessian(size_t size, size_t repeat)
{
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

    size_t n = size;
    vector<double> x(n);
    if( size != previous_size )
    {   choose_row_col(n, row, col);
        previous_size = size;
    }
    size_t K = row.size();
    vector<double> hessian(K);
# ifndef NDEBUG
    for(size_t k = 0; k < K; k++)
        CPPAD_ASSERT_UNKNOWN( col[k] <= row[k] );
# endif

    // note that cppad/sparse_hessian.cpp assumes that x.size() == size
    size_t n_color;
    link_sparse_hessian(n, repeat, row, col, x, hessian, n_color);
    return;
}
/*
$begin info_sparse_hessian$$
$spell
    Namespace
    CppAD
$$

$section Sparse Hessian Speed Test Information$$

$head Namespace$$
This function is in the global namespace, not the CppAD namespace.

$head Syntax$$
$codei%info_spares_hessian(%size%, %n_color%)%$$

$head size$$
This $code size_t$$ value is equal to
$cref/size/speed_sparse_hessian/size/$$
in the corresponding call to $code speed_sparse_hessian$$.

$head n_color$$
The input value of this $icode size_t$$ does not matter.
Upon return, it is the value $cref/n_color/link_sparse_hessian/n_color/$$
returned by the corresponding call to $code link_sparse_hessian$$.

$end
*/
void info_sparse_hessian(size_t size, size_t& n_color)
{   size_t n      = size;
    size_t repeat = 1;
    vector<size_t> row, col;
    choose_row_col(n, row, col);

    // note that cppad/speed/sparse_hessian.cpp assumes that x.size()
    // is the size corresponding to this test
    vector<double> x(n);
    size_t K = row.size();
    vector<double> hessian(K);
    link_sparse_hessian(n, repeat, row, col, x, hessian, n_color);
    return;
}
