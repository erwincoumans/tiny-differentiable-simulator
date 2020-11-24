# ifndef CPPAD_LOCAL_COLOR_GENERAL_HPP
# define CPPAD_LOCAL_COLOR_GENERAL_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/configure.hpp>
# include <cppad/local/cppad_colpack.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file color_general.hpp
Coloring algorithm for a general sparse matrix.
*/
// --------------------------------------------------------------------------
/*!
Determine which rows of a general sparse matrix can be computed together;
i.e., do not have non-zero entries with the same column index.

\tparam SizeVector
is a simple vector class with elements of type size_t.

\tparam SetVector
is vector_of_sets class.

\param pattern [in]
Is a representation of the sparsity pattern for the matrix.

\param row [in]
is a vector specifying which row indices to compute.

\param col [in]
is a vector, with the same size as row,
that specifies which column indices to compute.
For each  valid index k, the index pair
<code>(row[k], col[k])</code> must be present in the sparsity pattern.
It may be that some entries in the sparsity pattern do not need to be computed;
i.e, do not appear in the set of
<code>(row[k], col[k])</code> entries.

\param color [out]
is a vector with size m.
The input value of its elements does not matter.
Upon return, it is a coloring for the rows of the sparse matrix.
\n
\n
If for some i, <code>color[i] == m</code>, then
the i-th row does not appear in the vector row.
Otherwise, <code>color[i] < m</code>.
\n
\n
Suppose two differen rows, <code>i != r</code> have the same color and
column index j is such that both of the pairs
<code>(i, j)</code> and <code>(r, j)</code> appear in the sparsity pattern.
It follows that neither of these pairs appear in the set of
<code>(row[k], col[k])</code> entries.
\n
\n
This routine tries to minimize, with respect to the choice of colors,
the maximum, with respct to k, of <code>color[ row[k] ]</code>
(not counting the indices k for which row[k] == m).
*/
template <class SetVector, class SizeVector>
void color_general_cppad(
    const SetVector&        pattern ,
    const SizeVector&       row     ,
    const SizeVector&       col     ,
    CppAD::vector<size_t>&  color   )
{
    size_t K = row.size();
    size_t m = pattern.n_set();
    size_t n = pattern.end();

    CPPAD_ASSERT_UNKNOWN( size_t( col.size() )   == K );
    CPPAD_ASSERT_UNKNOWN( size_t( color.size() ) == m );

    // We define the set of rows, columns, and pairs that appear
    // by the set ( row[k], col[k] ) for k = 0, ... , K-1.

    // initialize rows that appear
    CppAD::vector<bool> row_appear(m);
    for(size_t i = 0; i < m; i++)
            row_appear[i] = false;

    // rows and columns that appear
    SetVector c2r_appear, r2c_appear;
    c2r_appear.resize(n, m);
    r2c_appear.resize(m, n);
    for(size_t k = 0;  k < K; k++)
    {   CPPAD_ASSERT_KNOWN( pattern.is_element(row[k], col[k]) ,
            "color_general_cppad: requesting value for a matrix element\n"
            "that is not in the matrice's sparsity pattern.\n"
            "Such a value must be zero."
        );
        row_appear[ row[k] ] = true;
        c2r_appear.post_element(col[k], row[k]);
        r2c_appear.post_element(row[k], col[k]);
    }
    // process posts
    for(size_t j = 0; j < n; ++j)
        c2r_appear.process_post(j);
    for(size_t i = 0; i < m; ++i)
        r2c_appear.process_post(i);

    // for each column, which rows are non-zero and do not appear
    SetVector not_appear;
    not_appear.resize(n, m);
    for(size_t i = 0; i < m; i++)
    {   typename SetVector::const_iterator pattern_itr(pattern, i);
        size_t j = *pattern_itr;
        while( j != pattern.end() )
        {   if( ! c2r_appear.is_element(j , i) )
                not_appear.post_element(j, i);
            j = *(++pattern_itr);
        }
    }
    // process posts
    for(size_t j = 0; j < n; ++j)
        not_appear.process_post(j);

    // initial coloring
    color.resize(m);
    size_t ell = 0;
    for(size_t i = 0; i < m; i++)
    {   if( row_appear[i] )
            color[i] = ell++;
        else
            color[i] = m;
    }
    /*
    See GreedyPartialD2Coloring Algorithm Section 3.6.2 of
    Graph Coloring in Optimization Revisited by
    Assefaw Gebremedhin, Fredrik Maane, Alex Pothen

    The algorithm above was modified (by Brad Bell) to take advantage of the
    fact that only the entries (subset of the sparsity pattern) specified by
    row and col need to be computed.
    */
    CppAD::vector<bool> forbidden(m);
    for(size_t i = 1; i < m; i++) // for each row that appears
    if( color[i] < m )
    {
        // initial all colors as ok for this row
        // (value of forbidden for ell > initial color[i] does not matter)
        for(ell = 0; ell <= color[i]; ell++)
            forbidden[ell] = false;

        // -----------------------------------------------------
        // Forbid colors for which this row would destroy results:
        //
        // for each column that is non-zero for this row
        typename SetVector::const_iterator pattern_itr(pattern, i);
        size_t j = *pattern_itr;
        while( j != pattern.end() )
        {   // for each row that appears with this column
            typename SetVector::const_iterator c2r_itr(c2r_appear, j);
            size_t r = *c2r_itr;
            while( r != c2r_appear.end() )
            {   // if this is not the same row, forbid its color
                if( (r < i) & (color[r] < m) )
                    forbidden[ color[r] ] = true;
                r = *(++c2r_itr);
            }
            j = *(++pattern_itr);
        }


        // -----------------------------------------------------
        // Forbid colors that destroy results needed for this row.
        //
        // for each column that appears with this row
        typename SetVector::const_iterator r2c_itr(r2c_appear, i);
        j = *r2c_itr;
        while( j != r2c_appear.end() )
        {   // For each row that is non-zero for this column
            // (the appear rows have already been checked above).
            typename SetVector::const_iterator not_itr(not_appear, j);
            size_t r = *not_itr;
            while( r != not_appear.end() )
            {   // if this is not the same row, forbid its color
                if( (r < i) & (color[r] < m) )
                    forbidden[ color[r] ] = true;
                r = *(++not_itr);
            }
            j = *(++r2c_itr);
        }

        // pick the color with smallest index
        ell = 0;
        while( forbidden[ell] )
        {   ell++;
            CPPAD_ASSERT_UNKNOWN( ell <= color[i] );
        }
        color[i] = ell;
    }
    return;
}

# if CPPAD_HAS_COLPACK
/*!
Colpack version of determining which rows of a sparse matrix
can be computed together.

\copydetails color_general
*/
template <class SetVector, class SizeVector>
void color_general_colpack(
    const SetVector&        pattern ,
    const SizeVector&       row     ,
    const SizeVector&       col     ,
    CppAD::vector<size_t>&  color   )
{
    size_t m = pattern.n_set();
    size_t n = pattern.end();

    // Determine number of non-zero entries in each row
    CppAD::vector<size_t> n_nonzero(m);
    size_t n_nonzero_total = 0;
    for(size_t i = 0; i < m; i++)
    {   n_nonzero[i] = 0;
        typename SetVector::const_iterator pattern_itr(pattern, i);
        size_t j = *pattern_itr;
        while( j != pattern.end() )
        {   n_nonzero[i]++;
            j = *(++pattern_itr);
        }
        n_nonzero_total += n_nonzero[i];
    }

    // Allocate memory and fill in Adolc sparsity pattern
    CppAD::vector<unsigned int*> adolc_pattern(m);
    CppAD::vector<unsigned int>  adolc_memory(m + n_nonzero_total);
    size_t i_memory = 0;
    for(size_t i = 0; i < m; i++)
    {   adolc_pattern[i]    = adolc_memory.data() + i_memory;
        CPPAD_ASSERT_KNOWN(
            std::numeric_limits<unsigned int>::max() >= n_nonzero[i],
            "Matrix is too large for colpack"
        );
        adolc_pattern[i][0] = static_cast<unsigned int>( n_nonzero[i] );
        typename SetVector::const_iterator pattern_itr(pattern, i);
        size_t j = *pattern_itr;
        size_t k = 1;
        while(j != pattern.end() )
        {
            CPPAD_ASSERT_KNOWN(
                std::numeric_limits<unsigned int>::max() >= j,
                "Matrix is too large for colpack"
            );
            adolc_pattern[i][k++] = static_cast<unsigned int>( j );
            j = *(++pattern_itr);
        }
        CPPAD_ASSERT_UNKNOWN( k == 1 + n_nonzero[i] );
        i_memory += k;
    }
    CPPAD_ASSERT_UNKNOWN( i_memory == m + n_nonzero_total );

    // Must use an external routine for this part of the calculation because
    // ColPack/ColPackHeaders.h has as 'using namespace std' at global level.
    cppad_colpack_general(color, m, n, adolc_pattern);

    return;
}
# endif // CPPAD_HAS_COLPACK

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
