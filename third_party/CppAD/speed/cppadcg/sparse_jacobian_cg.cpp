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
$begin cppadcg_sparse_jacobian_cg.cpp$$
$spell
    const
    namespace
    work work
    jac
    CppAD
    cppad
    cppadcg
    cg
    hpp
    bool
    typedef
    endif
    std
    cout
    endl
    Jacobian
    nnz
    subgraph
    vec
$$

$section Cppadcg Speed: Source Generation: Sparse Jacobian$$

$head Syntax$$
$codei%sparse_jacobian_cg(%subgraph%, %optimize%, %seed%, %size_vec%)%$$

$head Purpose$$
This program generates C++ source code that computes the Jacobian of the
function $cref sparse_jac_fun$$.

$head subgraph$$
If $icode subgraph$$ is true (false),
the generated source code uses (does not use) $cref subgraph_jac_rev$$.

$head optimize$$
If $icode optimize$$ is true (false),
the AD function object is optimized (is not optimized)
before the source code is generated.

$head seed$$
Is the random number seed used during the choice of
row and column vectors in the sparse Jacobian.

$head size_vec$$
For $icode%i% = 0, %...%, %size_vec%.size()-1%$$,
$icode%size_vec%[%i%]%$$ is a positive integer specifying the
dimension of the domain space for the function.

$head sparse_jacobian.c$$
The source code is written to the file
$code sparse_jacobian.c$$ in the current working directory.
The corresponding function call has the following syntax:
$codei%
     %flag% = sparse_jacobian_c(
            %subgraph%, %optimize%, %seed%, %size%, %nnz%, %x%, %y%
    )
%$$
see $cref cppadcg_sparse_jacobian.c$$.

$head choose_row_col$$
The row vector $icode row$$ and column vector $icode col$$
that define the sparsity pattern are be determined by calling
$codei%
    choose_row_col_sparse_jacobian(%seed%, %n%, %m%, %row%, %col%)
%$$
where $icode seed$$ is the random number seed,
$icode n$$ is the size,
and $icode m$$ is $codei%2*%size%$$.


$head Implementation$$
$srccode%cpp% */
# include <cppad/cg/cppadcg.hpp>
# include <cppad/speed/sparse_jac_fun.hpp>
//
extern void choose_row_col_sparse_jacobian(size_t seed,
    size_t n, size_t m, CppAD::vector<size_t>& row, CppAD::vector<size_t>& col
);

namespace {

}

void sparse_jacobian_cg(
    bool subgraph                         ,
    bool optimize                         ,
    size_t seed                           ,
    const CppAD::vector<size_t>& size_vec )
{
    using CppAD::vector;
    typedef vector<size_t>          s_vector;
    typedef CppAD::cg::CG<double>   c_double;
    typedef CppAD::AD<c_double>     ac_double;
    typedef vector<c_double>        c_vector;
    typedef vector<ac_double>       ac_vector;
    //
    // optimization options: no conditional skips or compare operators
    std::string optimize_options =
        "no_conditional_skip no_compare_op no_print_for_op";
    //
    // Open file sparse_jacobian.c were souce code will be written
    std::fstream fs;
    fs.open("sparse_jacobian.c", std::fstream::out);
    fs << "# include <assert.h>\n";
    fs << "# include <math.h>\n";
    //
    // -----------------------------------------------------------------------
    // loop over sizes
    // -----------------------------------------------------------------------
    size_t n_size = size_vec.size();
    for(size_t i = 0; i < n_size; ++i)
    {
        size_t n     = size_vec[i]; // number of independent variables
        size_t m     = 2 * n;       // number of dependent variables
        //
        // determine row and colunn vectors in sparsity pattern
        s_vector row, col;
        choose_row_col_sparse_jacobian(seed, n, m, row, col);
        //
        ac_vector ac_x(n);          // AD domain space vector
        ac_vector ac_y(m);          // AD range space vector y = f(x)
        //
        // declare sparsity pattern
        CppAD::sparse_rc<s_vector>  sparsity;
        //
        // declare subset where Jacobian is evaluated
        // equal to entire sparsity pattern and using c_double
        CppAD::sparse_rc<s_vector> subset_pattern;
        size_t nr  = m;
        size_t nc  = n;
        size_t nnz = row.size();
        subset_pattern.resize(nr, nc, nnz);
        for(size_t k = 0; k < nnz; k++)
            subset_pattern.set(k, row[k], col[k]);
        CppAD::sparse_rcv<s_vector, c_vector> c_subset( subset_pattern );
        const c_vector& c_subset_val( c_subset.val() );
        //
        // coloring method
        std::string coloring = "cppad";
        //
        // maximum number of colors at once
        size_t group_max = 1;
        //
        // do not even record comparison operators
        size_t abort_op_index = 0;
        bool record_compare   = false;
        //
        // values of independent variables do not matter
        for(size_t j = 0; j < n; j++)
            ac_x[j] = ac_double( double(j) / double(n) );
        //
        // declare independent variables
        CppAD::Independent(ac_x, abort_op_index, record_compare);
        //
        // AD computation of f(x) (order zero derivative is function value)
        size_t order = 0;
        CppAD::sparse_jac_fun<ac_double>(m, n, ac_x, row, col, order, ac_y);
        //
        // create function object f : x -> y
        CppAD::ADFun<c_double> c_f;
        c_f.Dependent(ac_x, ac_y);
        //
        if( optimize )
            c_f.optimize(optimize_options);
        //
        // source code generator used for sparse_jacobian_c(x) = d/dx f(x)
        CppAD::cg::CodeHandler<double> code_handler;
        //
        // declare the independent variables in sparse_jacobian_c
        c_vector c_x(n);
        code_handler.makeVariables(c_x);
        //
        // evaluate sparse sparse jacobian as a function of c_x
        if( subgraph )
        {   // user reverse mode becasue forward not yet implemented
            c_f.subgraph_jac_rev(c_x, c_subset);
        }
        else
        {
            // calculate the Jacobian sparsity pattern for this function
            // using forward mode
            bool transpose     = false;
            bool internal_bool = false;
            bool dependency    = false;
            CPPAD_ASSERT_UNKNOWN( n == c_f.Domain() )
            CppAD::sparse_rc<s_vector> identity;
            identity.resize(n, n, n);
            for(size_t k = 0; k < n; k++)
                identity.set(k, k, k);
            c_f.for_jac_sparsity(
                identity, transpose, dependency, internal_bool, sparsity
            );
            //
            // structure that holds some of the work done by sparse_jac_for
            CppAD::sparse_jac_work work;
            //
            // calculate the Jacobian at this x
            c_f.sparse_jac_for(
                group_max, c_x, c_subset, sparsity, coloring, work
            );
        }
        //
        // set the dependent variables in sparse_jacobiain(x)
        c_vector c_y(nnz);
        for(size_t k = 0; k < nnz; k++)
            c_y[k] = c_subset_val[k];
        //
        // Mapping from variables in this program to variables in source_code
        // independent variable = x
        // dependent variable   = y
        // temporary variable   = v
        CppAD::cg::LanguageC<double> langC("double");
        CppAD::cg::LangCDefaultVariableNameGenerator<double> nameGen;
        //
        // generate the source code
        std::ostringstream source_code;
        code_handler.generateCode(source_code, langC, c_y, nameGen);
        //
        // number of temporary variables
        size_t nv = code_handler.getTemporaryVariableCount();
        //
        // wrap the string generated by code_handler into a function
        // sparse_jacobian_<size>[_opt][_sub](x, y)
        std::string name = "sparse_jacobian_" + CppAD::to_string(size_vec[i]);
        std::string source_str;
        source_str +=
            "// " + name + "\n"
            "static void " + name + "(int nnz, const double* x, double* y)\n"
            "{\n"
            "\tassert( nnz == " + CppAD::to_string(nnz) + " );\n"
        ;
        if( nv > 0 )
            source_str += "\tdouble v[" + CppAD::to_string(nv) + "];\n";
        source_str += "// Begin code generated by CppADCodeGen\n";
        source_str += source_code.str();
        source_str +=
            "// End code generated by CppADCodeGen\n"
            "}\n"
        ;
        fs << source_str;
    }
    //
    std::string subgraph_str = "0";
    if( subgraph )
        subgraph_str = "1";
    std::string optimize_str = "0";
    if( optimize )
        optimize_str = "1";
    //
    // sparse_jacobian_c(subgraph, optimize, size, nnz, x, y)
    fs <<
    "\nint sparse_jacobian_c(\n"
    "\tint subgraph    ,\n"
    "\tint optimize    ,\n"
    "\tint seed        ,\n"
    "\tint size        ,\n"
    "\tint nnz         ,\n"
    "\tconst double* x ,\n"
    "\tdouble* y       )\n"
    "{\tif( subgraph != " + subgraph_str + ")\n"
    "\t\treturn 1;\n"
    "\tif( optimize != " + optimize_str + ")\n"
    "\t\treturn 1;\n"
    "\tif( seed != " + CppAD::to_string(seed) + ")\n"
    "\t\treturn 1;\n"
    "\tswitch( size )\n"
    "\t{\n"
    ;
    for(size_t i = 0; i < n_size; ++i)
    {   std::string size_i = CppAD::to_string(size_vec[i]);
        fs << "\t\tcase " + size_i + ":\n";
        fs << "\t\tsparse_jacobian_" + size_i + "(nnz, x, y);\n";
        fs << "\t\tbreak;\n\n";
    }
    fs <<
    "\t\tdefault:\n"
    "\t\treturn 2;\n"
    "\t}\n"
    "\treturn 0;\n"
    "}\n"
    ;
    fs.close();
    //
    return;
}
/* %$$
$end
*/
