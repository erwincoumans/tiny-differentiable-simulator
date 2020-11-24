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
$begin cppadcg_det_minor_cg.cpp$$
$spell
    Cppadcg
    typedef
    cppad
    CppAD
    det
    hpp
    const
    bool
    std
    cg
    vec
$$

$section Cppadcg Speed: Source Generation: Gradient of Determinant by Minor$$

$head Syntax$$
$codei%det_minor_cg(%size_vec%)%$$

$head Purpose$$
This program generates C++ source code that computes the derivative of the
determinant of a square matrix.

$head n_size$$
The positive integer $icode n_size$$
is the size of the vector $icode size_vec$$.
This is the number of sizes that the source code is generated for.

$head size_vec$$
For $icode%i% = 1, %...%, %n_size%$$,
$icode%size_vec%[%i%]%$$ is a positive integer specifying the
row and column dimension of the matrix.

$head det_minor_grad.c$$
The source code is written to the file
$code det_minor_grad.c$$ in the current working directory.
The corresponding function call has the following syntax:
$icode%
     %flag% = det_minor_grad_c(%optimize%, %size%, %x%, %y%)
%$$
see $cref cppadcg_det_minor_grad.c$$.

$head Implementation$$
$srccode%cpp% */
# include <cppad/cg/cppadcg.hpp>
# include <cppad/speed/det_by_minor.hpp>

void det_minor_cg(const CppAD::vector<size_t>& size_vec)
{   // --------------------------------------------------------------------
    // optimization options: no conditional skips or compare operators
    std::string optimize_options =
        "no_conditional_skip no_compare_op no_print_for_op";
    // --------------------------------------------------------------------
    // typedefs
    typedef CppAD::cg::CG<double>    c_double;
    typedef CppAD::AD<c_double>      ac_double;
    typedef CppAD::vector<c_double>  c_vector;
    typedef CppAD::vector<ac_double> ac_vector;

    // Open file det_minor_grad.cpp where source code will be written
    std::fstream fs;
    fs.open("det_minor_grad.c", std::fstream::out);

    // ----------------------------------------------------------------------
    // loop over sizes, optimize
    // ----------------------------------------------------------------------
    size_t n_size = size_vec.size();
    for(size_t i = 0; i < n_size; ++i)
    for(int opt = 0; opt < 2; ++opt)
    {   // object for computing determinant
        CppAD::det_by_minor<ac_double>   ac_det(size_vec[i]);

        // number of dependent variables in determinant
        size_t nd = 1;
        // number of independent variables
        size_t nx = size_vec[i] * size_vec[i];
        // determinant domain space vector
        ac_vector   ac_A(nx);
        // determinant range space vector
        ac_vector   ac_detA(nd);

        // values of independent variables do not matter
        for(size_t j = 0; j < nx; j++)
            ac_A[j] = ac_double( double(j) / double(nx) );

        // declare independent variables without comparison operators
        size_t abort_op_index = 0;
        bool record_compare   = false;
        CppAD::Independent(ac_A, abort_op_index, record_compare);

        // AD computation of the determinant
        ac_detA[0] = ac_det(ac_A);

        // create function object f : A -> detA
        CppAD::ADFun<c_double> c_f;
        c_f.Dependent(ac_A, ac_detA);

        if( opt == 1 )
            c_f.optimize(optimize_options);

        // source code generator used for det_minor_grad_c(x) = d/dx f(x)
        CppAD::cg::CodeHandler<double> code_handler;

        // declare the independent variables in det_minor_grad_c
        c_vector c_x(nx);
        code_handler.makeVariables(c_x);

        // declare the dependent variables in det_minor_grad_c
        size_t ny = nd * nx;
        c_vector c_y(ny);

        // evaluate the determinant as a function of c_x
        c_f.Forward(0, c_x);

        // evaluate the gradient using reverse mode
        CppAD::vector<c_double> c_w(nd);
        c_w[0] = c_double(1.0);
        c_y    = c_f.Reverse(1, c_w);

        // Mapping from variables in this program to variables in source_code
        // independent variable = x
        // dependent variable   = y
        // temporary variable   = v
        CppAD::cg::LanguageC<double> langC("double");
        CppAD::cg::LangCDefaultVariableNameGenerator<double> nameGen;

        // generate the source code
        std::ostringstream source_code;
        code_handler.generateCode(source_code, langC, c_y, nameGen);

        // number of temporary variables
        size_t nv = code_handler.getTemporaryVariableCount();

        // wrap the string generated by code_handler into a function
        // det_minor_grad_<size>_<opt>(x, y)
        std::string name = "det_minor_grad_" + CppAD::to_string(size_vec[i]);
        if( opt == 1 )
            name += "_opt";
        std::string source_str = "// " + name + "\n";
        source_str += "static void " + name + "(const double* x, double* y)\n";
        source_str += "{\n";
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
    // det_minor_grad_c(optimize, size, x, y)
    fs << "\nint det_minor_grad_c(\n";
    fs << "\tint optimize, int size, const double* x, double* y\n";
    fs << ")\n";
    fs << "{\tswitch( size )\n";
    fs << "\t{\n";
    for(size_t i = 0; i < n_size; ++i)
    {   std::string size_i = CppAD::to_string(size_vec[i]);
        fs << "\t\tcase " + size_i + ":\n";
        fs << "\t\tif( optimize )\n";
        fs << "\t\t\tdet_minor_grad_" + size_i + "_opt(x, y);\n";
        fs << "\t\telse\n";
        fs << "\t\t\tdet_minor_grad_" + size_i + "(x, y);\n";
        fs << "\t\tbreak;\n\n";
    }
    fs << "\t\tdefault:\n";
    fs << "\t\treturn 1;\n";
    fs << "\t}\n";
    fs << "\treturn 0;\n";
    fs << "}\n";
    fs.close();
    //
    return;
}
/* %$$
$end
*/
