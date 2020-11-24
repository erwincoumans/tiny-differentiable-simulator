/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
 *    Copyright (C) 2020 Joao Leal
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */
#include <iosfwd>
#include <cppad/cg.hpp>
#include <cppad/cg/lang/dot/dot.hpp>

using namespace CppAD;
using namespace CppAD::cg;

int main() {
    // use a special object for source code generation
    using CGD = CG<double>;
    using ADCG = AD<CGD>;

    /***************************************************************************
     *                               the model
     **************************************************************************/

    // independent variable vector
    CppAD::vector<ADCG> x(2);
    x[0] = 2.;
    x[1] = 3.;
    Independent(x);

    // dependent variable vector 
    CppAD::vector<ADCG> y(1);

    // the model
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;

    ADFun<CGD> fun(x, y); // the model tape

    /***************************************************************************
     *                       Generate the dot file
     **************************************************************************/

    CodeHandler<double> handler;

    CppAD::vector<CGD> xv(x.size());
    handler.makeVariables(xv);

    CppAD::vector<CGD> vals = fun.Forward(0, xv);

    LanguageDot<double> langDot;
    LangCDefaultVariableNameGenerator<double> nameGen;

    std::ofstream texfile;
    texfile.open("algorithm.dot");

    handler.generateCode(texfile, langDot, vals, nameGen);

    texfile.close();

    /***************************************************************************
     *                        Generate an image
     **************************************************************************/
#ifdef GRAPHVIZ_DOT_PATH
    std::string dir = system::getWorkingDirectory();

    system::callExecutable(GRAPHVIZ_DOT_PATH,{"-Tpng", "-oalgorithm.png", system::createPath(dir, "algorithm.dot")});
#endif
}