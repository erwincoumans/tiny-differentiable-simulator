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
$begin fabs.cpp$$
$spell
    fabs
    abs
$$

$section AD Absolute Value Function: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

bool fabs(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]     = 0.;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 6;
    CPPAD_TESTVECTOR(AD<double>) y(m);
    y[0]     = fabs(x[0] - 1.);
    y[1]     = fabs(x[0]);
    y[2]     = fabs(x[0] + 1.);
    //
    y[3]     = fabs(x[0] - 1.);
    y[4]     = fabs(x[0]);
    y[5]     = fabs(x[0] + 1.);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    // check values
    ok &= (y[0] == 1.);
    ok &= (y[1] == 0.);
    ok &= (y[2] == 1.);
    //
    ok &= (y[3] == 1.);
    ok &= (y[4] == 0.);
    ok &= (y[5] == 1.);

    // forward computation of partials w.r.t. a positive x[0] direction
    size_t p = 1;
    CPPAD_TESTVECTOR(double) dx(n), dy(m);
    dx[0] = 1.;
    dy    = f.Forward(p, dx);
    ok  &= (dy[0] == - dx[0]);
    ok  &= (dy[1] ==   0.   ); // used to be (dy[1] == + dx[0]);
    ok  &= (dy[2] == + dx[0]);
    //
    ok  &= (dy[3] == - dx[0]);
    ok  &= (dy[4] ==   0.   ); // used to be (dy[1] == + dx[0]);
    ok  &= (dy[5] == + dx[0]);

    // forward computation of partials w.r.t. a negative x[0] direction
    dx[0] = -1.;
    dy    = f.Forward(p, dx);
    ok  &= (dy[0] == - dx[0]);
    ok  &= (dy[1] ==   0.   ); // used to be (dy[1] == - dx[0]);
    ok  &= (dy[2] == + dx[0]);
    //
    ok  &= (dy[3] == - dx[0]);
    ok  &= (dy[4] ==   0.   ); // used to be (dy[1] == - dx[0]);
    ok  &= (dy[5] == + dx[0]);

    // reverse computation of derivative of y[0]
    p    = 1;
    CPPAD_TESTVECTOR(double)  w(m), dw(n);
    w[0] = 1.; w[1] = 0.; w[2] = 0.; w[3] = 0.; w[4] = 0.; w[5] = 0.;
    dw   = f.Reverse(p, w);
    ok  &= (dw[0] == -1.);

    // reverse computation of derivative of y[1]
    w[0] = 0.; w[1] = 1.;
    dw   = f.Reverse(p, w);
    ok  &= (dw[0] == 0.);

    // reverse computation of derivative of y[5]
    w[1] = 0.; w[5] = 1.;
    dw   = f.Reverse(p, w);
    ok  &= (dw[0] == 1.);

    // use a VecAD<Base>::reference object with abs and fabs
    CppAD::VecAD<double> v(1);
    AD<double> zero(0);
    v[zero]           = -1;
    AD<double> result = fabs(v[zero]);
    ok    &= NearEqual(result, 1., eps99, eps99);
    result = fabs(v[zero]);
    ok    &= NearEqual(result, 1., eps99, eps99);

    return ok;
}

// END C++
