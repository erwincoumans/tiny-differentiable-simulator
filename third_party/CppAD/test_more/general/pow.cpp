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
Old examples now just used for validation testing.
*/
# include <cppad/cppad.hpp>

namespace { // BEGIN empty namespace

bool PowTestOne(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    double x = 0.5;
    double y = 2.;
    CPPAD_TESTVECTOR(AD<double>) XY(n);
    XY[0]      = x;
    XY[1]      = y;

    // declare independent variables and start tape recording
    CppAD::Independent(XY);

    // range space vector
    size_t m = 3;
    CPPAD_TESTVECTOR(AD<double>) Z(m);
    Z[0] = CppAD::pow(XY[0], XY[1]);  // pow(variable, variable)
    Z[1] = CppAD::pow(XY[0], y);      // pow(variable, parameter)
    Z[2] = CppAD::pow(x,     XY[1]);  // pow(parameter, variable)

    // create f: XY -> Z and stop tape recording
    CppAD::ADFun<double> f(XY, Z);

    // check value
    double check = std::pow(x, y);
    size_t i;
    for(i = 0; i < m; i++)
        ok &= NearEqual(Z[i] , check, eps99, eps99);

    // forward computation of first partial w.r.t. x
    CPPAD_TESTVECTOR(double) dxy(n);
    CPPAD_TESTVECTOR(double) dz(m);
    dxy[0] = 1.;
    dxy[1] = 0.;
    dz    = f.Forward(1, dxy);
    check = y * std::pow(x, y-1.);
    ok   &= NearEqual(dz[0], check, eps99, eps99);
    ok   &= NearEqual(dz[1], check, eps99, eps99);
    ok   &= NearEqual(dz[2],    0., eps99, eps99);

    // forward computation of first partial w.r.t. y
    dxy[0] = 0.;
    dxy[1] = 1.;
    dz    = f.Forward(1, dxy);
    check = std::log(x) * std::pow(x, y);
    ok   &= NearEqual(dz[0], check, eps99, eps99);
    ok   &= NearEqual(dz[1],    0., eps99, eps99);
    ok   &= NearEqual(dz[2], check, eps99, eps99);

    // reverse computation of derivative of z[0] + z[1] + z[2]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0]  = 1.;
    w[1]  = 1.;
    w[2]  = 1.;
    dw    = f.Reverse(1, w);
    check = y * std::pow(x, y-1.);
    ok   &= NearEqual(dw[0], 2. * check, eps99, eps99);
    check = std::log(x) * std::pow(x, y);
    ok   &= NearEqual(dw[1], 2. * check, eps99, eps99);

    // use a VecAD<Base>::reference object with pow
    CppAD::VecAD<double> v(2);
    AD<double> zero(0);
    AD<double> one(1);
    v[zero]           = XY[0];
    v[one]            = XY[1];
    AD<double> result = CppAD::pow(v[zero], v[one]);
    ok               &= NearEqual(result, Z[0], eps99, eps99);

    return ok;
}

bool PowTestTwo(void)
{   bool ok = true;

    using CppAD::pow;
    using CppAD::exp;
    using namespace CppAD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();


    // independent variable vector, indices, values, and declaration
    CPPAD_TESTVECTOR(AD<double>) U(2);
    size_t s = 0;
    size_t t = 1;
    U[s]     = 2.;
    U[t]     = 3.;
    Independent(U);

    // dependent variable vector and indices
    CPPAD_TESTVECTOR(AD<double>) Z(2);
    size_t x = 0;
    size_t y = 1;


    // dependent variable values
    AD<double> u = exp(U[s]);        // u = exp(s)
    Z[x]         = pow(u, U[t]);     // x = exp(s * t)
    Z[y]         = pow(Z[x], u);     // y = exp( s * t * exp(s) )

    // create f: U -> Z and vectors used for derivative calculations
    ADFun<double> f(U, Z);
    CPPAD_TESTVECTOR(double) v( f.Domain() );
    CPPAD_TESTVECTOR(double) w( f.Range() );

    /*
    u_s  (s, t) = u
    u_t  (s, t) = 0
    y_s  (s, t) = (1 + s) t * u * y
    y_t  (s, t) = s * u * y
    y_st (s, t) = ( u + s * u ) * y
                    + ( t * u + s * t * u ) * s * u * y
    */

    // check values
    ok &= NearEqual(Z[x] , exp(2. * 3.), eps99, eps99);
    ok &= NearEqual(Z[y] , exp( 2. * 3. * exp(2.) ), eps99, eps99);

    // forward computation of partials w.r.t. s
    v[s] = 1.;
    v[t] = 0.;
    w = f.Forward(1, v);
    ok &= ( w[x] == U[t] * Z[x] );                   // dx/ds
    ok &= ( w[y] == (1. + U[s]) * U[t] * u * Z[y] ); // dy/ds

    // forward computation of partials w.r.t. t
    v[s] = 0.;
    v[t] = 1.;
    w = f.Forward(1, v);
    ok &= ( w[y] == U[s] * u * Z[y] );               // dy/dt

    // forward computation of second Taylor coefficient w.r.t. t
    v[t] = 1.;
    w    = f.Forward(1, v);
    v[t] = 0.;
    CPPAD_TESTVECTOR(double) f_tt = f.Forward(2, v);

    // forward computation of second Taylor coefficient w.r.t. s
    v[s] = 1.;
    w    = f.Forward(1, v);
    v[s] = 0.;
    CPPAD_TESTVECTOR(double) f_ss = f.Forward(2, v);

    // second Taylor coefficient w.r.t. direction r = (s,t)
    v[s] = 1.;
    v[t] = 1.;
    w    = f.Forward(1, v);
    v[s] = 0.;
    v[t] = 0.;
    CPPAD_TESTVECTOR(double) f_rr = f.Forward(2, v);

    // check second order partial of y
    ok &= NearEqual(
        f_rr[y] - f_ss[y] - f_tt[y],
        (1. + U[s]) * u * Z[y] +
            (1. + U[s]) * U[t] * u * U[s] * u * Z[y],
        eps99 ,
        eps99
    );

    return ok;
}

bool PowTestThree(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = 2.;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 4;
    CPPAD_TESTVECTOR(AD<double>) y(m);

    // some special cases
    y[0] = pow(x[0], 0.);
    y[1] = pow(0., x[0]);
    y[2] = pow(x[0], 1.);
    y[3] = pow(1., x[0]);

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    // check function values
    ok  &= (Value(y[0]) == 1.);
    ok  &= (Value(y[1]) == 0.);
    ok  &= (Value(y[2]) == Value(x[0]));
    ok  &= (Value(y[3]) == 1.);

    // forward computation of first partial w.r.t. x
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    ok   &= (dy[0] == 0.);
    ok   &= (dy[1] == 0.);
    ok   &= NearEqual(dy[2], 1., eps99, eps99);
    ok   &= (dy[3] == 0.);

    // reverse mode computation of derivative of y[0]+y[1]+y[2]+y[3]
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0] = 1.;
    w[1] = 1.;
    w[2] = 1.;
    w[3] = 1.;
    dw   = f.Reverse(1, w);
    ok  &= NearEqual(dw[0], 1., eps99, eps99);

    return ok;
}

bool PowTestFour(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    double x0 = -2;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 5;
    CPPAD_TESTVECTOR(AD<double>) y(m);

    // some special cases (skip zero raised to a negative power)
    y[0] = pow(1., x[0]);
    size_t i;
    for(i = 1; i < m; i++)
        y[i] = CppAD::pow(x[0], int(i-1) );   // pow(AD<double>, int)

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    ok  &= (Value(y[0]) == 1.);
    double check;
    for(i = 1; i < m; i++)
    {   check = std::pow(x0, double(i-1));
        ok   &= NearEqual(y[i], check, eps99, eps99);
    }

    // forward computation of first partial w.r.t. x
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    ok   &= (dy[0] == 0.);
    double sum = 0;
    for(i = 1; i < m; i++)
    {   if( i == 1 )
            check = 0.;
        else
            check = double(i-1) * std::pow(x0, double(i-2));
        ok   &= NearEqual(dy[i], check, eps99, eps99);
        sum  += check;
    }

    // reverse mode computation of derivative of y[0] + .. y[m-1];
    CPPAD_TESTVECTOR(double)  w(m);
    CPPAD_TESTVECTOR(double) dw(n);
    for(i = 0; i < m; i++)
        w[i] = 1.;
    dw   = f.Reverse(1, w);
    ok  &= NearEqual(dw[0], sum, eps99, eps99);

    return ok;
}
bool PowTestFive(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    double x0 = -1.;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(x);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) y(m);

    // case of zero raised to a positive integer power
    double e = 2.;
    y[0] = pow(x[0], int(e)); // use pow(AD<double>, int)

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(x, y);

    // check function value
    ok  &= (Value(y[0]) == pow(x0, e) );

    // forward computation of first partial w.r.t. x[1]
    double d1 = e * pow(x0, (e-1));
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dy    = f.Forward(1, dx);
    ok   &= NearEqual(dy[0], d1, eps99, eps99);

    // reverse mode computation of second partials
    // x.r.t. x[1],x[0]  and x[1], x[1]
    double d2 = e * (e-1) * pow(x0, (e-2));
    CPPAD_TESTVECTOR(double)   w(m);
    CPPAD_TESTVECTOR(double) ddw(2*n);
    w[0] = 1.;
    ddw  = f.Reverse(2, w);
    ok  &= NearEqual(ddw[0], d1, eps99, eps99);
    ok  &= NearEqual(ddw[1], d2, eps99, eps99);

    return ok;
}
bool PowTestSix(void)
{   bool ok = true;

    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 1;
    double x0 = 1.5;
    CPPAD_TESTVECTOR(AD<double>) x(n);
    x[0]      = x0;

    // domain space vector
    CPPAD_TESTVECTOR(AD< AD<double> >) X(n);
    X[0]      = x[0];

    // declare independent variables and start tape recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD< AD<double> >) Y(m);

    // case of AD< AD<double> > raised to a double power
    double e = 2.5;
    Y[0] = pow(X[0], e);

    // create F: X -> Y and stop tape recording
    CppAD::ADFun< AD<double> > F(X, Y);

    // check function value
    ok  &= NearEqual(Value(Value(Y[0])), pow(x0, e), eps99, eps99);

    // forward computation of first partial w.r.t. x[1]
    double d1 = e * pow(x0, (e-1));
    CPPAD_TESTVECTOR(AD<double>) dx(n);
    CPPAD_TESTVECTOR(AD<double>) dy(m);
    dx[0] = 1.;
    dy    = F.Forward(1, dx);
    ok   &= NearEqual(dy[0], d1, eps99, eps99);

    // reverse mode computation of second partials
    // x.r.t. x[1],x[0]  and x[1], x[1]
    double d2 = e * (e-1) * pow(x0, (e-2));
    CPPAD_TESTVECTOR(AD<double>)   w(m);
    CPPAD_TESTVECTOR(AD<double>) ddw(2*n);
    w[0] = 1.;
    ddw  = F.Reverse(2, w);
    ok  &= NearEqual(ddw[0], d1, eps99, eps99);
    ok  &= NearEqual(ddw[1], d2, eps99, eps99);

    return ok;
}

/*
// This test does not pass because azmul is used be reverse mode to deteremine which
// partials are slected and the result is zero instead of nan or infinity.
// There is a wish list item to fix this problem.
bool PowTestSeven(void)
{   bool ok = true;

    using std::cout;
    using CppAD::AD;
    using CppAD::vector;
    //
    vector< double> x(1), y(1), dx(1), dy(1), w(1), dw(2);
    vector< AD<double> > ax(1), ay(1);
    //
    ax[0] = 0.0;
    //
    CppAD::Independent(ax);
    ay[0] = pow(ax[0], 0.5);
    CppAD::ADFun<double> f(ax, ay);
    f.check_for_nan(false);
    //
    x[0]  = 0.0;
    y     = f.Forward(0, x);
    //
    dx[0] = 1.0;
    dy    = f.Forward(1, dx);
    //
    w[0]  = 1.0;
    dw    = f.Reverse(2, w);
    //
    ok &= y[0] == 0.0;
    ok &= ! std::isfinite( dw[0] );
    ok &= ! std::isfinite( dw[1] );
    //
    return ok;
}
*/
// Test x^e where x is negative and e is AD<double> equal to an integer
bool PowTestEight(void)
{   bool ok = true;

    using std::cout;
    using CppAD::AD;
    using CppAD::vector;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();
    //
    vector< double>      x(1), y(2), dx(1), dy(2), w(2), dw(2);
    vector< AD<double> > ax(1), ay(2);
    //
    x[0]  = -2.0;
    ax[0] = x[0];
    //
    CppAD::Independent(ax);
    ay[0] = pow(ax[0],  2.0);
    ay[1] = pow(ax[0], -2.0);
    CppAD::ADFun<double> f(ax, ay);
    f.check_for_nan(true);
    //
    double check;
    y     = f.Forward(0, x);
    check = x[0] * x[0];
    ok   &= NearEqual(y[0], check, eps99, eps99);
    check = 1.0 / (x[0] * x[0]);
    ok   &= NearEqual(y[1], check, eps99, eps99);
    //
    dx[0] = 1.0;
    dy    = f.Forward(1, dx);
    check = 2.0 * x[0];
    ok   &= NearEqual(dy[0], check, eps99, eps99);
    check = -2.0 / ( x[0] * x[0] * x[0] );
    ok   &= NearEqual(dy[1], check, eps99, eps99);
    //
    w[0]   = 1.0;
    w[1]   = 0.0;
    dw     = f.Reverse(2, w);
    check  = 2.0 * x[0];
    ok    &= NearEqual(dw[0], check, eps99, eps99);
    check  = 2.0;
    ok    &= NearEqual(dw[1], check, eps99, eps99);
    //
    w[0]   = 0.0;
    w[1]   = 1.0;
    dw     = f.Reverse(2, w);
    check  = - 2.0 / (x[0] * x[0] * x[0]);
    ok    &= NearEqual(dw[0], check, eps99, eps99);
    check  = 6.0 / (x[0] * x[0] * x[0] * x[0]);
    ok    &= NearEqual(dw[1], check, eps99, eps99);
    //
    return ok;
}

} // END empty namespace

bool Pow(void)
{   bool ok = true;
    ok     &= PowTestOne();
    ok     &= PowTestTwo();
    ok     &= PowTestThree();
    ok     &= PowTestFour();
    ok     &= PowTestFive();
    ok     &= PowTestSix();
    // ok     &= PowTestSeven();
    ok     &= PowTestEight();
    //
    return ok;
}
