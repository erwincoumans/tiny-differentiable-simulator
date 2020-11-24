# ifndef CPPAD_LOCAL_COMP_OP_HPP
# define CPPAD_LOCAL_COMP_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file comp_op.hpp
Zero order forward mode check how many comparisons changed.
*/

// -------------------------------- <= -----------------------------------
/*!
Zero order forward mode comparison check that left <= right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
parameter[ arg[0] ] is the left operand and
parameter[ arg[1] ] is the right operand.

\param parameter
vector of parameter values.
*/
template <class Base>
void forward_lepp_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LeppOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LeppOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base x = parameter[ arg[0] ];
    Base y = parameter[ arg[1] ];

    count += size_t( GreaterThanZero(x - y) );
}
/*!
Zero order forward mode comparison check that left <= right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
parameter[ arg[0] ] is the left operand and
taylor[ size_t(arg[1]) * cap_order + 0 ] is the zero order Taylor coefficient
for the right operand.

\param parameter
vector of parameter values.

\param cap_order
number of Taylor coefficients allocated for each variable

\param taylor
vector of taylor coefficients.
*/
template <class Base>
void forward_lepv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LepvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LepvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base  x = parameter[ arg[0] ];
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += GreaterThanZero(x - y[0]);
}
/*!
Zero order forward mode comparison check that left <= right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
taylor[ size_t(arg[0]) * cap_order + 0 ] is the zero order Taylor coefficient
for the left operand and  parameter[ arg[1] ] is the right operand

\param parameter
vector of parameter values.

\param cap_order
number of Taylor coefficients allocated for each variable

\param taylor
vector of taylor coefficients.
*/
template <class Base>
void forward_levp_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LevpOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LevpOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base  y = parameter[ arg[1] ];

    count += GreaterThanZero(x[0] - y);
}
/*!
Zero order forward mode comparison check that left <= right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
taylor[ size_t(arg[0]) * cap_order + 0 ] is the zero order Taylor coefficient
for the left operand and
taylor[ size_t(arg[1]) * cap_order + 0 ] is the zero order Taylor coefficient
for the right operand.

\param parameter
vector of parameter values.

\param cap_order
number of Taylor coefficients allocated for each variable

\param taylor
vector of taylor coefficients.
*/
template <class Base>
void forward_levv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LevvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LevvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += GreaterThanZero(x[0] - y[0]);
}
// ------------------------------- < -------------------------------------
/*!
Zero order forward mode comparison check that left < right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
parameter[ arg[0] ] is the left operand and
parameter[ arg[1] ] is the right operand.

\param parameter
vector of parameter values.
*/
template <class Base>
void forward_ltpp_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LtppOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LtppOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base x = parameter[ arg[0] ];
    Base y = parameter[ arg[1] ];

    count += GreaterThanOrZero(x - y);
}
/*!
Zero order forward mode comparison check that left < right

\copydetails CppAD::local::forward_lepv_op_0
*/
template <class Base>
void forward_ltpv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LtpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LtpvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base  x = parameter[ arg[0] ];
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += GreaterThanOrZero(x - y[0]);
}
/*!
Zero order forward mode comparison check that left < right

\copydetails CppAD::local::forward_levp_op_0
*/
template <class Base>
void forward_ltvp_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LtvpOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LtvpOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base  y = parameter[ arg[1] ];

    count += GreaterThanOrZero(x[0] - y);
}
/*!
Zero order forward mode comparison check that left < right

\copydetails CppAD::local::forward_levv_op_0
*/
template <class Base>
void forward_ltvv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(LtvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(LtvvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += GreaterThanOrZero(x[0] - y[0]);
}
// ------------------------------ == -------------------------------------
/*!
Zero order forward mode comparison check that left == right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
parameter[ arg[0] ] is the left operand and
parameter[ arg[1] ] is the right operand.

\param parameter
vector of parameter values.
*/
template <class Base>
void forward_eqpp_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(EqppOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(EqppOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base x = parameter[ arg[0] ];
    Base y = parameter[ arg[1] ];

    count += size_t(x != y);
}
/*!
Zero order forward mode comparison check that left == right

\copydetails CppAD::local::forward_lepv_op_0
*/
template <class Base>
void forward_eqpv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(EqpvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(EqpvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base  x = parameter[ arg[0] ];
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += size_t(x != y[0]);
}
/*!
Zero order forward mode comparison check that left == right

\copydetails CppAD::local::forward_levv_op_0
*/
template <class Base>
void forward_eqvv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(EqvvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(EqvvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += size_t(x[0] != y[0]);
}
// -------------------------------- != -----------------------------------
/*!
Zero order forward mode comparison check that left != right

\param count
It the condition is not true, ths counter is incremented by one.

\param arg
parameter[ arg[0] ] is the left operand and
parameter[ arg[1] ] is the right operand.

\param parameter
vector of parameter values.
*/
template <class Base>
void forward_nepp_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(NeppOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(NeppOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base x = parameter[ arg[0] ];
    Base y = parameter[ arg[1] ];

    count += size_t(x == y);
}
/*!
Zero order forward mode comparison check that left != right

\copydetails CppAD::local::forward_lepv_op_0
*/
template <class Base>
void forward_nepv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(NepvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(NepvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base  x = parameter[ arg[0] ];
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += size_t(x == y[0]);
}
/*!
Zero order forward mode comparison check that left != right

\copydetails CppAD::local::forward_levv_op_0
*/
template <class Base>
void forward_nevv_op_0(
    size_t&       count       ,
    const addr_t* arg         ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(NevvOp) == 2 );
    CPPAD_ASSERT_UNKNOWN( NumRes(NevvOp) == 0 );

    // Taylor coefficients corresponding to arguments and result
    Base* x = taylor + size_t(arg[0]) * cap_order;
    Base* y = taylor + size_t(arg[1]) * cap_order;

    count += size_t(x[0] == y[0]);
}


} } // END_CPPAD_LOCAL_NAMESPACE
# endif
