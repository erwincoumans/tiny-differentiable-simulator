# ifndef CPPAD_CORE_ATANH_HPP
# define CPPAD_CORE_ATANH_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
-------------------------------------------------------------------------------
$begin atanh$$
$spell
    atanh
    const
    Vec
    std
    cmath
    CppAD
    tanh
$$
$section The Inverse Hyperbolic Tangent Function: atanh$$

$head Syntax$$
$icode%y% = atanh(%x%)%$$

$head Description$$
The inverse hyperbolic tangent function is defined by
$icode%x% == tanh(%y%)%$$.

$head x, y$$
See the $cref/possible types/unary_standard_math/Possible Types/$$
for a unary standard math function.

$head CPPAD_USE_CPLUSPLUS_2011$$

$subhead true$$
If this preprocessor symbol is true ($code 1$$),
and $icode x$$ is an AD type,
this is an $cref/atomic operation/glossary/Operation/Atomic/$$.

$subhead false$$
If this preprocessor symbol is false ($code 0$$),
CppAD uses the representation
$latex \[
\R{atanh} (x) = \frac{1}{2} \log \left( \frac{1 + x}{1 - x} \right)
\] $$
to compute this function.

$head Example$$
$children%
    example/general/atanh.cpp
%$$
The file
$cref atanh.cpp$$
contains an example and test of this function.

$end
-------------------------------------------------------------------------------
*/
# include <cppad/configure.hpp>
# if ! CPPAD_USE_CPLUSPLUS_2011

// BEGIN CppAD namespace
namespace CppAD {

template <class Type>
Type atanh_template(const Type &x)
{   return CppAD::log( (Type(1) + x) / (Type(1) - x) ) / Type(2);
}

inline float atanh(const float &x)
{   return atanh_template(x); }

inline double atanh(const double &x)
{   return atanh_template(x); }

template <class Base>
AD<Base> atanh(const AD<Base> &x)
{   return atanh_template(x); }

template <class Base>
AD<Base> atanh(const VecAD_reference<Base> &x)
{   return atanh_template( x.ADBase() ); }


} // END CppAD namespace

# endif // CPPAD_USE_CPLUSPLUS_2011
# endif // CPPAD_ATANH_INCLUDED
