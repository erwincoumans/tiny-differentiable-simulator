/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin microsoft_timer$$
$spell
    Microsoft
    cpp
    src
$$

$section Microsoft Version of Elapsed Number of Seconds$$


$head Syntax$$
$icode%s% = microsoft_timer()%$$

$head Purpose$$
This routine is accurate to within .02 seconds
(see $cref elapsed_seconds$$ which uses this routine when
the preprocessor symbol $code _MSC_VER$$ is defined).
It does not necessary work for time intervals that are greater than a day.
It uses $code ::GetSystemTime$$ for timing.

$head s$$
is a $code double$$ equal to the
number of seconds since the first call to $code microsoft_timer$$.

$head Linking$$
The source code for this routine is located in
$code speed/src/microsoft_timer.cpp$$.
The preprocessor symbol $code _MSC_VER$$ must
be defined, or this routine is not compiled.

$end
-----------------------------------------------------------------------
*/
# ifdef _MSC_VER
# include <windows.h>
# include <cassert>

// Note that the doxygen for this routine does not get generated because
// _MSC_VER is not defined during generation. In general, it is a problem
// that not all preprocessor options get documented.
/*!
\{
\file microsoft_timer.cpp
\brief Microsoft version of elapsed_seconds.
*/

/*!
Microsoft version of elapsed number of seconds since frist call.

\copydetails elapsed_seconds
*/
double microsoft_timer(void)
{   static bool       first_  = true;
    static SYSTEMTIME st_;
    SYSTEMTIME st;

    if( first_ )
    {   ::GetSystemTime(&st_);
        first_ = false;
        return 0.;
    }
    ::GetSystemTime(&st);

    double hour   = double(st.wHour)         - double(st_.wHour);
    double minute = double(st.wMinute)       - double(st_.wMinute);
    double second = double(st.wSecond)       - double(st_.wSecond);
    double milli  = double(st.wMilliseconds) - double(st_.wMilliseconds);

    double diff   = 1e-3*milli + second + 60.*minute + 3600.*hour;
    if( diff < 0. )
        diff += 3600.*24.;
    assert( 0 <= diff && diff < 3600.*24. );

    return diff;
}

# endif
