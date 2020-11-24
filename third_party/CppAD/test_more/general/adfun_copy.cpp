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
Test that ADFun copy constructor generates an error message.
*/

# include <cppad/cppad.hpp>
# include <string>

namespace {

    // error handler to catch the error
    void myhandler(
        bool known       ,
        int  line        ,
        const char *file ,
        const char *exp  ,
        const char *msg  )
    {   // error handler must not return, so throw an exception
        throw std::string("myhandler");
    }

}

bool adfun_copy(void)
{
    // error handler for this routine
    CppAD::ErrorHandler info(myhandler);
    // an ADFun object
    CppAD::ADFun<double> f;
    // value of ok if no error occurs
    bool ok = false;
    try {
        // This operation uses the ADFun copy constructor which is defined,
        // but should not be used and should generate an error
        CppAD::ADFun<double> g(f);
    }
    catch ( std::string msg )
    {   // check for expected return
        ok = (msg == "myhandler");
    }
    return ok;
}
