/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// system include files used for I/O
# include <iostream>

// C style asserts
# include <cassert>

// for thread_alloc
# include <cppad/utility/thread_alloc.hpp>

// test runner
# include <cppad/utility/test_boolofvoid.hpp>

// external compiled tests
extern bool CheckNumericType(void);
extern bool CheckSimpleVector(void);
extern bool CppAD_vector(void);
extern bool ErrorHandler(void);
extern bool index_sort(void);
extern bool LuFactor(void);
extern bool LuInvert(void);
extern bool LuSolve(void);
extern bool nan(void);
extern bool Near_Equal(void);
extern bool OdeErrControl(void);
extern bool OdeErrMaxabs(void);
extern bool OdeGearControl(void);
extern bool OdeGear(void);
extern bool RombergMul(void);
extern bool RombergOne(void);
extern bool runge_45_1(void);
extern bool set_union(void);
extern bool SimpleVector(void);
extern bool sparse_rc(void);
extern bool sparse_rcv(void);
extern bool thread_alloc(void);
extern bool to_string(void);
extern bool vectorBool(void);

// main program that runs all the tests
int main(void)
{   std::string group = "example/utility";
    size_t      width = 20;
    CppAD::test_boolofvoid Run(group, width);

    // This line is used by test_one.sh

    // external compiled tests
    Run( CheckNumericType,       "CheckNumericType" );
    Run( CheckSimpleVector,      "CheckSimpleVector" );
    Run( CppAD_vector,           "CppAD_vector" );
    Run( ErrorHandler,           "ErrorHandler" );
    Run( index_sort,             "index_sort" );
    Run( LuFactor,               "LuFactor" );
    Run( LuInvert,               "LuInvert" );
    Run( LuSolve,                "LuSolve" );
    Run( nan,                    "nan" );
    Run( Near_Equal,             "Near_Equal" );
    Run( OdeErrControl,          "OdeErrControl" );
    Run( OdeErrMaxabs,           "OdeErrMaxabs" );
    Run( OdeGearControl,         "OdeGearControl" );
    Run( OdeGear,                "OdeGear" );
    Run( RombergMul,             "RombergMul" );
    Run( RombergOne,             "RombergOne" );
    Run( runge_45_1,             "runge_45_1" );
    Run( set_union,              "set_union" );
    Run( SimpleVector,           "SimpleVector" );
    Run( thread_alloc,           "thread_alloc" );
    Run( sparse_rc,              "sparse_rc" );
    Run( sparse_rcv,             "sparse_rcv" );
    Run( to_string,              "to_string" );
    Run( vectorBool,             "vectorBool" );
    //
    // check for memory leak
    bool memory_ok = CppAD::thread_alloc::free_all();
    // print summary at end
    bool ok = Run.summary(memory_ok);
    //
    return static_cast<int>( ! ok );
}
