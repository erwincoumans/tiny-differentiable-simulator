# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Configure the CppAD include file directory
# -----------------------------------------------------------------------------
# check_match
MACRO(check_match match_variable match_constant output_variable)
    STRING(COMPARE EQUAL ${${match_variable}} ${match_constant} match_flag )
    IF( match_flag )
        SET(${output_variable} 1)
    ELSE( match_flag )
        SET(${output_variable} 0)
    ENDIF( match_flag )
    print_variable(${output_variable})
ENDMACRO(check_match)
# -----------------------------------------------------------------------------
# compiler_has_conversion_warn
SET( clang_or_gnu 0 )
IF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
    SET(clang_or_gnu 1)
ENDIF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang" )
IF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" )
    SET(clang_or_gnu 1)
ENDIF( "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" )
IF( clang_or_gnu )
    SET(backup "${cppad_cxx_flags}")
    SET(cppad_cxx_flags "${backup} -Wfloat-conversion -Wconversion -Werror")
    #
    SET(source "int main(void) { return 0; }")
    run_source_test("${source}" compiler_has_conversion_warn )
    #
    SET(cppad_cxx_flags "${backup}")
ELSE( clang_or_gnu )
    SET( compiler_has_conversion_warn 0 )
ENDIF( clang_or_gnu )
# -----------------------------------------------------------------------------
# cppad_boostvector, cppad_cppadvector, cppad_eigenvector, cppad_stdvector
#
check_match(cppad_testvector boost cppad_boostvector)
check_match(cppad_testvector cppad cppad_cppadvector)
check_match(cppad_testvector eigen cppad_eigenvector)
check_match(cppad_testvector std   cppad_stdvector)
IF( NOT cppad_boostvector )
IF( NOT cppad_cppadvector )
IF( NOT cppad_eigenvector )
IF( NOT cppad_stdvector )
MESSAGE(FATAL_ERROR
"cppad_testvector not one of following: boost, cppad, eigen, std."
"This should have been found earlier, please report this as a bug."
)
ENDIF( NOT cppad_stdvector )
ENDIF( NOT cppad_eigenvector )
ENDIF( NOT cppad_cppadvector )
ENDIF( NOT cppad_boostvector )

IF( cppad_boostvector )
    # FIND_PACKAGE(Boost) done by ../CMakeLists.txt
    IF( NOT Boost_FOUND )
        MESSAGE(FATAL_ERROR
"cppad_testvector == boost but cannot find boost include files"
        )
    ENDIF( NOT Boost_FOUND )
ENDIF( cppad_boostvector )
#
IF( cppad_eigenvector )
    IF( NOT eigen_prefix )
        MESSAGE(FATAL_ERROR
"cppad_testvector == eigen but eigen_prefix is not specified"
        )
    ENDIF( NOT eigen_prefix )
ENDIF( cppad_eigenvector )
# =============================================================================
# Begin C++11 features
#
SET(cppad_cplusplus_201100_ok 1)
# -----------------------------------------------------------------------------
# cppad_has_signed_long_long
#
SET(source "
int main(void)
{   signed long long value = 0;
    return int(value);
}"
)
run_source_test("${source}" cppad_has_signed_long_long )
IF( NOT cppad_has_signed_long_long )
    SET(cppad_cplusplus_201100_ok 0)
ENDIF( NOT cppad_has_signed_long_long )
# -----------------------------------------------------------------------------
# cppad_has_rvalue
#
SET(source "
int main(void)
{   int&& value = 1 + 2;
    if( value != 3 )
        return 1;
    return 0;
}"
)
run_source_test("${source}" cppad_has_rvalue)
IF( NOT cppad_has_rvalue )
    SET(cppad_cplusplus_201100_ok 0)
ENDIF( NOT cppad_has_rvalue )
# -----------------------------------------------------------------------------
# cppad_has_nullptr
#
SET(source "
int main(void)
{   char *c = nullptr;
    return 0;
}"
)
run_source_test("${source}" cppad_has_nullptr)
IF( NOT cppad_has_nullptr )
    SET(cppad_cplusplus_201100_ok 0)
ENDIF( NOT cppad_has_nullptr )
# -----------------------------------------------------------------------------
# cppad_has_long_long
#
SET(source "
template <class T> bool is_pod(void)  { return false; }
template <> inline bool is_pod<signed long long>(void) { return true;  }
template <> inline bool is_pod<unsigned long long>(void) { return true;  }
int main(void)
{   return 0; }
"
)
run_source_test("${source}" cppad_has_long_long)
IF( NOT cppad_has_long_long )
    SET(cppad_cplusplus_201100_ok 0)
ENDIF( NOT cppad_has_long_long )
# -----------------------------------------------------------------------------
# cppad_compiler_has_erf
# cppad_compiler_has_asinh
# cppad_compiler_has_acosh
# cppad_compiler_has_atanh
# cppad_compiler_has_expm1
#
SET(source_template "
# include <cmath>
int main(void)
{   std::c11_function(0.0);
    return 0;
}
"
)
FOREACH(c11_function erf asinh acosh atanh expm1 log1p)
    STRING(REPLACE "c11_function" ${c11_function} source "${source_template}" )
    # really only checking if program compiles
    run_source_test("${source}" cppad_compiler_has_${c11_function} )
    IF( NOT cppad_compiler_has_${c11_function} )
        SET(cppad_cplusplus_201100_ok 0)
    ENDIF( NOT cppad_compiler_has_${c11_function} )
ENDFOREACH(c11_function)
# -----------------------------------------------------------------------------
# cppad_has_steady_clock
#
SET(source "
# include <chrono>
# include <thread>
# include <iostream>
int main(void)
{   std::chrono::time_point<std::chrono::steady_clock> start, end;
    start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for( std::chrono::milliseconds(1) );
    end   = std::chrono::steady_clock::now();
    std::chrono::duration<double> difference = end - start;
    double elapsed_seconds = difference.count();
    if( elapsed_seconds >= 0.9e-3 ) // 1e-3 = one millisecond
        return 0;
    return 1;
}
"
)
run_source_test("${source}" cppad_has_steady_clock)
IF( NOT cppad_has_steady_clock )
    SET(cppad_cplusplus_201100_ok 0)
ENDIF( NOT cppad_has_steady_clock )
# =============================================================================
# End C++11 features
#
print_variable(cppad_cplusplus_201100_ok)
# -----------------------------------------------------------------------------
# cppad_has_gettimeofday
#
SET(source "
# include<sys/time.h>
int main(void)
{   struct timeval time;
    gettimeofday(&time, 0);
    return 0;
}"
)
run_source_test("${source}" cppad_has_gettimeofday)
# -----------------------------------------------------------------------------
# cppad_tape_addr_type, cppad_tape_id_type
#
FOREACH(cmake_var cppad_tape_id_type cppad_tape_addr_type )
    SET(source "
# include <limits>
int main(void)
{   bool is_unsigned = ! std::numeric_limits<${${cmake_var}}>::is_signed;
    return int(! is_unsigned);
}
"
    )
    run_source_test("${source}" ${cmake_var}_is_unsigned)
    IF( ${cmake_var}_is_unsigned STREQUAL 0  )
        MESSAGE(STATUS
"Warning: using a signed ${cmake_var} is for CppAD developers only !"
    )
    ENDIF( ${cmake_var}_is_unsigned STREQUAL 0  )
ENDFOREACH( cmake_var )
# -----------------------------------------------------------------------------
# check that cppad_max_num_threads is >= 4
#
SET(CMAKE_REQUIRED_DERINITIONS "")
SET(CMAKE_REQUIRED_INCLUDES    "")
SET(CMAKE_REQUIRED_LIBRARIES   "")
SET(CMAKE_REQUIRED_FLAGS       "")
SET(source "
int main(void)
{   const char* number = \"${cppad_max_num_threads}\";
    int value = 0;
    while( *number == ' ' )
        number++;
    while( '0' <= *number && *number <= '9' )
    {   value = 10 * value + (int)(*number - '0');
        number++;
    }
    while( *number == ' ' )
        number++;
    if( *number != char(0) )
        return 1;
    if( value < 4 )
        return 1;
    return 0;
}
" )
# Using CHECK_CXX_SOURCE_RUNS directly (instead of run_source_test).
IF( DEFINED cppad_max_num_threads_is_integer_ge_4 )
    MESSAGE( ERROR
        "cppad_max_num_threads_is_integer_ge_4 is defined before expected"
    )
ENDIF( DEFINED cppad_max_num_threads_is_integer_ge_4 )
CHECK_CXX_SOURCE_RUNS("${source}" cppad_max_num_threads_is_integer_ge_4 )
IF( NOT cppad_max_num_threads_is_integer_ge_4 )
    MESSAGE(FATAL_ERROR
    "cppad_max_num_threads is not an integer greater than or equal 4"
    )
ENDIF( NOT cppad_max_num_threads_is_integer_ge_4 )
# -----------------------------------------------------------------------------
# cppad_has_mkstemp
#
SET(source "
# include <stdlib.h>
# include <unistd.h>
int main(void)
{
    char pattern[] = \"/tmp/fileXXXXXX\";
    int fd = mkstemp(pattern);
    return 0;
}
" )
run_source_test("${source}" cppad_has_mkstemp )
# -----------------------------------------------------------------------------
# cppad_has_tmpname_s
#
SET(source "
# include <stdio.h>
int main(void)
{   char filename[L_tmpnam_s ];
    if( tmpnam_s(filename, L_tmpnam_s ) != 0 )
        return 1;
    return 0;
}
" )
run_source_test("${source}" cppad_has_tmpnam_s )
# -----------------------------------------------------------------------------
# configure.hpp
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/configure.hpp.in
    ${CMAKE_CURRENT_SOURCE_DIR}/configure.hpp
)
# -----------------------------------------------------------------------------
# configure files in local subdirectory
ADD_SUBDIRECTORY(local)
