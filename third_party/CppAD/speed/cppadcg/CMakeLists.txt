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
# Build the speed/cppadcg directory tests
# Inherit build type from ../CMakeList.txt

# assert cppadcg_prefix is defined
assert ( cppadcg_prefix )

# Adds flags to the compiler command line for sources in the current directory
# and below. This command can be used to add any flags, but it was originally
# intended to add preprocessor definitions.
ADD_DEFINITIONS("-DCPPAD_CPPADCG_SPEED")

# Local include directories to search (not in package_prefix/includdir)
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/../src )

SET(source_list ../main.cpp
    det_lu.cpp
    det_minor.cpp
    det_minor_cg.cpp
    mat_mul.cpp
    ode.cpp
    poly.cpp
    sparse_hessian.cpp
    sparse_jacobian.cpp
    sparse_jacobian_cg.cpp
)
set_compile_flags( speed_cppadcg "${cppad_debug_which}" "${source_list}" )
#
# Adding C source code here because flags above are for C++
# Binary directory version of these files gets replaced by running *_cg.cpp
IF( "${cppad_debug_which}" STREQUAL "debug_all" )
    SET(flags ${CMAKE_C_FLAGS_DEBUG} )
ELSEIF( "${cppad_debug_which}" STREQUAL "debug_all" )
    SET(flags ${CMAKE_C_FLAGS_RELEASE} )
ENDIF( "${cppad_debug_which}" STREQUAL "debug_all" )
FOREACH(c_source det_minor_grad.c sparse_jacobian.c )
    CONFIGURE_FILE(
            ${CMAKE_CURRENT_SOURCE_DIR}/${c_source}
            ${CMAKE_CURRENT_BINARY_DIR}/${c_source}
            COPYONLY
    )
    SET(file_name ${CMAKE_CURRENT_BINARY_DIR}/${c_source})
    add_to_list(source_list ${file_name})
    SET_SOURCE_FILES_PROPERTIES(
        ${file_name} PROPERTIES COMPILE_FLAGS "${flags}"
    )
ENDFOREACH(c_source det_minor_grad.c link_sparse_jacobian.c )
#
ADD_EXECUTABLE(speed_cppadcg EXCLUDE_FROM_ALL ${source_list} )

# cppadcg is an include file only library
TARGET_LINK_LIBRARIES(speed_cppadcg
    speed_src
    ${cppad_lib}
    ${colpack_libs}
)


# Add the check_speed_cppadcg target
SET(random_seed 54321)
ADD_CUSTOM_TARGET(check_speed_cppadcg
    speed_cppadcg correct ${random_seed} onetape
    DEPENDS speed_cppadcg speed_src
)
MESSAGE(STATUS "make check_speed_cppadcg: available")

# Change check depends in parent environment
add_to_list(check_speed_depends check_speed_cppadcg)
SET(check_speed_depends "${check_speed_depends}" PARENT_SCOPE)
