# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# set_compile_flags( program_name debug_which source_list)
#
# program_name: (in)
# Is the name of the program that we are building. This is only used to
# report which files have debug and which have release properties.
#
# debug_which: (in)
# Is one of the following cases:
#
# Case debug_even:
# The files with an even (odd) index in source_list have debug (release) flags.
# In addition the compiler flag -DCPPAD_DEBUG_AND_RELEASE is added.
#
# Case debug_odd:
# The files with an odd (even) index in source_list have debug (release) flags.
# In addition the compiler flag -DCPPAD_DEBUG_AND_RELEASE is added.
#
# Case debug_all:
# All the files have debug flags.
#
# Case debug_none:
# All the the files have release flags.
#
# source_list: (in)
# is a list of source files that get set to have debug or release
# compile flags. The cppad_cxx_flags compile flags always get included.
#
FUNCTION(set_compile_flags program_name debug_which source_list)
    # debug compile flags
    SET(debug_flags "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_DEBUG}")
    # relese compile flags
    SET(release_flags "${cppad_cxx_flags} ${CMAKE_CXX_FLAGS_RELEASE}")
    #
    # set alternate, report random number result,
    # set compile flags property when not alternating.
    IF( "${debug_which}" STREQUAL "debug_even" )
        SET(alternate TRUE)
        SET(count_mod_2 0)
    ELSEIF( "${debug_which}" STREQUAL "debug_odd" )
        SET(alternate TRUE)
        SET(count_mod_2 1)
    ELSEIF( "${debug_which}" STREQUAL "debug_all" )
        SET(alternate FALSE)
        SET_SOURCE_FILES_PROPERTIES(
            ${source_list} PROPERTIES COMPILE_FLAGS "${debug_flags}"
        )
    ELSEIF( "${debug_which}" STREQUAL "debug_none" )
        SET(alternate FALSE)
        SET_SOURCE_FILES_PROPERTIES(
            ${source_list} PROPERTIES COMPILE_FLAGS "${release_flags}"
        )
    ELSE( "${debug_which}" )
        MESSAGE(FATAL_ERROR "cmake error: debug_which = ${debug_which}")
    ENDIF( "${debug_which}" STREQUAL "debug_even" )
    #
    IF( alternate )
        SET(debug_list   "")
        SET(release_list "")
        FOREACH(source ${source_list})
            MATH(EXPR count_mod_2 "(${count_mod_2} + 1) % 2")
            IF( count_mod_2 )
                add_to_list(debug_list ${source})
            ELSE( count_mod_2 )
                add_to_list(release_list ${source})
            ENDIF( count_mod_2 )
        ENDFOREACH(source ${source_list})
        SET_SOURCE_FILES_PROPERTIES(
            ${debug_list} PROPERTIES COMPILE_FLAGS
            "${debug_flags} -DCPPAD_DEBUG_AND_RELEASE"
        )
        SET_SOURCE_FILES_PROPERTIES(
            ${release_list} PROPERTIES COMPILE_FLAGS
            "${release_flags} -DCPPAD_DEBUG_AND_RELEASE"
        )
    ENDIF( alternate )
ENDFUNCTION(set_compile_flags program_name debug_which source_list)
