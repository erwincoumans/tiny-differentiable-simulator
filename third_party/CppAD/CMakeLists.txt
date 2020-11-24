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
# =============================================================================
# Some constants
# =============================================================================
# Suppress warnging that WIN32 not defined on cygwin
SET(CMAKE_LEGACY_CYGWIN_WIN32 0) # remove when version below >= 2.8.4
#
# Set the minimum required version of cmake for this project.
# see http://www.cmake.org/pipermail/cmake/2013-January/053213.html
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)
#
# https://gitlab.kitware.com/cmake/cmake/issues/17292
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
#
# cppad_version is used by version.sh to get the version number.
SET(cppad_version "20201118")
SET(cppad_url          "http://www.coin-or.org/CppAD" )
SET(cppad_description  "Differentiation of C++ Algorithms" )
#
# Set name of this project and create the variables
# cppad_BINARY_DIR and cppad_SOURCE_DIR.
# project(projectname [CXX] [C] [Java])
PROJECT(cppad)
#
# Add the include sub-directory to the list of C++ preprocessor
# include directories for the entire project.  The SYSTEM flag is not included
# so warnings will be geenreated for this directory.
INCLUDE_DIRECTORIES( ${cppad_SOURCE_DIR}/include )
#
# =============================================================================
# Some system cmake language extensions
# =============================================================================
# CHECK_CXX_SOURCE_COMPILES(source variable)
# Checks whether the code given in source will compile, link and run and
# return zero status. You can set
# CMAKE_REQUIRED_LIBRARIES, CMAKE_REQUIRED_FLAGS and CMAKE_REQUIRED_INCLUDES
# accordingly if additional libraries or compiler flags are required.
INCLUDE(CheckCXXSourceRuns)

# ============================================================================
# Some local cmake language
# ============================================================================
# assert_value_in_set(value element_1 ... element_n)
INCLUDE(cmake/assert_value_in_set.cmake)
#
# dos_path_to_unix(dos_path unix_path)
INCLUDE(cmake/dos_path_to_unix.cmake)
#
# add_to_list(variable_list constant_value)
INCLUDE(cmake/add_to_list.cmake)
#
# command_line_arg(variable default type description)
INCLUDE(cmake/command_line_arg.cmake)
#
# optional_package(package description)
INCLUDE(cmake/optional_package.cmake)
#
# run_source_test(source variable)
INCLUDE(cmake/run_source_test.cmake)
#
# assert(variable)
INCLUDE(cmake/assert.cmake)
#
# print_variable(variable)
INCLUDE(cmake/print_variable.cmake)
#
# set_compile_flags( program_name debug_which source_list)
INCLUDE(cmake/set_compile_flags.cmake)
# =============================================================================
# command line arguments
# =============================================================================
# cmake_install_datadir
command_line_arg(cmake_install_datadir share STRING
    "directory, below prefix, where cmake installs cppad data files"
)
# -----------------------------------------------------------------------------
# cmake_install_docdir
command_line_arg(cmake_install_docdir NOTFOUND STRING
    "directory, below prefix, where cmake installs cppad documentation files"
)
# -----------------------------------------------------------------------------
# cmake_install_includedirs
command_line_arg(cmake_install_includedirs include STRING
    "directories, below prefix, where cmake installs include files"
)
# -----------------------------------------------------------------------------
# cmake_install_libdirs
command_line_arg(cmake_install_libdirs lib STRING
    "directories, below prefix, where cmake installs library files"
)
# -----------------------------------------------------------------------------
# cppad_prefix
command_line_arg(cppad_prefix "${CMAKE_INSTALL_PREFIX}" PATH
    "cppad install prefix"
)
SET(CMAKE_INSTALL_PREFIX "${cppad_prefix}"
    CACHE PATH "value copied from cppad_prefix" FORCE
)
IF( cmake_install_prefix )
    MESSAGE(FATAL_ERROR
        "cmake_install_prefix has been changed to cppad_prefix"
    )
ENDIF( cmake_install_prefix )
# -----------------------------------------------------------------------------
# cppad_postfix
command_line_arg(cppad_postfix NOTFOUND STRING
    "cppad install postfix"
)
IF( cmake_install_postfix )
    MESSAGE(FATAL_ERROR
        "cmake_install_postfix has been changed to cppad_postfix"
    )
ENDIF( cmake_install_postfix )
# -----------------------------------------------------------------------------
# cppad_cxx_flags
command_line_arg(cppad_cxx_flags "" STRING
    "compile flags used with cppad (besides debug, release, and profile flags)"
)
# -----------------------------------------------------------------------------
# cppad_profile_flag
command_line_arg(cppad_profile_flag NOTFOUND STRING
    "compile flag used to compile and link a profile version of a program"
)
# -----------------------------------------------------------------------------
# All the External package prefixes without warnings
SET(system_include TRUE)
#
# adolc_prefix
optional_package(adolc ${system_include} "adolc install prefix")
#
# colpack_prefix
optional_package(colpack ${system_include} "colpack install prefix")
#
# ipopt_prefix
optional_package(ipopt ${system_include} "ipopt install prefix")
#
# sacado_prefix
optional_package(sacado ${system_include} "sacado install prefix")
#
# eigen_prefix
optional_package(eigen ${system_include} "eigen install prefix")
#
# fadbad_prefix
optional_package(fadbad ${system_include} "fadbad install prefix")
#
# cppadcg_prefix
optional_package(cppadcg ${system_include} "cppadcg install prefix")
#
# xpackage_prefix is an example package prefix that is not really used
SET(xpackage_prefix "/usr")
# -----------------------------------------------------------------------------
# cppad_testvector
command_line_arg(cppad_testvector cppad STRING
    "Namespace of vector used for testing, one of: boost, cppad, eigen, std"
)
assert_value_in_set(cppad_testvector boost cppad eigen std)
#
# cppad_max_num_threads
command_line_arg(cppad_max_num_threads 48 STRING
    "maximum number of threads that CppAD can use use"
)
# cppad_sparse_list
IF( cppad_sparse_list )
    MESSAGE(FATAL_ERROR
        "cppad_sparse_list = ${cppad_sparse_list}"
        ": this command line argument has been removed"
    )
ENDIF( cppad_sparse_list )
#
# cppad_tape_id_type
command_line_arg(cppad_tape_id_type "unsigned int" STRING
    "type used to identify different tapes, size must be <= sizeof(size_t)"
)
#
# cppad_tape_addr_type
command_line_arg(cppad_tape_addr_type "unsigned int" STRING
"type used to identify variables on one tape, size must be <= sizeof(size_t)"
)
#
# cppad_debug_which
command_line_arg(cppad_debug_which "debug_all" STRING
    "debug_even, debug_odd, debug_all, or debug_none"
)
assert_value_in_set(cppad_debug_which debug_even debug_odd debug_all debug_none)
#
# cppad_deprecated
IF( cppad_deprecated )
    MESSAGE(FATAL_ERROR "cppad_deprecated has been removed from cmake comamnd" )
ENDIF( cppad_deprecated )
# =============================================================================
# cppad_lib
# Perhaps in the future cppad_lib will depend on cmake header only flag ?
SET( cppad_lib    "cppad_lib" )
LINK_DIRECTORIES( ${cppad_BINARY_DIR}/cppad_lib )
# =============================================================================
# colpack_libs
#
IF( cppad_has_colpack )
    SET( colpack_libs "ColPack" )
ELSE( cppad_has_colpack )
    SET( colpack_libs "" )
ENDIF( cppad_has_colpack )
# =============================================================================
# automated system configuration
# =============================================================================
# CMAKE_CXX_FLAGS
IF( "${debug_which}" STREQUAL "debug_all" )
    print_variable(CMAKE_CXX_FLAGS_DEBUG)
ELSEIF( "${debug_which}" STREQUAL "debug_none" )
    print_variable(CMAKE_CXX_FLAGS_RELEASE)
ELSE( "${debug_which}" )
    print_variable(CMAKE_CXX_FLAGS_DEBUG)
    print_variable(CMAKE_CXX_FLAGS_RELEASE)
ENDIF( "${debug_which}" STREQUAL "debug_all" )
# -----------------------------------------------------------------------------
# cppad_abs_includedir, cppad_abs_libdir, cppad_abs_datadir, cppad_abs_docdir
#
# for dir_types = includedirs, libdirs, datadir, docdir
FOREACH(dir_types includedirs libdirs datadir docdir)
    # set dir_type = dir_types with  "dirs" -> "dir"
    STRING(REGEX REPLACE "dirs" "dir" dir_type ${dir_types})
    #
    # set dir_name to first directory in cmake_install_${dir_types}
    SET(dir_name NOTFOUND)
    FOREACH(dir ${cmake_install_${dir_types}})
        IF( NOT dir_name )
            SET(dir_name ${dir})
        ENDIF( NOT dir_name )
    ENDFOREACH(dir ${cmake_install_${dir_types}})
    #
    # set cppad_abs_${dir_type}
    SET(cppad_abs_${dir_type} "${cppad_prefix}/${dir_name}" )
    #
    # check if we need to add a postfix to it
    IF( cppad_postfix )
        SET(cppad_abs_${dir_type} ${cppad_abs_${dir_type}}/${cppad_postfix} )
    ENDIF( cppad_postfix )
ENDFOREACH(dir_types includedirs libdirs datadir docdir)
# -----------------------------------------------------------------------------
# OPENMP_FOUND, OpenMP_CXX_FLAGS
FIND_PACKAGE(OpenMP)
# -----------------------------------------------------------------------------
# boost_prefix
# Only need components for object libraries, not include libraries.
FIND_PACKAGE(Boost COMPONENTS thread)
SET(cppad_has_boost 0)
IF ( Boost_FOUND )
    SET(cppad_has_boost 1)
    #
    # Extract the Boost prefix from Boost_INCLUDE_DIRS
    #
    # convert to using unix directory separator
    dos_path_to_unix("${Boost_INCLUDE_DIRS}" boost_include_dirs)
    #
    # convert to just one directory
    STRING(REGEX REPLACE
        "([^ ]+).*" "\\1"
        boost_include_dir "${boost_include_dirs}"
    )
    #
    # extract part before last backslash
    STRING(REGEX REPLACE
        "([^ ]*)/[^/ ]*" "\\1"
        boost_prefix "${boost_include_dir}"
    )
    print_variable(boost_prefix)
    #
    # add boost include directories
    FOREACH(dir ${cmake_install_includedirs})
        IF( IS_DIRECTORY ${boost_prefix}/${dir} )
            INCLUDE_DIRECTORIES( ${boost_prefix}/${dir} )
            MESSAGE(STATUS "Found ${boost_prefix}/${dir}")
        ENDIF( IS_DIRECTORY ${boost_prefix}/${dir} )
    ENDFOREACH( dir )
    #
    # add boost link directories
    FOREACH(dir ${cmake_install_libdirs})
        IF( IS_DIRECTORY ${boost_prefix}/${dir} )
            LINK_DIRECTORIES( ${boost_prefix}/${dir} )
            MESSAGE(STATUS "Found ${boost_prefix}/${dir}")
        ENDIF( IS_DIRECTORY ${boost_prefix}/${dir} )
    ENDFOREACH( dir )
ENDIF ( Boost_FOUND )
# -----------------------------------------------------------------------------
# ipopt_LIBRARIES and ipopt_LIBRARY_DIRS
IF( cppad_has_ipopt )
    FIND_PACKAGE(PkgConfig)
    IF( NOT PKG_CONFIG_FOUND )
        MESSAGE(FATAL_ERROR "Using ipopt_prefix but cannot find pkg-config")
    ENDIF( NOT PKG_CONFIG_FOUND )
    #
    # Set the system environment variable PKG_CONFIG_PATH
    FOREACH(dir ${cmake_install_libdirs})
        IF(EXISTS "${ipopt_prefix}/${dir}/pkgconfig/ipopt.pc")
            SET( ENV{PKG_CONFIG_PATH} ${ipopt_prefix}/${dir}/pkgconfig )
        ENDIF(EXISTS "${ipopt_prefix}/${dir}/pkgconfig/ipopt.pc")
    ENDFOREACH(dir)
    #
    # pkg_check_modules(<PREFIX> [REQUIRED] <MODULE> [<MODULE>]*)
    # ipopt_LIBRARIES      ... only the libraries (w/o the '-l')
    # ipopt_LIBRARY_DIRS   ... the paths of the libraries (w/o the '-L')
    pkg_check_modules(ipopt ipopt)
    IF( NOT ipopt_FOUND )
        MESSAGE(FATAL_ERROR
"For all directories dir in cmake_install_libdirs, cannot find the file
    ipopt_prefix/dir/pkgconfig/ipopt.pc
where
    ipopt_prefix = ${ipopt_prefix}
    cmake_install_libdirs = ${cmake_install_libdirs}
"
        )
    ENDIF( NOT ipopt_FOUND )
ENDIF( cppad_has_ipopt )
# =============================================================================
# Currently building tests as normal executables
# =============================================================================
# The CMakeLists.txt file in the specified source directory is processed
# before the current input file continues beyond this command.
# add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
#
# Initialize list of tests as empty
SET(check_depends "")
#
# directories with no check depends entries
ADD_SUBDIRECTORY(include/cppad)
ADD_SUBDIRECTORY(pkgconfig)
ADD_SUBDIRECTORY(cppad_lib)
#
IF( NOT ( "${check_depends}" STREQUAL "" ) )
    MESSAGE(FATAL_ERROR "Error in CMakeLists.txt scripts")
ENDIF( NOT ( "${check_depends}" STREQUAL "" ) )
#
# directories with check depends entries
ADD_SUBDIRECTORY(example)
ADD_SUBDIRECTORY(introduction)
ADD_SUBDIRECTORY(test_more)
ADD_SUBDIRECTORY(speed)
IF( cppad_has_ipopt )
    ADD_SUBDIRECTORY(cppad_ipopt)
ENDIF( cppad_has_ipopt )
#
ADD_CUSTOM_TARGET(check DEPENDS ${check_depends})
MESSAGE(STATUS "make check: avialable")
# ============================================================================
# Copy a file to another location and modify its contents.
# configure_file(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/bin/test_one.sh.in
    ${CMAKE_CURRENT_SOURCE_DIR}/bin/test_one.sh
)
# =============================================================================
# install procedure
# =============================================================================
# install(DIRECTORY dirs... DESTINATION <dir>
#   [FILE_PERMISSIONS permissions...]
#   [DIRECTORY_PERMISSIONS permissions...]
#   [USE_SOURCE_PERMISSIONS] [OPTIONAL]
#   [CONFIGURATIONS [Debug|Release|...]]
#   [COMPONENT <component>] [FILES_MATCHING]
#   [[PATTERN <pattern> | REGEX <regex>]
#   [EXCLUDE] [PERMISSIONS permissions...]] [...]
# )
# Note a trailing / in the source directory name drops the source directory
# name during the copy.
#
# During install copy all the cppad include files to
# ${cppad_abs_includedir}/cppad
INSTALL(
    DIRECTORY "${CMAKE_SOURCE_DIR}/include/cppad/"
    DESTINATION ${cppad_abs_includedir}/cppad
    FILES_MATCHING PATTERN "*.hpp" PATTERN "omh" EXCLUDE
)
#
# During install copy doc direcrory to cppad_abs_docdir/cppad
IF ( cmake_install_docdir )
    INSTALL( DIRECTORY "doc/" DESTINATION ${cppad_abs_docdir}/cppad )
ENDIF ( cmake_install_docdir )

# =============================================================================
# uninstall procedure
# =============================================================================
ADD_CUSTOM_TARGET(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cppad_uninstall.cmake)
