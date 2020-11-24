# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Construct cppad.pc from cppad.pc.in; see
# http://people.freedesktop.org/~dbn/pkg-config-guide.html
# -----------------------------------------------------------------------------
#
# Colpack does not have a pkgconfig file. Search for the missing information.
# Note that colpack is linked through cppad_lib, so we only need
# colpack_libdir
SET(colpack_libdir     NOTFOUND)
IF( cppad_has_colpack )
    FOREACH(dir ${cmake_install_libdirs})
        FILE(GLOB file_list "${colpack_prefix}/${dir}/libColPack.*" )
        IF( file_list )
            SET(colpack_libdir "${colpack_prefix}/${dir}")
        ENDIF( file_list )
    ENDFOREACH(dir ${cmake_install_libdirs})
    IF( NOT colpack_libdir )
        MESSAGE(FATAL_ERROR "Cannit find libColPack.* below ${colpack_prefix}")
    ENDIF( NOT colpack_libdir )
ENDIF( cppad_has_colpack )
# -----------------------------------------------------------------------------
# cppad_pkgconfig_requires
# cppad_pkgconfig_requires_uninstalled
#
SET(cppad_pkgconfig_requires             "")
SET(cppad_pkgconfig_requires_uninstalled "")
IF( cppad_has_ipopt )
    SET(cppad_pkgconfig_requires             "ipopt")
    SET(cppad_pkgconfig_requires_uninstalled "ipopt")
ENDIF( cppad_has_ipopt )
# -----------------------------------------------------------------------------
# cppad_pkgconfig_cflags
# cppad_pkgconfig_cflags_uninstalled
#
SET(cppad_pkgconfig_cflags             "-I${cppad_abs_includedir}" )
SET(cppad_pkgconfig_cflags_uninstalled "-I${cppad_SOURCE_DIR}"     )
# ----------------------------------------------------------------------------
# cppad_pkgconfig_libs
# cppad_pkgconfig_libs_uninstalled
#
# initilaize as empty
SET(libs        "")
SET(uninstalled "")
#
# cppad_lib
IF( cppad_lib )
    SET(libs          "${libs} -lcppad_lib")
    SET(uninstalled   "${uninstalled} -L${cppad_BINARY_DIR}/cppad_lib")
ENDIF( cppad_lib )
#
# cppad_ipopt
IF( cppad_has_ipopt )
    SET(libs           "${libs} -lcppad_ipopt")
    SET(uninstalled    "${uninstalled} -L${cppad_BINARY_DIR}/cppad_ipopt/src")
ENDIF( cppad_has_ipopt)
#
# ColPack
IF( cppad_has_colpack )
    SET(libs            "${libs} -lColPack")
    SET(uninstalled     "${uninstalled} -L${colpack_libdir}")
ENDIF( cppad_has_colpack )
#
IF( "${libs}" STREQUAL "" )
    SET(cppad_pkgconfig_libs             "")
    SET(cppad_pkgconfig_libs_uninstalled "")
ELSE( "${libs}" STREQUAL "" )
    SET(cppad_pkgconfig_libs             "-L${cppad_abs_libdir} ${libs}")
    SET(cppad_pkgconfig_libs_uninstalled "${uninstalled} ${libs}")
ENDIF( "${libs}" STREQUAL "" )
#
# ----------------------------------------------------------------------------
#
# Copy a file to another location and modify its contents.
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/cppad.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc
)
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/cppad-uninstalled.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/cppad-uninstalled.pc
)
# During install copy cppad.pc to datadir and libdir
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc
    DESTINATION ${cppad_abs_datadir}/pkgconfig
)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc
    DESTINATION ${cppad_abs_libdir}/pkgconfig
)
