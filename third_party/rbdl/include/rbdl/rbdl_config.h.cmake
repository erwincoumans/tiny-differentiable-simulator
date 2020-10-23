/*
* RBDL - Rigid Body Dynamics Library
* Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
*
* Licensed under the zlib license. See LICENSE for more details.
*/

#ifndef RBDL_CONFIG_H
#define RBDL_CONFIG_H

#define RBDL_API_VERSION (@RBDL_VERSION_MAJOR@ << 16) + (@RBDL_VERSION_MINOR@ << 8) + @RBDL_VERSION_PATCH@

#cmakedefine RBDL_USE_SIMPLE_MATH
#cmakedefine RBDL_ENABLE_LOGGING
#cmakedefine RBDL_BUILD_REVISION "@RBDL_BUILD_REVISION@"
#cmakedefine RBDL_BUILD_TYPE "@RBDL_BUILD_TYPE@"
#cmakedefine RBDL_BUILD_BRANCH "@RBDL_BUILD_BRANCH@"
#cmakedefine RBDL_BUILD_ADDON_LUAMODEL
#cmakedefine RBDL_BUILD_ADDON_URDFREADER
#cmakedefine RBDL_BUILD_STATIC
#cmakedefine RBDL_USE_ROS_URDF_LIBRARY
#cmakedefine RBDL_BUILD_ADDON_MUSCLE_FITTING

/* compatibility defines */
#ifdef _WIN32
#define __func__ __FUNCTION__
#define M_PI 3.1415926535897932384
#endif

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define RBDL_DLLIMPORT __declspec(dllimport)
#  define RBDL_DLLEXPORT __declspec(dllexport)
#  define RBDL_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define RBDL_DLLIMPORT __attribute__ ((visibility("default")))
#   define RBDL_DLLEXPORT __attribute__ ((visibility("default")))
#   define RBDL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define RBDL_DLLIMPORT
#   define RBDL_DLLEXPORT
#   define RBDL_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef RBDL_BUILD_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RBDL_DLLAPI
#  define RBDL_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef rbdl_EXPORTS
#   define RBDL_DLLAPI RBDL_DLLEXPORT
#  else
#   define RBDL_DLLAPI RBDL_DLLIMPORT
#  endif // RBDL_EXPORTS
#  define RBDL_LOCAL RBDL_DLLLOCAL
# endif // RBDL_BUILD_STATIC

#endif
