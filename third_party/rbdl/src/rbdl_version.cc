/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <rbdl/rbdl_config.h>

#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>

RBDL_DLLAPI int rbdl_get_api_version() {
  static int compile_version = RBDL_API_VERSION;
  return compile_version;
}

RBDL_DLLAPI void rbdl_check_api_version(int version) {
  int compile_version = rbdl_get_api_version();

  int compile_major = (compile_version & 0xff0000) >> 16;
  int compile_minor = (compile_version & 0x00ff00) >> 8;
  int compile_patch = (compile_version & 0x0000ff);

  std::ostringstream compile_version_string("");
  compile_version_string << compile_major << "." << compile_minor << "." << compile_patch;

  int version_major = (version & 0xff0000) >> 16;
  int version_minor = (version & 0x00ff00) >> 8;
  int version_patch = (version & 0x0000ff);

  std::ostringstream link_version_string ("");
  link_version_string << version_major << "." << version_minor << "." << version_patch;

  if (version_major != compile_major) {
    std::cerr << "Error: trying to link against an incompatible RBDL library." << std::endl;
    std::cerr << "The library version is: " << compile_version_string.str() << " but rbdl_config.h is version " << link_version_string.str() << std::endl;
    abort();
  } else if (version_minor != compile_minor) {
    std::cout << "Warning: RBDL library is of version " << compile_version_string.str() << " but rbdl_config.h is from version " << link_version_string.str() << std::endl;
  }
}

RBDL_DLLAPI void rbdl_print_version() {
  int compile_version = rbdl_get_api_version();

  int compile_major = (compile_version & 0xff0000) >> 16;
  int compile_minor = (compile_version & 0x00ff00) >> 8;
  int compile_patch = (compile_version & 0x0000ff);

  std::ostringstream compile_version_string("");
  compile_version_string << compile_major << "." << compile_minor << "." << compile_patch;

  std::cout << "RBDL version:" << std::endl
    << "  API version  : " << compile_version_string.str() << std::endl;

  if (std::string("unknown") != RBDL_BUILD_REVISION) {
    std::cout << "  revision     : " << RBDL_BUILD_REVISION 
      << " (branch: " << RBDL_BUILD_BRANCH << ")" << std::endl
      << "  build type   : " << RBDL_BUILD_TYPE << std::endl;
  }

#ifdef RBDL_ENABLE_LOGGING
  std::cout << "  logging      : on (warning: reduces performance!)" << std::endl;
#else
  std::cout << "  logging      : off" << std::endl;
#endif
#ifdef RBDL_USE_SIMPLE_MATH
  std::cout << "  simplemath   : on (warning: reduces performance!)" << std::endl;
#else
  std::cout << "  simplemath   : off" << std::endl;
#endif

#ifdef RBDL_BUILD_ADDON_LUAMODEL
  std::cout << "  LuaModel     : on" << std::endl;
#else
  std::cout << "  LuaModel     : off" << std::endl;
#endif
#ifdef RBDL_BUILD_ADDON_URDFREADER
  std::cout << "  URDFReader   : on" << std::endl;
#else
  std::cout << "  URDFReader   : off" << std::endl;
#endif

  std::string build_revision (RBDL_BUILD_REVISION);
  if (build_revision == "unknown") {
    std::cout << std::endl << "Version information incomplete: to enable version information re-build" << std::endl << "library from valid repository and enable RBDL_STORE_VERSION." << std::endl;
  }
}
