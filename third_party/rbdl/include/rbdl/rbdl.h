/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_H
#define RBDL_H

#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_mathutils.h"

#include "rbdl/Logging.h"

#include "rbdl/Body.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Joint.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Constraints.h"

#include "rbdl/rbdl_utils.h"

/** \page api_version_checking_page API Changes
 * @{
 *
 * This documentation was created for API version 2.2.0.
 *
 * Here is a list of changes introduced by the different versions and what
 * adjustements have to be made to migrate.
 *
 * \include api_changes.txt
 */

/** Returns the API version at compile time of the library. */
RBDL_DLLAPI int rbdl_get_api_version();

/** Ensures whether the RBDL library we are linking against is compatible
 * with the the version we have from rbdl.h.
 *
 * To perform the check run:
 * \code
 *   rbdl_check_api_version(API_VERSION);
 * \endcode
 *
 * This function will abort if compatibility is not met or warn if you run
 * a version that might not be entirely compatible.
 *
 * In most cases you want to specify a specific version to ensure you are
 * using a compatible version. To do so replace API_VERSION by a
 * value of the form 0xAABBCC where AA is the major, BB the minor, and CC
 * the patch version in hex-format, e.g:
 *
 * \code
 *   rbdl_check_api_version(0x020A0C);
 * \endcode
 * 
 * Would abort if the API major version is not 2 (= 0x02), warn if the
 * linked minor version is not 10 (= 0x0A). The patch version 12 (= 0x12)
 * does not have an influence on compatibility.
 */
RBDL_DLLAPI void rbdl_check_api_version(int version);

/** Prints version information to standard output */
RBDL_DLLAPI void rbdl_print_version();

/* RBDL_H */
#endif
