/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_UTILS_H
#define RBDL_UTILS_H

#include <string>
#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_math.h>

namespace RigidBodyDynamics {

struct Model;

/** \brief Namespace that contains optional helper functions */
namespace Utils {
/** \brief Creates a human readable overview of the model. */
RBDL_DLLAPI std::string GetModelHierarchy (const Model &model);
/** \brief Creates a human readable overview of the Degrees of Freedom. */
RBDL_DLLAPI std::string GetModelDOFOverview (const Model &model);
/** \brief Creates a human readable overview of the locations of all bodies that have names. */
RBDL_DLLAPI std::string GetNamedBodyOriginsOverview (Model &model);

/** \brief Computes the Center of Mass (COM) and optionally its linear velocity.
 *
 * When only interested in computing the location of the COM you can use
 * NULL as value for com_velocity.
 *
 * \param model The model for which we want to compute the COM
 * \param q The current joint positions
 * \param qdot The current joint velocities
 * \param mass (output) total mass of the model
 * \param com (output) location of the Center of Mass of the model in base coordinates
 * \param qddot (optional input) A pointer to the current joint accelerations
 * \param com_velocity (optional output) linear velocity of the COM in base coordinates
 * \param com_acceleration (optional output) linear acceleration of the COM in base coordinates
 * \param angular_momentum (optional output) angular momentum of the model at the COM in base coordinates
 * \param change_of_angular_momentum (optional output) change of angular momentum of the model at the COM in base coordinates
 * \param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 *
 * \note When wanting to compute com_acceleration or change_of_angular
 * momentum one has to provide qddot. In all other cases one can use NULL
 * as argument for qddot.
 */
RBDL_DLLAPI void CalcCenterOfMass (
  Model &model,
  const Math::VectorNd &q,
  const Math::VectorNd &qdot,
  const Math::VectorNd *qddot,
  double &mass,
  Math::Vector3d &com,
  Math::Vector3d *com_velocity = NULL,
  Math::Vector3d *com_acceleration = NULL, 
  Math::Vector3d *angular_momentum = NULL,
  Math::Vector3d *change_of_angular_momentum = NULL,
  bool update_kinematics = true
);

/** \brief Computes the Zero-Moment-Point (ZMP) on a given contact surface.
 *
 * \param model The model for which we want to compute the ZMP
 * \param q The current joint positions
 * \param qdot The current joint velocities
 * \param qdot The current joint accelerations
 * \param normal The normal of the respective contact surface
 * \param point A point on the contact surface
 * \param zmp (output) location of the Zero-Moment-Point of the model in base coordinates projected on the given contact surface
 * \param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
 */
RBDL_DLLAPI void CalcZeroMomentPoint (
  Model &model,
  const Math::VectorNd &q, const Math::VectorNd &qdot, const Math::VectorNd &qddot,
  Math::Vector3d* zmp,
  const Math::Vector3d &normal = Math::Vector3d (0., 0., 1.),
  const Math::Vector3d &point = Math::Vector3d (0., 0., 0.),
  bool update_kinematics = true
);

/** \brief Computes the potential energy of the full model. */
RBDL_DLLAPI double CalcPotentialEnergy (Model &model, const Math::VectorNd &q, bool update_kinematics = true);

/** \brief Computes the kinetic energy of the full model. */
RBDL_DLLAPI double CalcKineticEnergy (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, bool update_kinematics = true);
}

}

/* RBDL_UTILS_H */
#endif
