/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_DYNAMICS_H
#define RBDL_DYNAMICS_H

#include <assert.h>
#include <iostream>

#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_mathutils.h"

#include "rbdl/Logging.h"

namespace RigidBodyDynamics {

struct Model;

/** \page dynamics_page Dynamics
 *
 * All functions related to kinematics are specified in the \ref
 * dynamics_group "Dynamics Module".
 *
 * \defgroup dynamics_group Dynamics
 * @{
 */

/** \brief Computes inverse dynamics with the Newton-Euler Algorithm
 *
 * This function computes the generalized forces from given generalized
 * states, velocities, and accelerations:
 *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}) \f$
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param QDDot accelerations of the internals joints
 * \param Tau   actuations of the internal joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
RBDL_DLLAPI void InverseDynamics (
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &QDot,
    const Math::VectorNd &QDDot,
    Math::VectorNd &Tau,
    std::vector<Math::SpatialVector> *f_ext = NULL
    );

/** \brief Computes the coriolis forces
 *
 * This function computes the generalized forces from given generalized
 * states, velocities, and accelerations:
 *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}) \f$
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
RBDL_DLLAPI void NonlinearEffects (
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &QDot,
    Math::VectorNd &Tau,
    std::vector<Math::SpatialVector> *f_ext = NULL
    );

/** \brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) \f$
 *
 * \param model rigid body model
 * \param Q     state vector of the model
 * \param H     a matrix where the result will be stored in
 * \param update_kinematics  whether the kinematics should be updated (safer, but at a higher computational cost!)
 *
 * \note This function only evaluates the entries of H that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling H.setZero().
 */
RBDL_DLLAPI void CompositeRigidBodyAlgorithm (
    Model& model,
    const Math::VectorNd &Q,
    Math::MatrixNd &H,
    bool update_kinematics = true
    );

/** \brief Computes forward dynamics with the Articulated Body Algorithm
 *
 * This function computes the generalized accelerations from given
 * generalized states, velocities and forces:
 *   \f$ \ddot{q} = M(q)^{-1} ( -N(q, \dot{q}) + \tau)\f$
 * It does this by using the recursive Articulated Body Algorithm that runs
 * in \f$O(n_{dof})\f$ with \f$n_{dof}\f$ being the number of joints.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
RBDL_DLLAPI void ForwardDynamics (
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &QDot,
    const Math::VectorNd &Tau,
    Math::VectorNd &QDDot,
    std::vector<Math::SpatialVector> *f_ext = NULL
    );

/** \brief Computes forward dynamics by building and solving the full Lagrangian equation
 *
 * This method builds and solves the linear system
 * \f[ 	H \ddot{q} = -C + \tau	\f]
 * for \f$\ddot{q}\f$ where \f$H\f$ is the joint space inertia matrix
 * computed with the CompositeRigidBodyAlgorithm(), \f$C\f$ the bias
 * force (sometimes called "non-linear effects").
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 * \param linear_solver specification which method should be used for solving the linear system
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 * \param H     preallocated workspace area for the joint space inertia matrix of size dof_count x dof_count (optional, defaults to NULL and allocates temporary matrix)
 * \param C     preallocated workspace area for the right hand side vector of size dof_count x 1 (optional, defaults to NULL and allocates temporary vector)
 */
RBDL_DLLAPI void ForwardDynamicsLagrangian (
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &QDot,
    const Math::VectorNd &Tau,
    Math::VectorNd &QDDot,
    Math::LinearSolver linear_solver = Math::LinearSolverColPivHouseholderQR,
    std::vector<Math::SpatialVector> *f_ext = NULL,
    Math::MatrixNd *H = NULL,
    Math::VectorNd *C = NULL	
    );

/** \brief Computes the effect of multiplying the inverse of the joint
 * space inertia matrix with a vector in linear time.
 *
 * \param model rigid body model
 * \param Q     state vector of the generalized positions
 * \param Tau   the vector that should be multiplied with the inverse of
 *              the joint space inertia matrix
 * \param QDDot vector where the result will be stored
 * \param update_kinematics whether the kinematics should be updated (safer, but at a higher computational cost)
 *
 * This function uses a reduced version of the Articulated %Body Algorithm
 * to compute: 
 *
 *   \f$ \ddot{q} = M(q)^{-1} \tau\f$
 *
 * for given \f$q\f$ and \f$\tau\f$ in \f$O(n_{\textit{dof}})\f$ time.
 *
 * \note When calling this function repeatedly for the same values of Q make sure
 * to set the last parameter to false as this avoids expensive
 * recomputations of transformations and articulated body inertias.
 */
RBDL_DLLAPI void CalcMInvTimesTau (
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &Tau,
    Math::VectorNd &QDDot,
    bool update_kinematics=true
    );

/** @} */

}

/* RBDL_DYNAMICS_H */
#endif
