/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_MATHUTILS_H
#define RBDL_MATHUTILS_H

#include <assert.h>
#include <cmath>

#include "rbdl/rbdl_math.h"

namespace RigidBodyDynamics {
struct Model;

namespace Math {

/** \brief Available solver methods for the linear systems.
 *
 * Please note that these methods are only available when Eigen3 is used.
 * When the math library SimpleMath is used it will always use a slow
 * column pivoting gauss elimination.
 */
enum RBDL_DLLAPI LinearSolver {
  LinearSolverUnknown = 0,
  LinearSolverPartialPivLU,
  LinearSolverColPivHouseholderQR,
  LinearSolverHouseholderQR,
  LinearSolverLLT,
  LinearSolverLast,
};

extern RBDL_DLLAPI Vector3d Vector3dZero;
extern RBDL_DLLAPI Matrix3d Matrix3dIdentity;
extern RBDL_DLLAPI Matrix3d Matrix3dZero;

RBDL_DLLAPI inline VectorNd VectorFromPtr (unsigned int n, double *ptr) {
  // TODO: use memory mapping operators for Eigen
  VectorNd result (n);

  for (unsigned int i = 0; i < n; i++) {
    result[i] = ptr[i];
  }

  return result;
}

RBDL_DLLAPI inline MatrixNd MatrixFromPtr (unsigned int rows, unsigned int cols, double *ptr, bool row_major = true) {
  MatrixNd result (rows, cols);

  if (row_major) {
    for (unsigned int i = 0; i < rows; i++) {
      for (unsigned int j = 0; j < cols; j++) {
        result(i,j) = ptr[i * cols + j];
      }
    }
  } else {
    for (unsigned int i = 0; i < rows; i++) {
      for (unsigned int j = 0; j < cols; j++) {
        result(i,j) = ptr[i + j * rows];
      }
    }
  }

  return result;
}

/// \brief Solves a linear system using gaussian elimination with pivoting
RBDL_DLLAPI bool LinSolveGaussElimPivot (MatrixNd A, VectorNd b, VectorNd &x);

// \todo write test 
RBDL_DLLAPI void SpatialMatrixSetSubmatrix(SpatialMatrix &dest, unsigned int row, unsigned int col, const Matrix3d &matrix);

RBDL_DLLAPI bool SpatialMatrixCompareEpsilon (const SpatialMatrix &matrix_a,
    const SpatialMatrix &matrix_b, double epsilon);
RBDL_DLLAPI bool SpatialVectorCompareEpsilon (const SpatialVector &vector_a,
    const SpatialVector &vector_b, double epsilon);

/** \brief Translates the inertia matrix to a new center. */
RBDL_DLLAPI Matrix3d parallel_axis (const Matrix3d &inertia, double mass, const Vector3d &com);

/** \brief Creates a transformation of a linear displacement
 *
 * This can be used to specify the translation to the joint center when
 * adding a body to a model. See also section 2.8 in RBDA.
 *
 * \note The transformation returned is for motions. For a transformation for forces
 * \note one has to conjugate the matrix.
 *
 * \param displacement The displacement as a 3D vector
 */
RBDL_DLLAPI SpatialMatrix Xtrans_mat (const Vector3d &displacement);

/** \brief Creates a rotational transformation around the Z-axis
 *
 * Creates a rotation around the current Z-axis by the given angle
 * (specified in radians).
 *
 * \param zrot Rotation angle in radians.
 */
RBDL_DLLAPI SpatialMatrix Xrotz_mat (const double &zrot);

/** \brief Creates a rotational transformation around the Y-axis
 *
 * Creates a rotation around the current Y-axis by the given angle
 * (specified in radians).
 *
 * \param yrot Rotation angle in radians.
 */
RBDL_DLLAPI SpatialMatrix Xroty_mat (const double &yrot);

/** \brief Creates a rotational transformation around the X-axis
 *
 * Creates a rotation around the current X-axis by the given angle
 * (specified in radians).
 *
 * \param xrot Rotation angle in radians.
 */
RBDL_DLLAPI SpatialMatrix Xrotx_mat (const double &xrot);

/** \brief Creates a spatial transformation for given parameters 
 *
 * Creates a transformation to a coordinate system that is first rotated
 * and then translated.
 *
 * \param displacement The displacement to the new origin
 * \param zyx_euler The orientation of the new coordinate system, specifyed
 * by ZYX-Euler angles.
 */
RBDL_DLLAPI SpatialMatrix XtransRotZYXEuler (const Vector3d &displacement, const Vector3d &zyx_euler);

RBDL_DLLAPI inline Matrix3d rotx (const double &xrot) {
  double s, c;
  s = sin (xrot);
  c = cos (xrot);
  return Matrix3d (
      1., 0., 0.,
      0., c, s,
      0., -s, c
      );
}

RBDL_DLLAPI inline Matrix3d roty (const double &yrot) {
  double s, c;
  s = sin (yrot);
  c = cos (yrot);
  return Matrix3d (
      c, 0., -s,
      0., 1., 0.,
      s, 0., c
      );
}

RBDL_DLLAPI inline Matrix3d rotz (const double &zrot) {
  double s, c;
  s = sin (zrot);
  c = cos (zrot);
  return Matrix3d (
      c, s, 0.,
      -s, c, 0.,
      0., 0., 1.
      );
}

RBDL_DLLAPI inline Matrix3d rotxdot (const double &x, const double &xdot) {
  double s, c;
  s = sin (x);
  c = cos (x);
  return Matrix3d (
      0., 0., 0.,
      0., -s * xdot, c * xdot,
      0., -c * xdot,-s * xdot 
      );
}

RBDL_DLLAPI inline Matrix3d rotydot (const double &y, const double &ydot) {
  double s, c;
  s = sin (y);
  c = cos (y);
  return Matrix3d (
      -s * ydot, 0., - c * ydot,
      0., 0., 0.,
      c * ydot, 0., - s * ydot 
      );
}

RBDL_DLLAPI inline Matrix3d rotzdot (const double &z, const double &zdot) {
  double s, c;
  s = sin (z);
  c = cos (z);
  return Matrix3d (
      -s * zdot, c * zdot, 0.,
      -c * zdot, -s * zdot, 0.,
      0., 0., 0.
      );
}

RBDL_DLLAPI inline Vector3d angular_velocity_from_angle_rates (const Vector3d &zyx_angles, const Vector3d &zyx_angle_rates) {
  double sy = sin(zyx_angles[1]);
  double cy = cos(zyx_angles[1]);
  double sx = sin(zyx_angles[2]);
  double cx = cos(zyx_angles[2]);

  return Vector3d (
      zyx_angle_rates[2] - sy * zyx_angle_rates[0],
      cx * zyx_angle_rates[1] + sx * cy * zyx_angle_rates[0],
      -sx * zyx_angle_rates[1] + cx * cy * zyx_angle_rates[0]
      );
}

RBDL_DLLAPI inline Vector3d global_angular_velocity_from_rates (const Vector3d &zyx_angles, const Vector3d &zyx_rates) {
  Matrix3d RzT = rotz(zyx_angles[0]).transpose();
  Matrix3d RyT = roty(zyx_angles[1]).transpose();

  return Vector3d (
      Vector3d (0., 0., zyx_rates[0])
      + RzT * Vector3d (0., zyx_rates[1], 0.)
      + RzT * RyT * Vector3d (zyx_rates[2], 0., 0.)
      );
}

RBDL_DLLAPI inline Vector3d angular_acceleration_from_angle_rates (const Vector3d &zyx_angles, const Vector3d &zyx_angle_rates, const Vector3d &zyx_angle_rates_dot) {
  double sy = sin(zyx_angles[1]);
  double cy = cos(zyx_angles[1]);
  double sx = sin(zyx_angles[2]);
  double cx = cos(zyx_angles[2]);
  double xdot = zyx_angle_rates[2];
  double ydot = zyx_angle_rates[1];
  double zdot = zyx_angle_rates[0];
  double xddot = zyx_angle_rates_dot[2];
  double yddot = zyx_angle_rates_dot[1];
  double zddot = zyx_angle_rates_dot[0];

  return Vector3d (
      xddot - (cy * ydot * zdot + sy * zddot),
      -sx * xdot * ydot + cx * yddot + cx * xdot * cy * zdot + sx * ( - sy * ydot * zdot + cy * zddot),
      -cx * xdot * ydot - sx * yddot - sx * xdot * cy * zdot + cx * ( - sy * ydot * zdot + cy * zddot)
      );
}

RBDL_DLLAPI
void SparseFactorizeLTL (Model &model, Math::MatrixNd &H);

RBDL_DLLAPI
void SparseMultiplyHx (Model &model, Math::MatrixNd &L);

RBDL_DLLAPI
void SparseMultiplyLx (Model &model, Math::MatrixNd &L);
RBDL_DLLAPI
void SparseMultiplyLTx (Model &model, Math::MatrixNd &L);

RBDL_DLLAPI
void SparseSolveLx (Model &model, Math::MatrixNd &L, Math::VectorNd &x);
RBDL_DLLAPI
void SparseSolveLTx (Model &model, Math::MatrixNd &L, Math::VectorNd &x); 

} /* Math */

} /* RigidBodyDynamics */

/* RBDL_MATHUTILS_H */
#endif
