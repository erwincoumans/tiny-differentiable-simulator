/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <cmath>
#include <limits>

#include <iostream>
#include <assert.h>

#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Model.h>

#include "rbdl/Logging.h"

namespace RigidBodyDynamics {
namespace Math {

RBDL_DLLAPI Vector3d Vector3dZero (0., 0., 0.);
RBDL_DLLAPI Matrix3d Matrix3dIdentity (
    1., 0., 0.,
    0., 1., 0.,
    0., 0., 1
    );
RBDL_DLLAPI Matrix3d Matrix3dZero (
    0., 0., 0.,
    0., 0., 0.,
    0., 0., 0.
    );

RBDL_DLLAPI SpatialVector SpatialVectorZero ( 0., 0., 0., 0., 0., 0.);

RBDL_DLLAPI bool LinSolveGaussElimPivot (MatrixNd A, VectorNd b, VectorNd &x) {
  x = VectorNd::Zero(x.size());

  // We can only solve quadratic systems
  assert (A.rows() == A.cols());

  unsigned int n = A.rows();
  unsigned int pi;

  // the pivots
  size_t *pivot = new size_t[n];

  // temporary result vector which contains the pivoted result
  VectorNd px(x);

  unsigned int i,j,k;

  for (i = 0; i < n; i++)
    pivot[i] = i;

  for (j = 0; j < n; j++) {
    pi = j;
    double pv = fabs (A(j,pivot[j]));

    // LOG << "j = " << j << " pv = " << pv << std::endl;
    // find the pivot
    for (k = j; k < n; k++) {
      double pt = fabs (A(j,pivot[k]));
      if (pt > pv) {
        pv = pt;
        pi = k;
        unsigned int p_swap = pivot[j];
        pivot[j] = pivot[pi];
        pivot[pi] = p_swap;
        //	LOG << "swap " << j << " with " << pi << std::endl;
        //	LOG << "j = " << j << " pv = " << pv << std::endl;
      }
    }

    for (i = j + 1; i < n; i++) {
      if (fabs(A(j,pivot[j])) <= std::numeric_limits<double>::epsilon()) {
        std::cerr << "Error: pivoting failed for matrix A = " << std::endl;
        std::cerr << "A = " << std::endl << A << std::endl;
        std::cerr << "b = " << b << std::endl;
      }
      //		assert (fabs(A(j,pivot[j])) > std::numeric_limits<double>::epsilon());
      double d = A(i,pivot[j])/A(j,pivot[j]);

      b[i] -= b[j] * d;

      for (k = j; k < n; k++) {
        A(i,pivot[k]) -= A(j,pivot[k]) * d;
      }
    }
  }

  // warning: i is an unsigned int, therefore a for loop of the 
  // form "for (i = n - 1; i >= 0; i--)" might end up in getting an invalid
  // value for i!
  i = n;
  do {
    i--;

    for (j = i + 1; j < n; j++) {
      px[i] += A(i,pivot[j]) * px[j];
    }
    px[i] = (b[i] - px[i]) / A(i,pivot[i]);

  } while (i > 0);

  // Unswapping
  for (i = 0; i < n; i++) {
    x[pivot[i]] = px[i];
  }

  /*
     LOG << "A = " << std::endl << A << std::endl;
     LOG << "b = " << b << std::endl;
     LOG << "x = " << x << std::endl;
     LOG << "pivot = " << pivot[0] << " " << pivot[1] << " " << pivot[2] << std::endl;
     std::cout << LogOutput.str() << std::endl;
     */

  delete[] pivot;

  return true;
}

RBDL_DLLAPI void SpatialMatrixSetSubmatrix(
    SpatialMatrix &dest, 
    unsigned int row, 
    unsigned int col, 
    const Matrix3d &matrix) {
  assert (row < 2 && col < 2);

  dest(row*3,col*3) = matrix(0,0);
  dest(row*3,col*3 + 1) = matrix(0,1);
  dest(row*3,col*3 + 2) = matrix(0,2);

  dest(row*3 + 1,col*3) = matrix(1,0);
  dest(row*3 + 1,col*3 + 1) = matrix(1,1);
  dest(row*3 + 1,col*3 + 2) = matrix(1,2);

  dest(row*3 + 2,col*3) = matrix(2,0);
  dest(row*3 + 2,col*3 + 1) = matrix(2,1);
  dest(row*3 + 2,col*3 + 2) = matrix(2,2);
}

RBDL_DLLAPI bool SpatialMatrixCompareEpsilon (
    const SpatialMatrix &matrix_a, 
    const SpatialMatrix &matrix_b, 
    double epsilon) {
  assert (epsilon >= 0.);
  unsigned int i, j;

  for (i = 0; i < 6; i++) {
    for (j = 0; j < 6; j++) {
      if (fabs(matrix_a(i,j) - matrix_b(i,j)) >= epsilon) {
        std::cerr << "Expected:" 
          << std::endl << matrix_a << std::endl
          << "but was" << std::endl 
          << matrix_b << std::endl;
        return false;
      }
    }
  }

  return true;
}

RBDL_DLLAPI bool SpatialVectorCompareEpsilon (
    const SpatialVector &vector_a, 
    const SpatialVector &vector_b, 
    double epsilon) {
  assert (epsilon >= 0.);
  unsigned int i;

  for (i = 0; i < 6; i++) {
    if (fabs(vector_a[i] - vector_b[i]) >= epsilon) {
      std::cerr << "Expected:" 
        << std::endl << vector_a << std::endl
        << "but was" << std::endl 
        << vector_b << std::endl;
      return false;
    }
  }

  return true;
}

RBDL_DLLAPI Matrix3d parallel_axis (
    const Matrix3d &inertia, 
    double mass, 
    const Vector3d &com) {
  Matrix3d com_cross = VectorCrossMatrix (com);

  return inertia + mass * com_cross * com_cross.transpose();
}

RBDL_DLLAPI SpatialMatrix Xtrans_mat (const Vector3d &r) {
  return SpatialMatrix (
      1.,    0.,    0.,  0.,  0.,  0.,
      0.,    1.,    0.,  0.,  0.,  0.,
      0.,    0.,    1.,  0.,  0.,  0.,
      0.,  r[2], -r[1],  1.,  0.,  0.,
      -r[2],    0.,  r[0],  0.,  1.,  0.,
      r[1], -r[0],    0.,  0.,  0.,  1.
      );
}

RBDL_DLLAPI SpatialMatrix Xrotx_mat (const double &xrot) {
  double s, c;
  s = sin (xrot);
  c = cos (xrot);

  return SpatialMatrix(
      1.,    0.,    0.,  0.,  0.,  0.,
      0.,     c,     s,  0.,  0.,  0.,
      0.,    -s,     c,  0.,  0.,  0.,
      0.,    0.,    0.,  1.,  0.,  0.,
      0.,    0.,    0.,  0.,   c,   s,
      0.,    0.,    0.,  0.,  -s,   c
      );
}

RBDL_DLLAPI SpatialMatrix Xroty_mat (const double &yrot) {
  double s, c;
  s = sin (yrot);
  c = cos (yrot);

  return SpatialMatrix(
      c,    0.,    -s,  0.,  0.,  0.,
      0.,    1.,    0.,  0.,  0.,  0.,
      s,    0.,     c,  0.,  0.,  0.,
      0.,    0.,    0.,   c,  0.,  -s,
      0.,    0.,    0.,  0.,  1.,  0.,
      0.,    0.,    0.,   s,  0.,   c
      );
}

RBDL_DLLAPI SpatialMatrix Xrotz_mat (const double &zrot) {
  double s, c;
  s = sin (zrot);
  c = cos (zrot);

  return SpatialMatrix(
      c,     s,    0.,  0.,  0.,  0.,
      -s,     c,    0.,  0.,  0.,  0.,
      0.,    0.,    1.,  0.,  0.,  0.,
      0.,    0.,    0.,   c,   s,  0.,
      0.,    0.,    0.,  -s,   c,  0.,
      0.,    0.,    0.,  0.,  0.,  1.
      );
}

RBDL_DLLAPI SpatialMatrix XtransRotZYXEuler (
    const Vector3d &displacement, 
    const Vector3d &zyx_euler) {
  return Xrotz_mat(zyx_euler[0]) * Xroty_mat(zyx_euler[1]) * Xrotx_mat(zyx_euler[2]) * Xtrans_mat(displacement);
}

RBDL_DLLAPI void SparseFactorizeLTL (Model &model, Math::MatrixNd &H) {
  for (unsigned int i = 0; i < model.qdot_size; i++) {
    for (unsigned int j = i + 1; j < model.qdot_size; j++) {
      H(i,j) = 0.;
    }
  }

  for (unsigned int k = model.qdot_size; k > 0; k--) {
    H(k - 1,k - 1) = sqrt (H(k - 1,k - 1));
    unsigned int i = model.lambda_q[k];
    while (i != 0) {
      H(k - 1,i - 1) = H(k - 1,i - 1) / H(k - 1,k - 1);
      i = model.lambda_q[i];
    }

    i = model.lambda_q[k];
    while (i != 0) {
      unsigned int j = i;
      while (j != 0) {
        H(i - 1,j - 1) = H(i - 1,j - 1) - H(k - 1,i - 1) * H(k - 1, j - 1);
        j = model.lambda_q[j];
      }
      i = model.lambda_q[i];
    }
  }
}

RBDL_DLLAPI void SparseMultiplyHx (Model &model, Math::MatrixNd &L) {
  assert (0 && !"Not yet implemented!");
}

RBDL_DLLAPI void SparseMultiplyLx (Model &model, Math::MatrixNd &L) {
  assert (0 && !"Not yet implemented!");
}

RBDL_DLLAPI void SparseMultiplyLTx (Model &model, Math::MatrixNd &L) {
  assert (0 && !"Not yet implemented!");
}

RBDL_DLLAPI void SparseSolveLx (Model &model, Math::MatrixNd &L, Math::VectorNd &x) {
  for (unsigned int i = 1; i <= model.qdot_size; i++) {
    unsigned int j = model.lambda_q[i];
    while (j != 0) {
      x[i - 1] = x[i - 1] - L(i - 1,j - 1) * x[j - 1];
      j = model.lambda_q[j];
    }
    x[i - 1] = x[i - 1] / L(i - 1,i - 1);
  }
}

RBDL_DLLAPI void SparseSolveLTx (Model &model, Math::MatrixNd &L, Math::VectorNd &x) {
  for (int i = model.qdot_size; i > 0; i--) {
    x[i - 1] = x[i - 1] / L(i - 1,i - 1);
    unsigned int j = model.lambda_q[i];
    while (j != 0) {
      x[j - 1] = x[j - 1] - L(i - 1,j - 1) * x[i - 1];
      j = model.lambda_q[j];
    }
  }
}

} /* Math */
} /* RigidBodyDynamics */
