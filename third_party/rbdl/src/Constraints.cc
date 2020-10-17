/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <sstream>
#include <limits>
#include <cassert>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "rbdl/Constraints.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

void SolveLinearSystem (
  const MatrixNd& A,
  const VectorNd& b,
  VectorNd& x,
  LinearSolver ls
);

unsigned int GetMovableBodyId (Model& model, unsigned int id);

unsigned int ConstraintSet::AddContactConstraint (
  unsigned int body_id,
  const Vector3d &body_point,
  const Vector3d &world_normal,
  const char *constraint_name,
  double normal_acceleration
  ) {
  assert (bound == false);

  unsigned int n_constr = size() + 1;

  std::string name_str;
  if (constraint_name != NULL) {
    name_str = constraint_name;
  }

  constraintType.push_back (ContactConstraint);
  name.push_back (name_str);
  mContactConstraintIndices.push_back(size());

  // These variables will be used for this type of constraint.
  body.push_back (body_id);
  point.push_back (body_point);
  normal.push_back (world_normal);

  // These variables will not be used.
  body_p.push_back (0);
  body_s.push_back (0);
  X_p.push_back (SpatialTransform());
  X_s.push_back (SpatialTransform());
  constraintAxis.push_back (SpatialVector::Zero());
  baumgarteParameters.push_back(Vector2d(0.0, 0.0));
  err.conservativeResize(n_constr);
  err[n_constr - 1] = 0.;
  errd.conservativeResize(n_constr);
  errd[n_constr - 1] = 0.;

  acceleration.conservativeResize (n_constr);
  acceleration[n_constr - 1] = normal_acceleration;

  force.conservativeResize (n_constr);
  force[n_constr - 1] = 0.;

  impulse.conservativeResize (n_constr);
  impulse[n_constr - 1] = 0.;

  v_plus.conservativeResize (n_constr);
  v_plus[n_constr - 1] = 0.;

  d_multdof3_u = std::vector<Math::Vector3d>(n_constr, Math::Vector3d::Zero());

  return n_constr - 1;
}

unsigned int ConstraintSet::AddLoopConstraint (
  unsigned int id_predecessor, 
  unsigned int id_successor,
  const Math::SpatialTransform &X_predecessor,
  const Math::SpatialTransform &X_successor,
  const Math::SpatialVector &axis,
  bool enable_stabilization,
  const double stabilization_param,
  const char *constraint_name
  ) {
  assert (bound == false);

  unsigned int n_constr = size() + 1;

  std::string name_str;
  if (constraint_name != NULL) {
    name_str = constraint_name;
  }

  constraintType.push_back(LoopConstraint);
  name.push_back (name_str);
  mLoopConstraintIndices.push_back(size());

  // These variables will be used for this kind of constraint.
  body_p.push_back (id_predecessor);
  body_s.push_back (id_successor);
  X_p.push_back (X_predecessor);
  X_s.push_back (X_successor);
  constraintAxis.push_back (axis);

  // Set up constraint stabilization
  double baumgarte_coefficient = 0.0;
  if (enable_stabilization) {
    if (stabilization_param == 0.0) {
      std::cerr << "Error: Baumgarte stabilization is enabled but the stabilization parameter is 0.0" << std::endl;
      abort();
    }
    baumgarte_coefficient = 1.0 / stabilization_param;
  }
  baumgarteParameters.push_back(
      Vector2d(baumgarte_coefficient, baumgarte_coefficient));

  err.conservativeResize(n_constr);
  err[n_constr - 1] = 0.;
  errd.conservativeResize(n_constr);
  errd[n_constr - 1] = 0.;

  // These variables will not be used by loop constraints but are necessary
  // for point constraints.
  body.push_back (0);
  point.push_back (Vector3d::Zero());
  normal.push_back (Vector3d::Zero());

  acceleration.conservativeResize (n_constr);
  acceleration[n_constr - 1] = 0.;

  force.conservativeResize (n_constr);
  force[n_constr - 1] = 0.;

  impulse.conservativeResize (n_constr);
  impulse[n_constr - 1] = 0.;

  v_plus.conservativeResize (n_constr);
  v_plus[n_constr - 1] = 0.;

  d_multdof3_u = std::vector<Math::Vector3d>(n_constr, Math::Vector3d::Zero());

  return n_constr - 1;
}

unsigned int ConstraintSet::AddCustomConstraint(
  CustomConstraint *customConstraint,
  unsigned int id_predecessor,
  unsigned int id_successor,
  const Math::SpatialTransform &X_predecessor,
  const Math::SpatialTransform &X_successor,
  bool enable_stabilization,
  const double stabilization_param,
  const char *constraint_name) {
  assert (bound == false);

  unsigned int n_constr_start_idx = size();
  unsigned int n_constr_size      = size() + customConstraint->mConstraintCount;

  std::string name_str;
  if (constraint_name != NULL) {
    name_str = constraint_name;
  }

  SpatialVector axis = SpatialVector::Zero();
  std::stringstream nameConstraintIndex;

  for(unsigned int i = 0; i < customConstraint->mConstraintCount; ++i ){
      constraintType.push_back( ConstraintTypeCustom );
      nameConstraintIndex << name_str << "_" << i;
      name.push_back (nameConstraintIndex.str());
      nameConstraintIndex.str(std::string());
      if(i==0){
        mCustomConstraintIndices.push_back(n_constr_start_idx);
      }
      // These variables will be used for each CustomConstraint
      body_p.push_back (id_predecessor);
      body_s.push_back (id_successor);
      X_p.push_back (X_predecessor);
      X_s.push_back (X_successor);
      constraintAxis.push_back (axis);

      // Set up constraint stabilization
      double baumgarte_coefficient = 0.0;
      if (enable_stabilization) {
        if (stabilization_param == 0.0) {
          std::cerr << "Error: Baumgarte stabilization is enabled but the stabilization parameter is 0.0" << std::endl;
          abort();
        }
        baumgarte_coefficient = 1.0 / stabilization_param;
      }
      baumgarteParameters.push_back(
          Vector2d(baumgarte_coefficient, baumgarte_coefficient));

      // These variables will not be used in CustomConstraints but are kept
      // so that the indexing across all of the ConstraintSet variables
      // remains preserved.
      body.push_back (0);
      point.push_back (Vector3d::Zero());
      normal.push_back (Vector3d::Zero());
  }

  err.conservativeResize( n_constr_size);
  errd.conservativeResize(n_constr_size);

  acceleration.conservativeResize ( n_constr_size);
  force.conservativeResize (        n_constr_size);
  impulse.conservativeResize (      n_constr_size);
  v_plus.conservativeResize (       n_constr_size);
  d_multdof3_u = std::vector<Math::Vector3d>(
                  n_constr_size, Math::Vector3d::Zero());

  for(unsigned int i = n_constr_start_idx; i < n_constr_size; i++){
      err[i]          = 0.;
      errd[i]         = 0.;
      acceleration[i] = 0.;
      force[i]        = 0.;
      impulse[i]      = 0.;
      v_plus[i]       = 0.;
  }

   mCustomConstraints.push_back(customConstraint);

  return n_constr_size - 1;
}

bool ConstraintSet::Bind (const Model &model) {
  assert (bound == false);

  if (bound) {
    std::cerr << "Error: binding an already bound constraint set!" << std::endl;
    abort();
  }
  unsigned int n_constr = size();

  H.conservativeResize (model.dof_count, model.dof_count);
  H.setZero();
  C.conservativeResize (model.dof_count);
  C.setZero();
  gamma.conservativeResize (n_constr);
  gamma.setZero();
  G.conservativeResize (n_constr, model.dof_count);
  G.setZero();
  A.conservativeResize (model.dof_count + n_constr, model.dof_count + n_constr);
  A.setZero();
  b.conservativeResize (model.dof_count + n_constr);
  b.setZero();
  x.conservativeResize (model.dof_count + n_constr);
  x.setZero();

  Gi.conservativeResize (3, model.qdot_size);
  GSpi.conservativeResize (6, model.qdot_size);
  GSsi.conservativeResize (6, model.qdot_size);
  GSJ.conservativeResize (6, model.qdot_size);

  // HouseHolderQR crashes if matrix G has more rows than columns.
#ifdef RBDL_USE_SIMPLE_MATH
  GT_qr = SimpleMath::HouseholderQR<Math::MatrixNd> (G.transpose());
#else
  GT_qr = Eigen::HouseholderQR<Math::MatrixNd> (G.transpose());
#endif
  GT_qr_Q = MatrixNd::Zero (model.dof_count, model.dof_count);
  Y = MatrixNd::Zero (model.dof_count, G.rows());
  Z = MatrixNd::Zero (model.dof_count, model.dof_count - G.rows());
  qddot_y = VectorNd::Zero (model.dof_count);
  qddot_z = VectorNd::Zero (model.dof_count);

  K.conservativeResize (n_constr, n_constr);
  K.setZero();
  a.conservativeResize (n_constr);
  a.setZero();
  QDDot_t.conservativeResize (model.dof_count);
  QDDot_t.setZero();
  QDDot_0.conservativeResize (model.dof_count);
  QDDot_0.setZero();
  f_t.resize (n_constr, SpatialVector::Zero());
  f_ext_constraints.resize (model.mBodies.size(), SpatialVector::Zero());
  point_accel_0.resize (n_constr, Vector3d::Zero());

  d_pA =std::vector<SpatialVector> (model.mBodies.size(),SpatialVector::Zero());
  d_a = std::vector<SpatialVector> (model.mBodies.size(),SpatialVector::Zero());
  d_u = VectorNd::Zero (model.mBodies.size());

  d_IA = std::vector<SpatialMatrix> (model.mBodies.size()
    , SpatialMatrix::Identity());
  d_U = std::vector<SpatialVector> (model.mBodies.size(),SpatialVector::Zero());
  d_d = VectorNd::Zero (model.mBodies.size());

  d_multdof3_u = std::vector<Math::Vector3d> (model.mBodies.size()
    , Math::Vector3d::Zero());

  bound = true;

  return bound;
}

void ConstraintSet::clear() {
  acceleration.setZero();
  force.setZero();
  impulse.setZero();

  H.setZero();
  C.setZero();
  gamma.setZero();
  G.setZero();
  A.setZero();
  b.setZero();
  x.setZero();

  K.setZero();
  a.setZero();
  QDDot_t.setZero();
  QDDot_0.setZero();

  unsigned int i;
  for (i = 0; i < f_t.size(); i++)
    f_t[i].setZero();

  for (i = 0; i < f_ext_constraints.size(); i++)
    f_ext_constraints[i].setZero();

  for (i = 0; i < point_accel_0.size(); i++)
    point_accel_0[i].setZero();

  for (i = 0; i < d_pA.size(); i++)
    d_pA[i].setZero();

  for (i = 0; i < d_a.size(); i++)
    d_a[i].setZero();

  d_u.setZero();
}

RBDL_DLLAPI
void SolveConstrainedSystemDirect (
  Math::MatrixNd &H, 
  const Math::MatrixNd &G, 
  const Math::VectorNd &c, 
  const Math::VectorNd &gamma, 
  Math::VectorNd &qddot, 
  Math::VectorNd &lambda, 
  Math::MatrixNd &A, 
  Math::VectorNd &b,
  Math::VectorNd &x,
  Math::LinearSolver &linear_solver
  ) {
  // Build the system: Copy H
  A.block(0, 0, c.rows(), c.rows()) = H;

  // Copy G and G^T
  A.block(0, c.rows(), c.rows(), gamma.rows()) = G.transpose();
  A.block(c.rows(), 0, gamma.rows(), c.rows()) = G;

  // Build the system: Copy -C + \tau
  b.block(0, 0, c.rows(), 1) = c;
  b.block(c.rows(), 0, gamma.rows(), 1) = gamma;

  LOG << "A = " << std::endl << A << std::endl;
  LOG << "b = " << std::endl << b << std::endl;

  switch (linear_solver) {
    case (LinearSolverPartialPivLU) :
      x = A.partialPivLu().solve(b);
      break;
    case (LinearSolverColPivHouseholderQR) :
      x = A.colPivHouseholderQr().solve(b);
      break;
    case (LinearSolverHouseholderQR) :
      x = A.householderQr().solve(b);
      break;
    default:
      LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
      assert (0);
      break;
  }

  LOG << "x = " << std::endl << x << std::endl;
}

RBDL_DLLAPI
void SolveConstrainedSystemRangeSpaceSparse (
  Model &model, 
  Math::MatrixNd &H, 
  const Math::MatrixNd &G, 
  const Math::VectorNd &c, 
  const Math::VectorNd &gamma, 
  Math::VectorNd &qddot, 
  Math::VectorNd &lambda, 
  Math::MatrixNd &K, 
  Math::VectorNd &a,
  Math::LinearSolver linear_solver
  ) {
  SparseFactorizeLTL (model, H);

  MatrixNd Y (G.transpose());

  for (unsigned int i = 0; i < Y.cols(); i++) {
    VectorNd Y_col = Y.block(0,i,Y.rows(),1);
    SparseSolveLTx (model, H, Y_col);
    Y.block(0,i,Y.rows(),1) = Y_col;
  }

  VectorNd z (c);
  SparseSolveLTx (model, H, z);

  K = Y.transpose() * Y;

  a = gamma - Y.transpose() * z;

  lambda = K.llt().solve(a);

  qddot = c + G.transpose() * lambda;
  SparseSolveLTx (model, H, qddot);
  SparseSolveLx (model, H, qddot);
}

RBDL_DLLAPI
void SolveConstrainedSystemNullSpace (
  Math::MatrixNd &H, 
  const Math::MatrixNd &G, 
  const Math::VectorNd &c, 
  const Math::VectorNd &gamma, 
  Math::VectorNd &qddot, 
  Math::VectorNd &lambda,
  Math::MatrixNd &Y,
  Math::MatrixNd &Z,
  Math::VectorNd &qddot_y,
  Math::VectorNd &qddot_z,
  Math::LinearSolver &linear_solver
  ) {
  switch (linear_solver) {
    case (LinearSolverPartialPivLU) :
      qddot_y = (G * Y).partialPivLu().solve (gamma);
      break;
    case (LinearSolverColPivHouseholderQR) :
      qddot_y = (G * Y).colPivHouseholderQr().solve (gamma);
      break;
    case (LinearSolverHouseholderQR) :
      qddot_y = (G * Y).householderQr().solve (gamma);
      break;
    default:
      LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
      assert (0);
      break;
  }

  qddot_z = (Z.transpose() * H * Z).llt().solve(Z.transpose() * (c - H * Y * qddot_y));

  qddot = Y * qddot_y + Z * qddot_z;

  switch (linear_solver) {
    case (LinearSolverPartialPivLU) :
      lambda = (G * Y).partialPivLu().solve (Y.transpose() * (H * qddot - c));
      break;
    case (LinearSolverColPivHouseholderQR) :
      lambda = (G * Y).colPivHouseholderQr().solve (Y.transpose() * (H * qddot - c));
      break;
    case (LinearSolverHouseholderQR) :
      lambda = (G * Y).householderQr().solve (Y.transpose() * (H * qddot - c));
      break;
    default:
      LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
      assert (0);
      break;
  }
}

RBDL_DLLAPI
void CalcConstraintsPositionError (
  Model& model,
  const Math::VectorNd &Q,
  ConstraintSet &CS,
  Math::VectorNd& err,
  bool update_kinematics
  ) {
  assert(err.size() == CS.size());

  if(update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  for (unsigned int i = 0; i < CS.mContactConstraintIndices.size(); i++) {
    const unsigned int c = CS.mContactConstraintIndices[i];
    err[c] = 0.;
  }

  for (unsigned int i = 0; i < CS.mLoopConstraintIndices.size(); i++) {
    const unsigned int lci = CS.mLoopConstraintIndices[i];

    // Variables used for computations.
    Vector3d pos_p;
    Vector3d pos_s;
    Matrix3d rot_p;
    Matrix3d rot_s;
    Matrix3d rot_ps;
    SpatialVector d;

    // Constraints computed in the predecessor body frame.

    // Compute the orientation of the two constraint frames.
    rot_p = CalcBodyWorldOrientation (model, Q, CS.body_p[lci], false).transpose()
      * CS.X_p[lci].E;
    rot_s = CalcBodyWorldOrientation (model, Q, CS.body_s[lci], false).transpose()
      * CS.X_s[lci].E;

    // Compute the orientation from the predecessor to the successor frame.
    rot_ps = rot_p.transpose() * rot_s;

    // Compute the position of the two contact points.
    pos_p = CalcBodyToBaseCoordinates (model, Q, CS.body_p[lci], CS.X_p[lci].r
      , false);
    pos_s = CalcBodyToBaseCoordinates (model, Q, CS.body_s[lci], CS.X_s[lci].r
      , false);

    // The first three elements represent the rotation error.
    // This formulation is equivalent to u * sin(theta), where u and theta are
    // the angle-axis of rotation from the predecessor to the successor frame.
    // These quantities are expressed in the predecessor frame.
    d[0] = -0.5 * (rot_ps(1,2) - rot_ps(2,1));
    d[1] = -0.5 * (rot_ps(2,0) - rot_ps(0,2));
    d[2] = -0.5 * (rot_ps(0,1) - rot_ps(1,0));

    // The last three elements represent the position error.
    // It is equivalent to the difference in the position of the two
    // constraint points.
    // The distance is projected on the predecessor frame to be consistent
    // with the rotation.
    d.block<3,1>(3,0) = rot_p.transpose() * (pos_s - pos_p);

    // Project the error on the constraint axis to find the actual error.
    err[lci] = CS.constraintAxis[lci].transpose() * d;
  } 

  for (unsigned int i = 0; i < CS.mCustomConstraintIndices.size(); i++) {
    const unsigned int cci = CS.mCustomConstraintIndices[i];
    CS.mCustomConstraints[i]->CalcPositionError(model,cci,Q,CS,err, cci);    
  }
}

RBDL_DLLAPI
void CalcConstraintsJacobian (
  Model &model,
  const Math::VectorNd &Q,
  ConstraintSet &CS,
  Math::MatrixNd &G,
  bool update_kinematics
  ) {
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  // variables to check whether we need to recompute G.
  unsigned int prev_body_id_1 = 0;
  unsigned int prev_body_id_2 = 0;
  SpatialTransform prev_body_X_1;
  SpatialTransform prev_body_X_2;

  for (unsigned int i = 0; i < CS.mContactConstraintIndices.size(); i++) {
    const unsigned int c = CS.mContactConstraintIndices[i];

    // only compute the matrix Gi if actually needed
    if (prev_body_id_1 != CS.body[c] 
        || prev_body_X_1.r != CS.point[c]) {

      // Compute the jacobian for the point.
      CS.Gi.setZero();
      CalcPointJacobian (model, Q, CS.body[c], CS.point[c], CS.Gi, false);

      // Update variables for optimization check.
      prev_body_id_1 = CS.body[c];
      prev_body_X_1 = Xtrans(CS.point[c]);
    }

    for(unsigned int j = 0; j < model.dof_count; j++) {
      Vector3d gaxis (CS.Gi(0,j), CS.Gi(1,j), CS.Gi(2,j));
      G(c,j) = gaxis.transpose() * CS.normal[c];
    }
  }

  // Variables used for computations.
  Vector3d normal;
  SpatialVector axis;
  Vector3d pos_p;
  Matrix3d rot_p;
  SpatialTransform X_0p;

  for (unsigned int i = 0; i < CS.mLoopConstraintIndices.size(); i++) {
    const unsigned int c = CS.mLoopConstraintIndices[i];

    // Only recompute variables if necessary.
    if( prev_body_id_1 != CS.body_p[c]
        || prev_body_id_2 != CS.body_s[c]
        || prev_body_X_1.r != CS.X_p[c].r
        || prev_body_X_2.r != CS.X_s[c].r
        || prev_body_X_1.E != CS.X_p[c].E
        || prev_body_X_2.E != CS.X_s[c].E) {

      // Compute the 6D jacobians of the two contact points.
      CS.GSpi.setZero();
      CS.GSsi.setZero();
      CalcPointJacobian6D(model, Q, CS.body_p[c], CS.X_p[c].r, CS.GSpi, false);
      CalcPointJacobian6D(model, Q, CS.body_s[c], CS.X_s[c].r, CS.GSsi, false);
      CS.GSJ = CS.GSsi - CS.GSpi;

      // Compute position and rotation matrix from predecessor body to base.
      pos_p = CalcBodyToBaseCoordinates (model, Q, CS.body_p[c], CS.X_p[c].r
          , false);
      rot_p = CalcBodyWorldOrientation (model, Q, CS.body_p[c]
          , false).transpose()* CS.X_p[c].E;
      X_0p = SpatialTransform (rot_p, pos_p);

      // Update variables for optimization check.
      prev_body_id_1 = CS.body_p[c];
      prev_body_id_2 = CS.body_s[c];
      prev_body_X_1 = CS.X_p[c];
      prev_body_X_2 = CS.X_s[c];
    }

    // Express the constraint axis in the base frame.
    axis = X_0p.apply(CS.constraintAxis[c]);

    // Compute the constraint Jacobian row.
    G.block(c, 0, 1, model.dof_count) = axis.transpose() * CS.GSJ;
  }

  // Go and get the CustomConstraint Jacobians
  for (unsigned int i = 0; i < CS.mCustomConstraintIndices.size(); i++) {
    const unsigned int cci = CS.mCustomConstraintIndices[i];
    const unsigned int rows= CS.mCustomConstraints[i]->mConstraintCount;
    const unsigned int cols= CS.G.cols();
    CS.mCustomConstraints[i]->CalcConstraintsJacobianAndConstraintAxis(
                                 model,cci,Q,CS,G, cci,0);
  }
}

RBDL_DLLAPI
void CalcConstraintsVelocityError (
  Model& model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  ConstraintSet &CS,
  Math::VectorNd& err,
  bool update_kinematics
  ) {
  
  //This works for the contact and loop constraints because they are
  //time invariant. But this does not necessarily work for the CustomConstraints
  //which can be time-varying. And thus the parts of err associated with the
  //custom constraints must be updated.
  //MatrixNd G(MatrixNd::Zero(CS.size(), model.dof_count));
  CalcConstraintsJacobian (model, Q, CS, CS.G, update_kinematics);
  err = CS.G * QDot;
  
  unsigned int cci, rows, cols;
  for (unsigned int i = 0; i < CS.mCustomConstraintIndices.size(); i++) {
    cci = CS.mCustomConstraintIndices[i];
    rows= CS.mCustomConstraints[i]->mConstraintCount;
    cols= CS.G.cols();
    CS.mCustomConstraints[i]->CalcVelocityError(model,cci,Q,QDot,CS,
                               CS.G.block(cci,0,rows,cols),err, cci);
  }


}

RBDL_DLLAPI
void CalcConstrainedSystemVariables (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  std::vector<Math::SpatialVector> *f_ext
  ) {
  // Compute C
  NonlinearEffects(model, Q, QDot, CS.C, f_ext);
  assert(CS.H.cols() == model.dof_count && CS.H.rows() == model.dof_count);

  // Compute H
  CS.H.setZero();
  CompositeRigidBodyAlgorithm(model, Q, CS.H, false);

  // Compute G
  // We have to update model.X_base as they are not automatically computed
  // by NonlinearEffects()
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    model.X_base[i] = model.X_lambda[i] * model.X_base[model.lambda[i]];
  }
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  // Compute position error for Baumgarte Stabilization.
  CalcConstraintsPositionError (model, Q, CS, CS.err, false);

  // Compute velocity error for Baugarte stabilization.
  CalcConstraintsVelocityError (model, Q, QDot, CS, CS.errd, false);  
  //CS.errd = CS.G * QDot;

  // Compute gamma
  unsigned int prev_body_id = 0;
  Vector3d prev_body_point = Vector3d::Zero();
  Vector3d gamma_i = Vector3d::Zero();

  CS.QDDot_0.setZero();
  UpdateKinematicsCustom(model, NULL, NULL, &CS.QDDot_0);

  for (unsigned int i = 0; i < CS.mContactConstraintIndices.size(); i++) {
    const unsigned int c = CS.mContactConstraintIndices[i];

    // only compute point accelerations when necessary
    if (prev_body_id != CS.body[c] || prev_body_point != CS.point[c]) {
      gamma_i = CalcPointAcceleration (model, Q, QDot, CS.QDDot_0, CS.body[c]
          , CS.point[c], false);
      prev_body_id = CS.body[c];
      prev_body_point = CS.point[c];
    }

    // we also substract ContactData[c].acceleration such that the contact
    // point will have the desired acceleration
    CS.gamma[c] = CS.acceleration[c] - CS.normal[c].dot(gamma_i);
  }



  for (unsigned int i = 0; i < CS.mLoopConstraintIndices.size(); i++) {
    const unsigned int c = CS.mLoopConstraintIndices[i];

    // Variables used for computations.
    Vector3d pos_p;
    Matrix3d rot_p;
    SpatialVector vel_p;
    SpatialVector vel_s;
    SpatialVector axis;
    unsigned int id_p;
    unsigned int id_s;


    // Express the constraint axis in the base frame.
    pos_p = CalcBodyToBaseCoordinates (model, Q, CS.body_p[c], 
                                      CS.X_p[c].r,false);
    rot_p = CalcBodyWorldOrientation (model, Q, CS.body_p[c], false
                                        ).transpose() * CS.X_p[c].E;
    axis = SpatialTransform (rot_p, pos_p).apply(CS.constraintAxis[c]);

    // Compute the spatial velocities of the two constrained bodies.
    vel_p = CalcPointVelocity6D (model, Q, QDot, CS.body_p[c],
                                  CS.X_p[c].r, false);
    vel_s = CalcPointVelocity6D (model, Q, QDot, CS.body_s[c],
                                  CS.X_s[c].r, false);

    // Compute the derivative of the axis wrt the base frame.
    SpatialVector axis_dot = crossm(vel_p, axis);

    // Compute the velocity product accelerations. These correspond to the
    // accelerations that the bodies would have if q ddot were 0.
    SpatialVector acc_p = CalcPointAcceleration6D (model, Q, QDot
      , VectorNd::Zero(model.dof_count), CS.body_p[c], CS.X_p[c].r, false);
    SpatialVector acc_s = CalcPointAcceleration6D (model, Q, QDot
      , VectorNd::Zero(model.dof_count), CS.body_s[c], CS.X_s[c].r, false);

    // Problem here if one of the bodies is fixed...
    // Compute the value of gamma.
    CS.gamma[c]
      // Right hand side term.
      = - axis.dot(acc_s - acc_p) - axis_dot.dot(vel_s - vel_p)
      // Baumgarte stabilization term.
      - 2. * CS.baumgarteParameters[c][0] * CS.errd[c]
      - CS.baumgarteParameters[c][1] * CS.baumgarteParameters[c][1] * CS.err[c];
  }

  unsigned int ccid,rows,cols,z;
  for(unsigned int i=0; i< CS.mCustomConstraintIndices.size(); i++){
    ccid  = CS.mCustomConstraintIndices[i];
    rows  = CS.mCustomConstraints[i]->mConstraintCount;
    cols  = CS.G.cols();
    CS.mCustomConstraints[i]->CalcGamma(model,ccid,Q,QDot,CS,
                                        CS.G.block(ccid,0,rows,cols),
                                        CS.gamma,ccid);
    for(unsigned int j=0; j<CS.mCustomConstraints[i]->mConstraintCount;j++){
      z = ccid+j;
      CS.gamma[z] += (- 2. * CS.baumgarteParameters[z][0] * CS.errd[z]
                      - CS.baumgarteParameters[z][1] 
                      * CS.baumgarteParameters[z][1] * CS.err[z]);
    }

  }

}

RBDL_DLLAPI
bool CalcAssemblyQ (
  Model &model,
  Math::VectorNd QInit, // Note: passed by value intentionally
  ConstraintSet &cs,
  Math::VectorNd &Q,
  const Math::VectorNd &weights,
  double tolerance,
  unsigned int max_iter
  ) {

  if(Q.size() != model.q_size) {
    std::cerr << "Incorrect Q vector size." << std::endl;
    assert(false);
    abort();
  }
  if(QInit.size() != model.q_size) {
    std::cerr << "Incorrect QInit vector size." << std::endl;
    assert(false);
    abort();
  }
  if(weights.size() != model.dof_count) {
    std::cerr << "Incorrect weights vector size." << std::endl;
    assert(false);
    abort();
  }

  // Initialize variables.
  MatrixNd constraintJac (cs.size(), model.dof_count);
  MatrixNd A = MatrixNd::Zero (cs.size() + model.dof_count, cs.size() 
    + model.dof_count);
  VectorNd b = VectorNd::Zero (cs.size() + model.dof_count);
  VectorNd x = VectorNd::Zero (cs.size() + model.dof_count);
  VectorNd d = VectorNd::Zero (model.dof_count);
  VectorNd e = VectorNd::Zero (cs.size());

  // The top-left block is the weight matrix and is constant.
  for(unsigned int i = 0; i < weights.size(); ++i) {
    A(i,i) = weights[i];
  }

  // Check if the error is small enough already. If so, just return the initial
  // guess as the solution.
  CalcConstraintsPositionError (model, QInit, cs, e);
  if (e.norm() < tolerance) {
    Q = QInit;
    return true;
  }

  // We solve the linearized problem iteratively.
  // Iterations are stopped if the maximum is reached.
  for(unsigned int it = 0; it < max_iter; ++it) {
    // Compute the constraint jacobian and build A and b.
    constraintJac.setZero();
    CalcConstraintsJacobian (model, QInit, cs, constraintJac);
    A.block (model.dof_count, 0, cs.size(), model.dof_count) = constraintJac;
    A.block (0, model.dof_count, model.dof_count, cs.size()) 
      = constraintJac.transpose();
    b.block (model.dof_count, 0, cs.size(), 1) = -e;

    // Solve the sistem A*x = b.
    SolveLinearSystem (A, b, x, cs.linear_solver);

    // Extract the d = (Q - QInit) vector from x.
    d = x.block (0, 0, model.dof_count, 1);

    // Update solution.
    for (size_t i = 0; i < model.mJoints.size(); ++i) {
      // If the joint is spherical, translate the corresponding components
      // of d into a modification in the joint quaternion.
      if (model.mJoints[i].mJointType == JointTypeSpherical) {
        Quaternion quat = model.GetQuaternion(i, QInit);
        Vector3d omega = d.block<3,1>(model.mJoints[i].q_index,0);
        // Convert the 3d representation of the displacement to 4d and sum it
        // to the components of the quaternion.
        quat += quat.omegaToQDot(omega);
        // The quaternion needs to be normalized after the previous sum.
        quat /= quat.norm();
        model.SetQuaternion(i, quat, QInit);
      }
      // If the current joint is not spherical, simply add the corresponding
      // components of d to QInit.
      else {
        unsigned int qIdx = model.mJoints[i].q_index;
        for(size_t j = 0; j < model.mJoints[i].mDoFCount; ++j) {
          QInit[qIdx + j] += d[qIdx + j];
        }
        // QInit.block(qIdx, 0, model.mJoints[i].mDoFCount, 1)
        //   += d.block(model.mJoints[i].q_index, 0, model.mJoints[i].mDoFCount, 1);
      }
    }

    // Update the errors.
    CalcConstraintsPositionError (model, QInit, cs, e);

    // Check if the error and the step are small enough to end.
    if (e.norm() < tolerance && d.norm() < tolerance){
      Q = QInit;
      return true;
    }
  }

  // Return false if maximum number of iterations is exceeded.
  Q = QInit;
  return false;
}

RBDL_DLLAPI
void CalcAssemblyQDot (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotInit,
  ConstraintSet &cs,
  Math::VectorNd &QDot,
  const Math::VectorNd &weights
  ) {
  if(QDot.size() != model.dof_count) {
    std::cerr << "Incorrect QDot vector size." << std::endl;
    assert(false);
    abort();
  }
  if(Q.size() != model.q_size) {
    std::cerr << "Incorrect Q vector size." << std::endl;
    assert(false);
    abort();
  }
  if(QDotInit.size() != QDot.size()) {
    std::cerr << "Incorrect QDotInit vector size." << std::endl;
    assert(false);
    abort();
  }
  if(weights.size() != QDot.size()) {
    std::cerr << "Incorrect weight vector size." << std::endl;
    assert(false);
    abort();
  }

  // Initialize variables.
  MatrixNd constraintJac = MatrixNd::Zero(cs.size(), model.dof_count);
  MatrixNd A = MatrixNd::Zero(cs.size() + model.dof_count, cs.size() 
    + model.dof_count);
  VectorNd b = VectorNd::Zero(cs.size() + model.dof_count);
  VectorNd x = VectorNd::Zero(cs.size() + model.dof_count);

  // The top-left block is the weight matrix and is constant.
  for(unsigned int i = 0; i < weights.size(); ++i) {
    A(i,i) = weights[i];
    b[i] = weights[i] * QDotInit[i];
  }
  CalcConstraintsJacobian (model, Q, cs, constraintJac);
  A.block (model.dof_count, 0, cs.size(), model.dof_count) = constraintJac;
  A.block (0, model.dof_count, model.dof_count, cs.size()) 
    = constraintJac.transpose();

  // Solve the sistem A*x = b.
  SolveLinearSystem (A, b, x, cs.linear_solver);

  // Copy the result to the output variable.
  QDot = x.block (0, 0, model.dof_count, 1);
}

RBDL_DLLAPI
void ForwardDynamicsConstraintsDirect (
  Model &model,
  const VectorNd &Q,
  const VectorNd &QDot,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext
  ) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  SolveConstrainedSystemDirect (CS.H, CS.G, Tau - CS.C, CS.gamma, QDDot
    , CS.force, CS.A, CS.b, CS.x, CS.linear_solver);

  // Copy back QDDot
  for (unsigned int i = 0; i < model.dof_count; i++)
    QDDot[i] = CS.x[i];

  // Copy back contact forces
  for (unsigned int i = 0; i < CS.size(); i++) {
    CS.force[i] = -CS.x[model.dof_count + i];
  }
}

RBDL_DLLAPI
void ForwardDynamicsConstraintsRangeSpaceSparse (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  Math::VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext) {

  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  SolveConstrainedSystemRangeSpaceSparse (model, CS.H, CS.G, Tau - CS.C
    , CS.gamma, QDDot, CS.force, CS.K, CS.a, CS.linear_solver);
}

RBDL_DLLAPI
void ForwardDynamicsConstraintsNullSpace (
  Model &model,
  const VectorNd &Q,
  const VectorNd &QDot,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext
  ) {

  LOG << "-------- " << __func__ << " --------" << std::endl;

  CalcConstrainedSystemVariables (model, Q, QDot, Tau, CS, f_ext);

  CS.GT_qr.compute (CS.G.transpose());
#ifdef RBDL_USE_SIMPLE_MATH
  CS.GT_qr_Q = CS.GT_qr.householderQ();
#else
  CS.GT_qr.householderQ().evalTo (CS.GT_qr_Q);
#endif

  CS.Y = CS.GT_qr_Q.block (0,0,QDot.rows(), CS.G.rows());
  CS.Z = CS.GT_qr_Q.block (0,CS.G.rows(),QDot.rows(), QDot.rows() - CS.G.rows());

  SolveConstrainedSystemNullSpace (CS.H, CS.G, Tau - CS.C, CS.gamma, QDDot
    , CS.force, CS.Y, CS.Z, CS.qddot_y, CS.qddot_z, CS.linear_solver);

}

RBDL_DLLAPI
void ComputeConstraintImpulsesDirect (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
  ) {

  // Compute H
  UpdateKinematicsCustom (model, &Q, NULL, NULL);
  CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

  // Compute G
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  SolveConstrainedSystemDirect (CS.H, CS.G, CS.H * QDotMinus, CS.v_plus
    , QDotPlus, CS.impulse, CS.A, CS.b, CS.x, CS.linear_solver);

  // Copy back QDotPlus
  for (unsigned int i = 0; i < model.dof_count; i++)
    QDotPlus[i] = CS.x[i];

  // Copy back constraint impulses 
  for (unsigned int i = 0; i < CS.size(); i++) {
    CS.impulse[i] = CS.x[model.dof_count + i];
  }

}

RBDL_DLLAPI
void ComputeConstraintImpulsesRangeSpaceSparse (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
  ) {

  // Compute H
  UpdateKinematicsCustom (model, &Q, NULL, NULL);
  CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

  // Compute G
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  SolveConstrainedSystemRangeSpaceSparse (model, CS.H, CS.G, CS.H * QDotMinus
    , CS.v_plus, QDotPlus, CS.impulse, CS.K, CS.a, CS.linear_solver);

}

RBDL_DLLAPI
void ComputeConstraintImpulsesNullSpace (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
  ) {

  // Compute H
  UpdateKinematicsCustom (model, &Q, NULL, NULL);
  CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

  // Compute G
  CalcConstraintsJacobian (model, Q, CS, CS.G, false);

  CS.GT_qr.compute(CS.G.transpose());
  CS.GT_qr_Q = CS.GT_qr.householderQ();

  CS.Y = CS.GT_qr_Q.block (0,0,QDotMinus.rows(), CS.G.rows());
  CS.Z = CS.GT_qr_Q.block (0,CS.G.rows(),QDotMinus.rows(), QDotMinus.rows()
    - CS.G.rows());

  SolveConstrainedSystemNullSpace (CS.H, CS.G, CS.H * QDotMinus, CS.v_plus
    , QDotPlus, CS.impulse, CS.Y, CS.Z, CS.qddot_y, CS.qddot_z
    , CS.linear_solver);
}

/** \brief Compute only the effects of external forces on the generalized accelerations
 *
 * This function is a reduced version of ForwardDynamics() which only
 * computes the effects of the external forces on the generalized
 * accelerations.
 *
 */
RBDL_DLLAPI
void ForwardDynamicsApplyConstraintForces (
    Model &model,
    const VectorNd &Tau,
    ConstraintSet &CS,
    VectorNd &QDDot
    ) {
  LOG << "-------- " << __func__ << " --------" << std::endl;
  assert (QDDot.size() == model.dof_count);

  unsigned int i = 0;

  for (i = 1; i < model.mBodies.size(); i++) {
    model.IA[i] = model.I[i].toMatrix();;
    model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

    if (CS.f_ext_constraints[i] != SpatialVector::Zero()) {
      LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint() * CS.f_ext_constraints[i] << std::endl;
      model.pA[i] -= model.X_base[i].toMatrixAdjoint() * CS.f_ext_constraints[i];
    }
  }

  // ClearLogOutput();

  LOG << "--- first loop ---" << std::endl;

  for (i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int q_index = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      unsigned int lambda = model.lambda[i];
      model.multdof3_u[i] = Vector3d (Tau[q_index], 
          Tau[q_index + 1], 
          Tau[q_index + 2]) 
        - model.multdof3_S[i].transpose() * model.pA[i];

      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i] - (model.multdof3_U[i] 
            * model.multdof3_Dinv[i] 
            * model.multdof3_U[i].transpose());

        SpatialVector pa = model.pA[i] + Ia * model.c[i] 
          + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i];

#ifdef EIGEN_CORE_H
        model.IA[lambda].noalias() += (model.X_lambda[i].toMatrixTranspose() 
            * Ia * model.X_lambda[i].toMatrix());
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
        model.IA[lambda] += (model.X_lambda[i].toMatrixTranspose() 
            * Ia * model.X_lambda[i].toMatrix());
        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() 
          << std::endl;
      }
    } else if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i] 
          - model.U[i] * (model.U[i] / model.d[i]).transpose();
        SpatialVector pa =  model.pA[i] + Ia * model.c[i] 
          + model.U[i] * model.u[i] / model.d[i];
#ifdef EIGEN_CORE_H
        model.IA[lambda].noalias() += (model.X_lambda[i].toMatrixTranspose() 
            * Ia * model.X_lambda[i].toMatrix());
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
        model.IA[lambda] += (model.X_lambda[i].toMatrixTranspose() 
            * Ia * model.X_lambda[i].toMatrix());
        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = " 
          << model.pA[lambda].transpose() << std::endl;
      }
    } else if(model.mJoints[i].mJointType == JointTypeCustom) {

      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      unsigned int lambda = model.lambda[i];
      VectorNd tau_temp = VectorNd::Zero(dofI);

      for(int z=0; z<dofI;++z){
        tau_temp[z] = Tau[q_index+z];
      }

      model.mCustomJoints[kI]->u = tau_temp
        - (model.mCustomJoints[kI]->S.transpose()
            * model.pA[i]);

      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
          - (   model.mCustomJoints[kI]->U
              * model.mCustomJoints[kI]->Dinv
              * model.mCustomJoints[kI]->U.transpose());

        SpatialVector pa = model.pA[i] + Ia * model.c[i]
          + (   model.mCustomJoints[kI]->U
              * model.mCustomJoints[kI]->Dinv
              * model.mCustomJoints[kI]->u);
#ifdef EIGEN_CORE_H
        model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
          * Ia * model.X_lambda[i].toMatrix();

        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
        model.IA[lambda] += model.X_lambda[i].toMatrixTranspose()
          * Ia * model.X_lambda[i].toMatrix();

        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose()
          << std::endl;
      }
    }
  }

  model.a[0] = SpatialVector (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

  for (i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];
    SpatialTransform X_lambda = model.X_lambda[i];

    model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
    LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = model.multdof3_Dinv[i] * 
        (model.multdof3_u[i] 
         - model.multdof3_U[i].transpose() * model.a[i]);

      QDDot[q_index] = qdd_temp[0];
      QDDot[q_index + 1] = qdd_temp[1];
      QDDot[q_index + 2] = qdd_temp[2];
      model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
      model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
    } else if (model.mJoints[i].mJointType == JointTypeCustom){
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      VectorNd qdd_temp = VectorNd::Zero(dofI);

      qdd_temp = model.mCustomJoints[kI]->Dinv 
        * (model.mCustomJoints[kI]->u
            - model.mCustomJoints[kI]->U.transpose()
            * model.a[i]);

      for(int z=0; z<dofI;++z){
        QDDot[q_index+z] = qdd_temp[z];
      }

      model.a[i] = model.a[i] + (model.mCustomJoints[kI]->S * qdd_temp);
    }
  }

  LOG << "QDDot = " << QDDot.transpose() << std::endl;
} 

/** \brief Computes the effect of external forces on the generalized accelerations.
 *
 * This function is essentially similar to ForwardDynamics() except that it
 * tries to only perform computations of variables that change due to
 * external forces defined in f_t.
 */
RBDL_DLLAPI
void ForwardDynamicsAccelerationDeltas (
    Model &model,
    ConstraintSet &CS,
    VectorNd &QDDot_t,
    const unsigned int body_id,
    const std::vector<SpatialVector> &f_t
    ) {
  LOG << "-------- " << __func__ << " ------" << std::endl;

  assert (CS.d_pA.size() == model.mBodies.size());
  assert (CS.d_a.size() == model.mBodies.size());
  assert (CS.d_u.size() == model.mBodies.size());

  // TODO reset all values (debug)
  for (unsigned int i = 0; i < model.mBodies.size(); i++) {
    CS.d_pA[i].setZero();
    CS.d_a[i].setZero();
    CS.d_u[i] = 0.;
    CS.d_multdof3_u[i].setZero();
  }
  for(unsigned int i=0; i<model.mCustomJoints.size();i++){
    model.mCustomJoints[i]->d_u.setZero();
  }

  for (unsigned int i = body_id; i > 0; i--) {
    if (i == body_id) {
      CS.d_pA[i] = -model.X_base[i].applyAdjoint(f_t[i]);
    }

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      CS.d_multdof3_u[i] = - model.multdof3_S[i].transpose() * (CS.d_pA[i]);

      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        CS.d_pA[lambda] =   CS.d_pA[lambda] 
          + model.X_lambda[i].applyTranspose (
              CS.d_pA[i] + (model.multdof3_U[i] 
                * model.multdof3_Dinv[i] 
                * CS.d_multdof3_u[i]));
      }
    } else if(model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      CS.d_u[i] = - model.S[i].dot(CS.d_pA[i]);
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        CS.d_pA[lambda] = CS.d_pA[lambda] 
          + model.X_lambda[i].applyTranspose (
              CS.d_pA[i] + model.U[i] * CS.d_u[i] / model.d[i]);
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom){

      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      //CS.
      model.mCustomJoints[kI]->d_u =
        - model.mCustomJoints[kI]->S.transpose() * (CS.d_pA[i]);
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        CS.d_pA[lambda] =
          CS.d_pA[lambda]
          + model.X_lambda[i].applyTranspose (
              CS.d_pA[i] + (   model.mCustomJoints[kI]->U
                * model.mCustomJoints[kI]->Dinv
                * model.mCustomJoints[kI]->d_u)
              );
      }
    }
  }

  for (unsigned int i = 0; i < f_t.size(); i++) {
    LOG << "f_t[" << i << "] = " << f_t[i].transpose() << std::endl;
  }

  for (unsigned int i = 0; i < model.mBodies.size(); i++) {
    LOG << "i = " << i << ": d_pA[i] " << CS.d_pA[i].transpose() << std::endl;
  }
  for (unsigned int i = 0; i < model.mBodies.size(); i++) {
    LOG << "i = " << i << ": d_u[i] = " << CS.d_u[i] << std::endl;
  }

  QDDot_t[0] = 0.;
  CS.d_a[0] = model.a[0];

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];

    SpatialVector Xa = model.X_lambda[i].apply(CS.d_a[lambda]);

    if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = model.multdof3_Dinv[i] 
        * (CS.d_multdof3_u[i] - model.multdof3_U[i].transpose() * Xa);

      QDDot_t[q_index] = qdd_temp[0];
      QDDot_t[q_index + 1] = qdd_temp[1];
      QDDot_t[q_index + 2] = qdd_temp[2];
      model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
      CS.d_a[i] = Xa + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom){

      QDDot_t[q_index] = (CS.d_u[i] - model.U[i].dot(Xa) ) / model.d[i];
      CS.d_a[i] = Xa + model.S[i] * QDDot_t[q_index];
    } else if (model.mJoints[i].mJointType == JointTypeCustom){
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      VectorNd qdd_temp = VectorNd::Zero(dofI);

      qdd_temp = model.mCustomJoints[kI]->Dinv
        * (model.mCustomJoints[kI]->d_u
            - model.mCustomJoints[kI]->U.transpose() * Xa);

      for(int z=0; z<dofI;++z){
        QDDot_t[q_index+z] = qdd_temp[z];
      }

      model.a[i] = model.a[i] + model.mCustomJoints[kI]->S * qdd_temp;
      CS.d_a[i] = Xa + model.mCustomJoints[kI]->S * qdd_temp;
    }

    LOG << "QDDot_t[" << i - 1 << "] = " << QDDot_t[i - 1] << std::endl;
    LOG << "d_a[i] = " << CS.d_a[i].transpose() << std::endl;
  }
}

inline void set_zero (std::vector<SpatialVector> &spatial_values) {
  for (unsigned int i = 0; i < spatial_values.size(); i++)
    spatial_values[i].setZero();
}

RBDL_DLLAPI
void ForwardDynamicsContactsKokkevis (
  Model &model,
  const VectorNd &Q,
  const VectorNd &QDot,
  const VectorNd &Tau,
  ConstraintSet &CS,
  VectorNd &QDDot
  ) {
  LOG << "-------- " << __func__ << " ------" << std::endl;

  assert (CS.f_ext_constraints.size() == model.mBodies.size());
  assert (CS.QDDot_0.size() == model.dof_count);
  assert (CS.QDDot_t.size() == model.dof_count);
  assert (CS.f_t.size() == CS.size());
  assert (CS.point_accel_0.size() == CS.size());
  assert (CS.K.rows() == CS.size());
  assert (CS.K.cols() == CS.size());
  assert (CS.force.size() == CS.size());
  assert (CS.a.size() == CS.size());

  Vector3d point_accel_t;

  unsigned int ci = 0;
  
  // The default acceleration only needs to be computed once
  {
    SUPPRESS_LOGGING;
    ForwardDynamics(model, Q, QDot, Tau, CS.QDDot_0);
  }

  LOG << "=== Initial Loop Start ===" << std::endl;
  // we have to compute the standard accelerations first as we use them to
  // compute the effects of each test force
  for(ci = 0; ci < CS.size(); ci++) {

    {
      SUPPRESS_LOGGING;
      UpdateKinematicsCustom(model, NULL, NULL, &CS.QDDot_0);
    }

    if(CS.constraintType[ci] == ConstraintSet::ContactConstraint)
    {
      LOG << "body_id = " << CS.body[ci] << std::endl;
      LOG << "point = " << CS.point[ci] << std::endl;
      LOG << "normal = " << CS.normal[ci] << std::endl;
      LOG << "QDDot_0 = " << CS.QDDot_0.transpose() << std::endl;
      {
        SUPPRESS_LOGGING;
        CS.point_accel_0[ci] = CalcPointAcceleration (model, Q, QDot
          , CS.QDDot_0, CS.body[ci], CS.point[ci], false);
        CS.a[ci] = - CS.acceleration[ci] 
          + CS.normal[ci].dot(CS.point_accel_0[ci]);
      }
      LOG << "point_accel_0 = " << CS.point_accel_0[ci].transpose();
    }
    else
    {
      std::cerr << "Forward Dynamic Contact Kokkevis: unsupported constraint \
        type." << std::endl;
      assert(false);
      abort();
    }   
  }

  // Now we can compute and apply the test forces and use their net effect
  // to compute the inverse articlated inertia to fill K.
  for (ci = 0; ci < CS.size(); ci++) {

    LOG << "=== Testforce Loop Start ===" << std::endl;

    unsigned int movable_body_id = 0;
    Vector3d point_global;

    switch (CS.constraintType[ci]) {

      case ConstraintSet::ContactConstraint:

        movable_body_id = GetMovableBodyId(model, CS.body[ci]);

        // assemble the test force
        LOG << "normal = " << CS.normal[ci].transpose() << std::endl;

        point_global = CalcBodyToBaseCoordinates(model, Q, CS.body[ci]
          , CS.point[ci], false);
        
        LOG << "point_global = " << point_global.transpose() << std::endl;

        CS.f_t[ci] = SpatialTransform(Matrix3d::Identity(), -point_global)
          .applyAdjoint(SpatialVector (0., 0., 0.
          , -CS.normal[ci][0], -CS.normal[ci][1], -CS.normal[ci][2]));
        CS.f_ext_constraints[movable_body_id] = CS.f_t[ci];

        LOG << "f_t[" << movable_body_id << "] = " << CS.f_t[ci].transpose() 
          << std::endl;

        {
          ForwardDynamicsAccelerationDeltas(model, CS, CS.QDDot_t
            , movable_body_id, CS.f_ext_constraints);

          LOG << "QDDot_0 = " << CS.QDDot_0.transpose() << std::endl;
          LOG << "QDDot_t = " << (CS.QDDot_t + CS.QDDot_0).transpose() 
            << std::endl;
          LOG << "QDDot_t - QDDot_0 = " << (CS.QDDot_t).transpose() << std::endl;
        }

        CS.f_ext_constraints[movable_body_id].setZero();

        CS.QDDot_t += CS.QDDot_0;

        // compute the resulting acceleration
        {
          SUPPRESS_LOGGING;
          UpdateKinematicsCustom(model, NULL, NULL, &CS.QDDot_t);
        }

        for(unsigned int cj = 0; cj < CS.size(); cj++) {
          {
            SUPPRESS_LOGGING;

            point_accel_t = CalcPointAcceleration(model, Q, QDot, CS.QDDot_t
              , CS.body[cj], CS.point[cj], false);
          }
      
          LOG << "point_accel_0  = " << CS.point_accel_0[ci].transpose() 
            << std::endl;
          LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;

          CS.K(ci,cj) = CS.normal[cj].dot(point_accel_t - CS.point_accel_0[cj]);
          
        }

      break;

      default:

        std::cerr << "Forward Dynamic Contact Kokkevis: unsupported constraint \
          type." << std::endl;
        assert(false);
        abort();

      break;

    }

  }

  LOG << "K = " << std::endl << CS.K << std::endl;
  LOG << "a = " << std::endl << CS.a << std::endl;

#ifndef RBDL_USE_SIMPLE_MATH
  switch (CS.linear_solver) {
    case (LinearSolverPartialPivLU) :
      CS.force = CS.K.partialPivLu().solve(CS.a);
      break;
    case (LinearSolverColPivHouseholderQR) :
      CS.force = CS.K.colPivHouseholderQr().solve(CS.a);
      break;
    case (LinearSolverHouseholderQR) :
      CS.force = CS.K.householderQr().solve(CS.a);
      break;
    default:
      LOG << "Error: Invalid linear solver: " << CS.linear_solver << std::endl;
      assert (0);
      break;
  }
#else
  bool solve_successful = LinSolveGaussElimPivot (CS.K, CS.a, CS.force);
  assert (solve_successful);
#endif

  LOG << "f = " << CS.force.transpose() << std::endl;

  for (ci = 0; ci < CS.size(); ci++) {
    unsigned int body_id = CS.body[ci];
    unsigned int movable_body_id = body_id;

    if (model.IsFixedBodyId(body_id)) {
      unsigned int fbody_id = body_id - model.fixed_body_discriminator;
      movable_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    }

    CS.f_ext_constraints[movable_body_id] -= CS.f_t[ci] * CS.force[ci]; 
    LOG << "f_ext[" << movable_body_id << "] = " << CS.f_ext_constraints[movable_body_id].transpose() << std::endl;
  }

  {
    SUPPRESS_LOGGING;
    ForwardDynamicsApplyConstraintForces (model, Tau, CS, QDDot);
  }

  LOG << "QDDot after applying f_ext: " << QDDot.transpose() << std::endl;
}

void SolveLinearSystem (
  const MatrixNd& A,
  const VectorNd& b, 
  VectorNd& x,
  LinearSolver ls
  ) {
  if(A.rows() != b.size() || A.cols() != x.size()) {
    std::cerr << "Mismatching sizes." << std::endl;
    assert(false);
    abort();
  }

  // Solve the sistem A*x = b.
  switch (ls) {
  case (LinearSolverPartialPivLU) :
      x = A.partialPivLu().solve(b);
    break;
  case (LinearSolverColPivHouseholderQR) :
    x = A.colPivHouseholderQr().solve(b);
    break;
  case (LinearSolverHouseholderQR) :
    x = A.householderQr().solve(b);
    break;
  default:
    std::cerr << "Error: Invalid linear solver: " << ls << std::endl;
    assert(false);
    abort();
    break;
  }
}

unsigned int GetMovableBodyId (Model& model, unsigned int id) {
  if(model.IsFixedBodyId(id)) {
    unsigned int fbody_id = id - model.fixed_body_discriminator;
    return model.mFixedBodies[fbody_id].mMovableParent;
  }
  else {
    return id;
  }
}

} /* namespace RigidBodyDynamics */
