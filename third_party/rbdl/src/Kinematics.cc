/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <cstring>
#include <assert.h>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

RBDL_DLLAPI void UpdateKinematics(
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    const VectorNd &QDDot) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  unsigned int i;

  model.a[0].setZero();

  for (i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];

    jcalc (model, i, Q, QDot);

    if (lambda != 0) {
      model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
      model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
    } else {
      model.X_base[i] = model.X_lambda[i];
      model.v[i] = model.v_J[i];
    }

    model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
    model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

    if(model.mJoints[i].mJointType != JointTypeCustom){
      if (model.mJoints[i].mDoFCount == 1) {
        model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
      } else if (model.mJoints[i].mDoFCount == 3) {
        Vector3d omegadot_temp (QDDot[q_index], 
            QDDot[q_index + 1], 
            QDDot[q_index + 2]);
        model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
      }
    } else {
      unsigned int custom_index = model.mJoints[i].custom_joint_index;
      const CustomJoint* custom_joint = model.mCustomJoints[custom_index];
      unsigned int joint_dof_count = custom_joint->mDoFCount;

      model.a[i] = model.a[i]
        + ( model.mCustomJoints[custom_index]->S 
            * QDDot.block(q_index, 0, joint_dof_count, 1));
    }
  }

  for (i = 1; i < model.mBodies.size(); i++) {
    LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
  }
}

RBDL_DLLAPI void UpdateKinematicsCustom(
    Model &model,
    const VectorNd *Q,
    const VectorNd *QDot,
    const VectorNd *QDDot) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  unsigned int i;

  if (Q) {
    for (i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];

      VectorNd QDot_zero (VectorNd::Zero (model.q_size));

      jcalc (model, i, (*Q), QDot_zero);

      if (lambda != 0) {
        model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
      } else {
        model.X_base[i] = model.X_lambda[i];
      }
    }
  }

  if (QDot) {
    for (i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];

      jcalc (model, i, *Q, *QDot);

      if (lambda != 0) {
        model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
        model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
      } else {
        model.v[i] = model.v_J[i];
        model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
      }
      // LOG << "v[" << i << "] = " << model.v[i].transpose() << std::endl;
    }
  }

  // FIXME?: Changing QDot can alter body accelerations via c[] - update to QDot but not QDDot can result in incorrect a[]
  if (QDDot) {
    for (i = 1; i < model.mBodies.size(); i++) {
      unsigned int q_index = model.mJoints[i].q_index;

      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];
      } else {
        model.a[i] = model.c[i];
      }

      if( model.mJoints[i].mJointType != JointTypeCustom){
        if (model.mJoints[i].mDoFCount == 1) {
          model.a[i] = model.a[i] + model.S[i] * (*QDDot)[q_index];
        } else if (model.mJoints[i].mDoFCount == 3) {
          Vector3d omegadot_temp ((*QDDot)[q_index], 
              (*QDDot)[q_index + 1], 
              (*QDDot)[q_index + 2]);
          model.a[i] = model.a[i] 
            + model.multdof3_S[i] * omegadot_temp;
        }
      } else {
        unsigned int k = model.mJoints[i].custom_joint_index;

        const CustomJoint* custom_joint = model.mCustomJoints[k];
        unsigned int joint_dof_count = custom_joint->mDoFCount;

        model.a[i] = model.a[i]
          + (  (model.mCustomJoints[k]->S)
              *(QDDot->block(q_index, 0, joint_dof_count, 1)));
      }
    } 
  }
}

RBDL_DLLAPI Vector3d CalcBodyToBaseCoordinates (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    const Vector3d &point_body_coordinates,
    bool update_kinematics) {
  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  if (body_id >= model.fixed_body_discriminator) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    unsigned int parent_id = model.mFixedBodies[fbody_id].mMovableParent;

    Matrix3d fixed_rotation = 
      model.mFixedBodies[fbody_id].mParentTransform.E.transpose();
    Vector3d fixed_position = model.mFixedBodies[fbody_id].mParentTransform.r;

    Matrix3d parent_body_rotation = model.X_base[parent_id].E.transpose();
    Vector3d parent_body_position = model.X_base[parent_id].r;

    return (parent_body_position 
        + (parent_body_rotation 
          * (fixed_position + fixed_rotation * (point_body_coordinates))) );
  }

  Matrix3d body_rotation = model.X_base[body_id].E.transpose();
  Vector3d body_position = model.X_base[body_id].r;

  return body_position + body_rotation * point_body_coordinates;
}

RBDL_DLLAPI Vector3d CalcBaseToBodyCoordinates (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    const Vector3d &point_base_coordinates,
    bool update_kinematics) {
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  if (body_id >= model.fixed_body_discriminator) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    unsigned int parent_id = model.mFixedBodies[fbody_id].mMovableParent;

    Matrix3d fixed_rotation = model.mFixedBodies[fbody_id].mParentTransform.E;
    Vector3d fixed_position = model.mFixedBodies[fbody_id].mParentTransform.r;

    Matrix3d parent_body_rotation = model.X_base[parent_id].E;
    Vector3d parent_body_position = model.X_base[parent_id].r;

    return (fixed_rotation 
        * ( - fixed_position 
          - parent_body_rotation 
          * (parent_body_position - point_base_coordinates)));
  }

  Matrix3d body_rotation = model.X_base[body_id].E;
  Vector3d body_position = model.X_base[body_id].r;

  return body_rotation * (point_base_coordinates - body_position);
}

RBDL_DLLAPI Matrix3d CalcBodyWorldOrientation(
    Model &model,
    const VectorNd &Q,
    const unsigned int body_id,
    bool update_kinematics) {
  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  if (body_id >= model.fixed_body_discriminator) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    model.mFixedBodies[fbody_id].mBaseTransform = 
      model.mFixedBodies[fbody_id].mParentTransform 
      * model.X_base[model.mFixedBodies[fbody_id].mMovableParent];

    return model.mFixedBodies[fbody_id].mBaseTransform.E;
  }

  return model.X_base[body_id].E;
}

RBDL_DLLAPI void CalcPointJacobian (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    const Vector3d &point_position,
    MatrixNd &G,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  SpatialTransform point_trans = 
    SpatialTransform (Matrix3d::Identity(), 
        CalcBodyToBaseCoordinates ( model, 
          Q, 
          body_id,
          point_position, 
          false));

  // std::cout << "RBDL point_trans " << point_trans.r.transpose() << std::endl;

  assert (G.rows() == 3 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
  }

  unsigned int j = reference_body_id;

  // e[j] is set to 1 if joint j contributes to the jacobian that we are
  // computing. For all other joints the column will be zero.
  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom){
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index, 3, 1) =
          point_trans.apply(
              model.X_base[j].inverse().apply(
                model.S[j])).block(3,0,3,1);
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0, q_index, 3, 3) =
          ((point_trans
            * model.X_base[j].inverse()).toMatrix()
           * model.multdof3_S[j]).block(3,0,3,3);
      }
    } else {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0, q_index, 3, model.mCustomJoints[k]->mDoFCount) =
        ((point_trans
          * model.X_base[j].inverse()).toMatrix()
         * model.mCustomJoints[k]->S).block( 
           3,0,3,model.mCustomJoints[k]->mDoFCount);
    }

    j = model.lambda[j];
  }
}

RBDL_DLLAPI void CalcPointJacobian6D (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    const Vector3d &point_position,
    MatrixNd &G,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  SpatialTransform point_trans =
    SpatialTransform (Matrix3d::Identity(),
        CalcBodyToBaseCoordinates (model,
          Q,
          body_id,
          point_position,
          false));

  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
  }

  unsigned int j = reference_body_id;

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom){
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index, 6, 1)
          = point_trans.apply(
              model.X_base[j].inverse().apply(
                model.S[j])).block(0,0,6,1);
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0, q_index, 6, 3)
          = ((point_trans
                * model.X_base[j].inverse()).toMatrix()
              * model.multdof3_S[j]).block(0,0,6,3);
      }
    } else {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0, q_index, 6, model.mCustomJoints[k]->mDoFCount)
        = ((point_trans
              * model.X_base[j].inverse()).toMatrix()
            * model.mCustomJoints[k]->S).block(
              0,0,6,model.mCustomJoints[k]->mDoFCount);
    }

    j = model.lambda[j];
  }
}

RBDL_DLLAPI void CalcBodySpatialJacobian (
    Model &model,
    const VectorNd &Q,
    unsigned int body_id,
    MatrixNd &G,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // update the Kinematics if necessary
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, NULL, NULL);
  }

  assert (G.rows() == 6 && G.cols() == model.qdot_size );

  unsigned int reference_body_id = body_id;

  SpatialTransform base_to_body;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id   = body_id
      - model.fixed_body_discriminator;

    reference_body_id       = model
      .mFixedBodies[fbody_id]
      .mMovableParent;

    base_to_body = model.mFixedBodies[fbody_id]
      .mParentTransform
      * model.X_base[reference_body_id];
  } else {
    base_to_body = model.X_base[reference_body_id];
  }

  unsigned int j = reference_body_id;

  while (j != 0) {
    unsigned int q_index = model.mJoints[j].q_index;

    if(model.mJoints[j].mJointType != JointTypeCustom){
      if (model.mJoints[j].mDoFCount == 1) {
        G.block(0,q_index,6,1) =
          base_to_body.apply(
              model.X_base[j]
              .inverse()
              .apply(model.S[j])
              );
      } else if (model.mJoints[j].mDoFCount == 3) {
        G.block(0,q_index,6,3) =
          (base_to_body * model.X_base[j].inverse()
          ).toMatrix() * model.multdof3_S[j];
      }
    }else if(model.mJoints[j].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[j].custom_joint_index;

      G.block(0,q_index,6,model.mCustomJoints[k]->mDoFCount ) =
        (base_to_body * model.X_base[j].inverse()
        ).toMatrix() * model.mCustomJoints[k]->S;
    }

    j = model.lambda[j];
  }
}

RBDL_DLLAPI Vector3d CalcPointVelocity (
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    unsigned int body_id,
    const Vector3d &point_position,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;
  assert (model.IsBodyId(body_id) || body_id == 0);
  assert (model.q_size == Q.size());
  assert (model.qdot_size == QDot.size());

  // Reset the velocity of the root body
  model.v[0].setZero();

  // update the Kinematics with zero acceleration
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, &QDot, NULL);
  }

  unsigned int reference_body_id = body_id;
  Vector3d reference_point = point_position;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    Vector3d base_coords = 
      CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false);
    reference_point =
      CalcBaseToBodyCoordinates(model, Q, reference_body_id, base_coords,false);
  }

  SpatialVector point_spatial_velocity = 
    SpatialTransform (
        CalcBodyWorldOrientation (model, Q, reference_body_id, false).transpose(), 
        reference_point).apply(model.v[reference_body_id]);

  return Vector3d (
      point_spatial_velocity[3],
      point_spatial_velocity[4],
      point_spatial_velocity[5]
      );
}

RBDL_DLLAPI Math::SpatialVector CalcPointVelocity6D(
    Model &model,
    const Math::VectorNd &Q,
    const Math::VectorNd &QDot,
    unsigned int body_id,
    const Math::Vector3d &point_position,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;
  assert (model.IsBodyId(body_id) || body_id == 0);
  assert (model.q_size == Q.size());
  assert (model.qdot_size == QDot.size());

  // Reset the velocity of the root body
  model.v[0].setZero();

  // update the Kinematics with zero acceleration
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &Q, &QDot, NULL);
  }

  unsigned int reference_body_id = body_id;
  Vector3d reference_point = point_position;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    Vector3d base_coords = 
      CalcBodyToBaseCoordinates(model, Q, body_id, point_position, false);
    reference_point = 
      CalcBaseToBodyCoordinates(model, Q, reference_body_id, base_coords,false);
  }

  return SpatialTransform (
      CalcBodyWorldOrientation (model, Q, reference_body_id, false).transpose(), 
      reference_point).apply(model.v[reference_body_id]);
}

RBDL_DLLAPI Vector3d CalcPointAcceleration (
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    const VectorNd &QDDot,
    unsigned int body_id,
    const Vector3d &point_position,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0].setZero();

  if (update_kinematics)
    UpdateKinematics (model, Q, QDot, QDDot);

  LOG << std::endl;

  unsigned int reference_body_id = body_id;
  Vector3d reference_point = point_position;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    Vector3d base_coords = 
      CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false);
    reference_point = 
      CalcBaseToBodyCoordinates (model, Q, reference_body_id,base_coords,false);
  }

  SpatialTransform p_X_i (
      CalcBodyWorldOrientation (model, Q, reference_body_id, false).transpose(),
      reference_point);

  SpatialVector p_v_i = p_X_i.apply(model.v[reference_body_id]);
  Vector3d a_dash = Vector3d (p_v_i[0], p_v_i[1], p_v_i[2]
      ).cross(Vector3d (p_v_i[3], p_v_i[4], p_v_i[5]));
  SpatialVector p_a_i = p_X_i.apply(model.a[reference_body_id]);

  return Vector3d (
      p_a_i[3] + a_dash[0],
      p_a_i[4] + a_dash[1],
      p_a_i[5] + a_dash[2]
      );
}

RBDL_DLLAPI SpatialVector CalcPointAcceleration6D(
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    const VectorNd &QDDot,
    unsigned int body_id,
    const Vector3d &point_position,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0].setZero();

  if (update_kinematics)
    UpdateKinematics (model, Q, QDot, QDDot);

  LOG << std::endl;

  unsigned int reference_body_id = body_id;
  Vector3d reference_point = point_position;

  if (model.IsFixedBodyId(body_id)) {
    unsigned int fbody_id = body_id - model.fixed_body_discriminator;
    reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    Vector3d base_coords = 
      CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false);
    reference_point = 
      CalcBaseToBodyCoordinates (model, Q, reference_body_id,base_coords,false);
  }

  SpatialTransform p_X_i (
      CalcBodyWorldOrientation (model, Q, reference_body_id, false).transpose(),
      reference_point);

  SpatialVector p_v_i = p_X_i.apply(model.v[reference_body_id]);
  Vector3d a_dash = Vector3d (p_v_i[0], p_v_i[1], p_v_i[2]
      ).cross(Vector3d (p_v_i[3], p_v_i[4], p_v_i[5]));
  return (p_X_i.apply(model.a[reference_body_id]) 
      + SpatialVector (0, 0, 0, a_dash[0], a_dash[1], a_dash[2]));
}

RBDL_DLLAPI bool InverseKinematics (
    Model &model,
    const VectorNd &Qinit,
    const std::vector<unsigned int>& body_id,
    const std::vector<Vector3d>& body_point,
    const std::vector<Vector3d>& target_pos,
    VectorNd &Qres,
    double step_tol,
    double lambda,
    unsigned int max_iter) {
  assert (Qinit.size() == model.q_size);
  assert (body_id.size() == body_point.size());
  assert (body_id.size() == target_pos.size());

  MatrixNd J = MatrixNd::Zero(3 * body_id.size(), model.qdot_size);
  VectorNd e = VectorNd::Zero(3 * body_id.size());

  Qres = Qinit;

  for (unsigned int ik_iter = 0; ik_iter < max_iter; ik_iter++) {
    UpdateKinematicsCustom (model, &Qres, NULL, NULL);

    for (unsigned int k = 0; k < body_id.size(); k++) {
      MatrixNd G (MatrixNd::Zero(3, model.qdot_size));
      CalcPointJacobian (model, Qres, body_id[k], body_point[k], G, false);
      Vector3d point_base = 
        CalcBodyToBaseCoordinates (model, Qres, body_id[k], body_point[k], false);
      LOG << "current_pos = " << point_base.transpose() << std::endl;

      for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int j = 0; j < model.qdot_size; j++) {
          unsigned int row = k * 3 + i;
          LOG << "i = " << i << " j = " << j << " k = " << k << " row = " 
            << row << " col = " << j << std::endl;
          J(row, j) = G (i,j);
        }

        e[k * 3 + i] = target_pos[k][i] - point_base[i];
      }
    }

    LOG << "J = " << J << std::endl;
    LOG << "e = " << e.transpose() << std::endl;

    // abort if we are getting "close"
    if (e.norm() < step_tol) {
      LOG << "Reached target close enough after " << ik_iter << " steps" << std::endl;
      return true;
    }

    MatrixNd JJTe_lambda2_I = 
      J * J.transpose() 
      + lambda*lambda * MatrixNd::Identity(e.size(), e.size());

    VectorNd z (body_id.size() * 3);
#ifndef RBDL_USE_SIMPLE_MATH
    z = JJTe_lambda2_I.colPivHouseholderQr().solve (e);
#else
    bool solve_successful = LinSolveGaussElimPivot (JJTe_lambda2_I, e, z);
    assert (solve_successful);
#endif

    LOG << "z = " << z << std::endl;

    VectorNd delta_theta = J.transpose() * z;
    LOG << "change = " << delta_theta << std::endl;

    Qres = Qres + delta_theta;
    LOG << "Qres = " << Qres.transpose() << std::endl;

    if (delta_theta.norm() < step_tol) {
      LOG << "reached convergence after " << ik_iter << " steps" << std::endl;
      return true;
    }

    VectorNd test_1 (z.size());
    VectorNd test_res (z.size());

    test_1.setZero();

    for (unsigned int i = 0; i < z.size(); i++) {
      test_1[i] = 1.;

      VectorNd test_delta = J.transpose() * test_1;

      test_res[i] = test_delta.squaredNorm();

      test_1[i] = 0.;
    }

    LOG << "test_res = " << test_res.transpose() << std::endl;
  }

  return false;
}

RBDL_DLLAPI
Vector3d CalcAngularVelocityfromMatrix (
    const Matrix3d &RotMat
    ) {
  double tol = 1e-12;

  Vector3d l = Vector3d (RotMat(2,1) - RotMat(1,2), RotMat(0,2) - RotMat(2,0), RotMat(1,0) - RotMat(0,1));
  if(l.norm() > tol){
    double preFactor = atan2(l.norm(),(RotMat.trace() - 1.0))/l.norm();
    return preFactor*l;
  }
  else if((RotMat(0,0)>0 && RotMat(1,1)>0 && RotMat(2,2) > 0) || l.norm() < tol){
    return Vector3dZero;
  }
  else{
    double PI = atan(1)*4.0;
    return Vector3d (PI/2*(RotMat(0,0) + 1.0),PI/2*(RotMat(1,1) + 1.0),PI/2*(RotMat(2,2) + 1.0));
  }
} 

RBDL_DLLAPI
InverseKinematicsConstraintSet::InverseKinematicsConstraintSet() {
  lambda = 1e-9;
  num_steps = 0;
  max_steps = 300;
  step_tol = 1e-12;
  constraint_tol = 1e-12;
  num_constraints = 0;
}

RBDL_DLLAPI
unsigned int InverseKinematicsConstraintSet::AddPointConstraint(
    unsigned int body_id,
    const Vector3d& body_point,
    const Vector3d& target_pos
    ) {
  constraint_type.push_back (ConstraintTypePosition);
  body_ids.push_back(body_id);
  body_points.push_back(body_point);
  target_positions.push_back(target_pos);
  target_orientations.push_back(Matrix3d::Zero(3,3));
  constraint_row_index.push_back(num_constraints);
  num_constraints = num_constraints + 3;
  return constraint_type.size() - 1;
}

RBDL_DLLAPI
unsigned int InverseKinematicsConstraintSet::AddOrientationConstraint(
    unsigned int body_id,
    const Matrix3d& target_orientation
    ) {
  constraint_type.push_back (ConstraintTypeOrientation);
  body_ids.push_back(body_id);
  body_points.push_back(Vector3d::Zero());
  target_positions.push_back(Vector3d::Zero());
  target_orientations.push_back(target_orientation);
  constraint_row_index.push_back(num_constraints);
  num_constraints = num_constraints + 3;
  return constraint_type.size() - 1;
}

RBDL_DLLAPI
unsigned int InverseKinematicsConstraintSet::AddFullConstraint(
    unsigned int body_id,
    const Vector3d& body_point,
    const Vector3d& target_pos,
    const Matrix3d& target_orientation
    ) {
  constraint_type.push_back (ConstraintTypeFull);
  body_ids.push_back(body_id);
  body_points.push_back(body_point);
  target_positions.push_back(target_pos);
  target_orientations.push_back(target_orientation);
  constraint_row_index.push_back(num_constraints);
  num_constraints = num_constraints + 6;
  return constraint_type.size() - 1;
}

RBDL_DLLAPI
unsigned int InverseKinematicsConstraintSet::ClearConstraints()
{
  for (unsigned int i =0; i < constraint_type.size(); i++){
    constraint_type.pop_back();
    body_ids.pop_back();
    body_points.pop_back();
    target_positions.pop_back();
    target_orientations.pop_back();
    num_constraints = 0;
  }
  return constraint_type.size();
}


RBDL_DLLAPI
bool InverseKinematics (
    Model &model,
    const Math::VectorNd &Qinit,
    InverseKinematicsConstraintSet &CS,
    Math::VectorNd &Qres
    ) {
  assert (Qinit.size() == model.q_size);
  assert (Qres.size() == Qinit.size());

  CS.J = MatrixNd::Zero(CS.num_constraints, model.qdot_size);
  CS.e = VectorNd::Zero(CS.num_constraints);

  Qres = Qinit;

  for (CS.num_steps = 0; CS.num_steps < CS.max_steps; CS.num_steps++) {
    UpdateKinematicsCustom (model, &Qres, NULL, NULL);

    for (unsigned int k = 0; k < CS.body_ids.size(); k++) {
      CS.G = MatrixNd::Zero(6, model.qdot_size);
      CalcPointJacobian6D (model, Qres, CS.body_ids[k], CS.body_points[k], CS.G, false);
      Vector3d point_base = CalcBodyToBaseCoordinates (model, Qres, CS.body_ids[k], CS.body_points[k], false);
      Matrix3d R = CalcBodyWorldOrientation(model, Qres, CS.body_ids[k], false);
      Vector3d angular_velocity = R.transpose()*CalcAngularVelocityfromMatrix(R*CS.target_orientations[k].transpose());

      //assign offsets and Jacobians
      if (CS.constraint_type[k] == InverseKinematicsConstraintSet::ConstraintTypeFull){
        for (unsigned int i = 0; i < 3; i++){
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row + 3] = CS.target_positions[k][i] - point_base[i];
          CS.e[row] = angular_velocity[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row + 3, j) = CS.G (i + 3,j);
            CS.J(row, j) = CS.G (i,j);
          }
        }
      }
      else if (CS.constraint_type[k] == InverseKinematicsConstraintSet::ConstraintTypeOrientation){
        for (unsigned int i = 0; i < 3; i++){
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = angular_velocity[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.G (i,j);
          }
        }
      }
      else if (CS.constraint_type[k] == InverseKinematicsConstraintSet::ConstraintTypePosition){
        for (unsigned int i = 0; i < 3; i++){
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.target_positions[k][i] - point_base[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.G (i + 3,j);
          }
        }
      }
      else {
        assert (false && !"Invalid inverse kinematics constraint");
      }
    }

    LOG << "J = " << CS.J << std::endl;
    LOG << "e = " << CS.e.transpose() << std::endl;
    CS.error_norm = CS.e.norm();

    // abort if we are getting "close"
    if (CS.error_norm < CS.step_tol) {
      LOG << "Reached target close enough after " << CS.num_steps << " steps" << std::endl;
      return true;
    }

    //     // "task space" from puppeteer
    //     MatrixNd Ek = MatrixNd::Zero (CS.e.size(), CS.e.size());
    // 
    //     for (size_t ei = 0; ei < CS.e.size(); ei ++) {
    // //      Ek(ei,ei) = CS.error_norm * CS.error_norm * 0.5 + CS.lambda;
    //       Ek(ei,ei) = CS.e[ei]*CS.e[ei] * 0.5 + CS.lambda;
    //     }
    // 
    //     MatrixNd JJT_Ek_wnI = CS.J * CS.J.transpose() + Ek;
    // 
    //     VectorNd delta_theta = CS.J.transpose() * JJT_Ek_wnI.colPivHouseholderQr().solve (CS.e);
    // 
    //     LOG << "change = " << delta_theta << std::endl;


    // "joint space" from puppeteer

    double Ek = 0.;

    for (size_t ei = 0; ei < CS.e.size(); ei ++) {
      Ek += CS.e[ei] * CS.e[ei] * 0.5;
    }

    VectorNd ek = CS.J.transpose() * CS.e;
    MatrixNd Wn = MatrixNd::Zero (Qres.size(), Qres.size());

    assert (ek.size() == Qres.size());

    for (size_t wi = 0; wi < Qres.size(); wi++) {
      Wn(wi, wi) = ek[wi] * ek[wi] * 0.5 + CS.lambda;
      //      Wn(wi, wi) = Ek + 1.0e-3;
    }

    MatrixNd A = CS.J.transpose() * CS.J + Wn;
    VectorNd delta_theta = A.colPivHouseholderQr().solve(CS.J.transpose() * CS.e);

    Qres = Qres + delta_theta;
    if (delta_theta.norm() < CS.step_tol) {
      LOG << "reached convergence after " << CS.num_steps << " steps" << std::endl;
      return true;
    }
  }

  return false;
}

}
