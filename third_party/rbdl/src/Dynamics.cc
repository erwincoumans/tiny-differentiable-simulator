/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <cassert>
#include <cstring>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

RBDL_DLLAPI void InverseDynamics (
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    const VectorNd &QDDot,
    VectorNd &Tau,
    std::vector<SpatialVector> *f_ext) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0].set (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];

    jcalc (model, i, Q, QDot);

    model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
    model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);

    if(model.mJoints[i].mJointType != JointTypeCustom){
      if (model.mJoints[i].mDoFCount == 1) {
        model.a[i] =  model.X_lambda[i].apply(model.a[lambda])
          + model.c[i]
          + model.S[i] * QDDot[q_index];
      } else if (model.mJoints[i].mDoFCount == 3) {
        model.a[i] =  model.X_lambda[i].apply(model.a[lambda])
          + model.c[i]
          + model.multdof3_S[i] * Vector3d (QDDot[q_index],
              QDDot[q_index + 1],
              QDDot[q_index + 2]);
      }
    }else if(model.mJoints[i].mJointType == JointTypeCustom){
      unsigned int k = model.mJoints[i].custom_joint_index;
      VectorNd customJointQDDot(model.mCustomJoints[k]->mDoFCount);
      for(unsigned z = 0; z < model.mCustomJoints[k]->mDoFCount; ++z){
        customJointQDDot[z] = QDDot[q_index+z];
      }
      model.a[i] =  model.X_lambda[i].apply(model.a[lambda])
        + model.c[i]
        + model.mCustomJoints[k]->S * customJointQDDot;
    }

    if (!model.mBodies[i].mIsVirtual) {
      model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
    } else {
      model.f[i].setZero();
    }
  }

  if (f_ext != NULL) {
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];
      model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
      model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
    }
  }

  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if(model.mJoints[i].mJointType != JointTypeCustom){
      if (model.mJoints[i].mDoFCount == 1) {
        Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
      } else if (model.mJoints[i].mDoFCount == 3) {
        Tau.block<3,1>(model.mJoints[i].q_index, 0)
          = model.multdof3_S[i].transpose() * model.f[i];
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {  
      unsigned int k = model.mJoints[i].custom_joint_index;
      Tau.block(model.mJoints[i].q_index,0,
          model.mCustomJoints[k]->mDoFCount, 1)
        = model.mCustomJoints[k]->S.transpose() * model.f[i];
    }

    if (model.lambda[i] != 0) {
      model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
    }
  }
}

RBDL_DLLAPI void NonlinearEffects ( 
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    VectorNd &Tau,
    std::vector<Math::SpatialVector> *f_ext) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  SpatialVector spatial_gravity (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0] = spatial_gravity;

  for (unsigned int i = 1; i < model.mJointUpdateOrder.size(); i++) {
    jcalc (model, model.mJointUpdateOrder[i], Q, QDot);
  }

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    if (model.lambda[i] == 0) {
      model.v[i] = model.v_J[i];
      model.a[i] = model.X_lambda[i].apply(spatial_gravity);
    }	else {
      model.v[i] = model.X_lambda[i].apply(model.v[model.lambda[i]]) + model.v_J[i];
      model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
      model.a[i] = model.X_lambda[i].apply(model.a[model.lambda[i]]) + model.c[i];
    }

    if (!model.mBodies[i].mIsVirtual) {
      model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
      if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero()) {
        model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
      }            
    } else {
      model.f[i].setZero();
    }
  }

  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if(model.mJoints[i].mJointType != JointTypeCustom){
      if (model.mJoints[i].mDoFCount == 1) {
        Tau[model.mJoints[i].q_index]
          = model.S[i].dot(model.f[i]);
      } else if (model.mJoints[i].mDoFCount == 3) {
        Tau.block<3,1>(model.mJoints[i].q_index, 0)
          = model.multdof3_S[i].transpose() * model.f[i];
      }
    } else if(model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int k = model.mJoints[i].custom_joint_index;
      Tau.block(model.mJoints[i].q_index,0,
          model.mCustomJoints[k]->mDoFCount, 1)
        = model.mCustomJoints[k]->S.transpose() * model.f[i];
    }

    if (model.lambda[i] != 0) {
      model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
    }
  }
}

RBDL_DLLAPI void CompositeRigidBodyAlgorithm (
    Model& model,
    const VectorNd &Q,
    MatrixNd &H,
    bool update_kinematics) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    if (update_kinematics) {
      jcalc_X_lambda_S (model, i, Q);
    }
    model.Ic[i] = model.I[i];
  }

  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    if (model.lambda[i] != 0) {
      model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.Ic[i]);
    }

    unsigned int dof_index_i = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 1 
        && model.mJoints[i].mJointType != JointTypeCustom) {

      SpatialVector F             = model.Ic[i] * model.S[i];
      H(dof_index_i, dof_index_i) = model.S[i].dot(F);

      unsigned int j = i;
      unsigned int dof_index_j = dof_index_i;

      while (model.lambda[j] != 0) {
        F = model.X_lambda[j].applyTranspose(F);
        j = model.lambda[j];
        dof_index_j = model.mJoints[j].q_index;

        if(model.mJoints[j].mJointType != JointTypeCustom) {
          if (model.mJoints[j].mDoFCount == 1) {
            H(dof_index_i,dof_index_j) = F.dot(model.S[j]);
            H(dof_index_j,dof_index_i) = H(dof_index_i,dof_index_j);
          } else if (model.mJoints[j].mDoFCount == 3) {
            Vector3d H_temp2 = 
              (F.transpose() * model.multdof3_S[j]).transpose();
            LOG << F.transpose() << std::endl 
              << model.multdof3_S[j] << std::endl;
            LOG << H_temp2.transpose() << std::endl;

            H.block<1,3>(dof_index_i,dof_index_j) = H_temp2.transpose();
            H.block<3,1>(dof_index_j,dof_index_i) = H_temp2;
          }
        } else if (model.mJoints[j].mJointType == JointTypeCustom){        
          unsigned int k      = model.mJoints[j].custom_joint_index;
          unsigned int dof    = model.mCustomJoints[k]->mDoFCount;
          VectorNd H_temp2    =
            (F.transpose() * model.mCustomJoints[k]->S).transpose();

          LOG << F.transpose()
            << std::endl
            << model.mCustomJoints[j]->S << std::endl;

          LOG << H_temp2.transpose() << std::endl;

          H.block(dof_index_i,dof_index_j,1,dof) = H_temp2.transpose();
          H.block(dof_index_j,dof_index_i,dof,1) = H_temp2;
        }
      }
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Matrix63 F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
      H.block<3,3>(dof_index_i, dof_index_i) = model.multdof3_S[i].transpose() * F_63;

      unsigned int j = i;
      unsigned int dof_index_j = dof_index_i;

      while (model.lambda[j] != 0) {
        F_63 = model.X_lambda[j].toMatrixTranspose() * (F_63);
        j = model.lambda[j];
        dof_index_j = model.mJoints[j].q_index;

        if(model.mJoints[j].mJointType != JointTypeCustom){
          if (model.mJoints[j].mDoFCount == 1) {
            Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

            H.block<3,1>(dof_index_i,dof_index_j) = H_temp2;
            H.block<1,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
          } else if (model.mJoints[j].mDoFCount == 3) {
            Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

            H.block<3,3>(dof_index_i,dof_index_j) = H_temp2;
            H.block<3,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
          }
        } else if (model.mJoints[j].mJointType == JointTypeCustom){
          unsigned int k = model.mJoints[j].custom_joint_index;
          unsigned int dof = model.mCustomJoints[k]->mDoFCount;

          MatrixNd H_temp2 = F_63.transpose() * (model.mCustomJoints[k]->S);

          H.block(dof_index_i,dof_index_j,3,dof) = H_temp2;
          H.block(dof_index_j,dof_index_i,dof,3) = H_temp2.transpose();
        }
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {      
      unsigned int kI = model.mJoints[i].custom_joint_index;
      unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;

      MatrixNd F_Nd = model.Ic[i].toMatrix()
        * model.mCustomJoints[kI]->S;

      H.block(dof_index_i, dof_index_i,dofI,dofI)
        = model.mCustomJoints[kI]->S.transpose() * F_Nd;

      unsigned int j = i;
      unsigned int dof_index_j = dof_index_i;

      while (model.lambda[j] != 0) {
        F_Nd = model.X_lambda[j].toMatrixTranspose() * (F_Nd);
        j = model.lambda[j];
        dof_index_j = model.mJoints[j].q_index;

        if(model.mJoints[j].mJointType != JointTypeCustom){
          if (model.mJoints[j].mDoFCount == 1) {
            MatrixNd H_temp2 = F_Nd.transpose() * (model.S[j]);
            H.block(   dof_index_i,  dof_index_j,
                H_temp2.rows(),H_temp2.cols()) = H_temp2;
            H.block(dof_index_j,dof_index_i,
                H_temp2.cols(),H_temp2.rows()) = H_temp2.transpose();
          } else if (model.mJoints[j].mDoFCount == 3) {
            MatrixNd H_temp2 = F_Nd.transpose() * (model.multdof3_S[j]);
            H.block(dof_index_i,   dof_index_j,
                H_temp2.rows(),H_temp2.cols()) = H_temp2;
            H.block(dof_index_j,   dof_index_i,
                H_temp2.cols(),H_temp2.rows()) = H_temp2.transpose();
          }
        } else if (model.mJoints[j].mJointType == JointTypeCustom){
          unsigned int k   = model.mJoints[j].custom_joint_index;
          unsigned int dof = model.mCustomJoints[k]->mDoFCount;

          MatrixNd H_temp2 = F_Nd.transpose() * (model.mCustomJoints[k]->S);

          H.block(dof_index_i,dof_index_j,3,dof) = H_temp2;
          H.block(dof_index_j,dof_index_i,dof,3) = H_temp2.transpose();
        }
      }
    }
  }
}

RBDL_DLLAPI void ForwardDynamics (
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    const VectorNd &Tau,
    VectorNd &QDDot,
    std::vector<SpatialVector> *f_ext) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

  unsigned int i = 0;

  LOG << "Q          = " << Q.transpose() << std::endl;
  LOG << "QDot       = " << QDot.transpose() << std::endl;
  LOG << "Tau        = " << Tau.transpose() << std::endl;
  LOG << "---" << std::endl;

  // Reset the velocity of the root body
  model.v[0].setZero();

  for (i = 1; i < model.mBodies.size(); i++) {
    unsigned int lambda = model.lambda[i];

    jcalc (model, i, Q, QDot);

    if (lambda != 0)
      model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
    else
      model.X_base[i] = model.X_lambda[i];

    model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];

    /*
       LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
       LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
       LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
       LOG << "X_base (" << i << "):" << std::endl << model.X_base[i] << std::endl;
       LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda[i] << std::endl;
       LOG << "SpatialVelocity (" << i << "): " << model.v[i] << std::endl;
       */
    model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
    model.I[i].setSpatialMatrix (model.IA[i]);

    model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

    if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero()) {
      LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint() * (*f_ext)[i] << std::endl;
      model.pA[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
    }
  }

  // ClearLogOutput();

  LOG << "--- first loop ---" << std::endl;

  for (i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int q_index = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {

      model.U[i] = model.IA[i] * model.S[i];
      model.d[i] = model.S[i].dot(model.U[i]);
      model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
      //      LOG << "u[" << i << "] = " << model.u[i] << std::endl;

      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia =    model.IA[i]
          - model.U[i]
          * (model.U[i] / model.d[i]).transpose();

        SpatialVector pa =  model.pA[i]
          + Ia * model.c[i]
          + model.U[i] * model.u[i] / model.d[i];

#ifdef EIGEN_CORE_H
        model.IA[lambda].noalias()
          += model.X_lambda[i].toMatrixTranspose()
          * Ia * model.X_lambda[i].toMatrix();
        model.pA[lambda].noalias()
          += model.X_lambda[i].applyTranspose(pa);
#else
        model.IA[lambda]
          += model.X_lambda[i].toMatrixTranspose()
          * Ia * model.X_lambda[i].toMatrix();

        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = "
          << model.pA[lambda].transpose() << std::endl;
      }
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
#ifdef EIGEN_CORE_H
      model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose()
          * model.multdof3_U[i]).inverse().eval();
#else
      model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose()
          * model.multdof3_U[i]).inverse();
#endif
      Vector3d tau_temp(Tau.block(q_index,0,3,1));
      model.multdof3_u[i] = tau_temp 
        - model.multdof3_S[i].transpose() * model.pA[i];

      // LOG << "multdof3_u[" << i << "] = " 
      //                      << model.multdof3_u[i].transpose() << std::endl;
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
          - model.multdof3_U[i]
          * model.multdof3_Dinv[i]
          * model.multdof3_U[i].transpose();
        SpatialVector pa = model.pA[i]
          + Ia
          * model.c[i]
          + model.multdof3_U[i]
          * model.multdof3_Dinv[i]
          * model.multdof3_u[i];
#ifdef EIGEN_CORE_H
        model.IA[lambda].noalias()
          += model.X_lambda[i].toMatrixTranspose()
          * Ia
          * model.X_lambda[i].toMatrix();

        model.pA[lambda].noalias()
          += model.X_lambda[i].applyTranspose(pa);
#else
        model.IA[lambda]
          += model.X_lambda[i].toMatrixTranspose()
          * Ia
          * model.X_lambda[i].toMatrix();

        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = "
          << model.pA[lambda].transpose()
          << std::endl;
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI   = model.mJoints[i].custom_joint_index;
      unsigned int dofI = model.mCustomJoints[kI]->mDoFCount;
      model.mCustomJoints[kI]->U =
        model.IA[i] * model.mCustomJoints[kI]->S;

#ifdef EIGEN_CORE_H
      model.mCustomJoints[kI]->Dinv
        = (model.mCustomJoints[kI]->S.transpose()
            * model.mCustomJoints[kI]->U).inverse().eval();
#else
      model.mCustomJoints[kI]->Dinv
        = (model.mCustomJoints[kI]->S.transpose()
            * model.mCustomJoints[kI]->U).inverse();
#endif
      VectorNd tau_temp(Tau.block(q_index,0,dofI,1));
      model.mCustomJoints[kI]->u = tau_temp
        - model.mCustomJoints[kI]->S.transpose() * model.pA[i];

      //      LOG << "multdof3_u[" << i << "] = " 
      //      << model.multdof3_u[i].transpose() << std::endl;
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialMatrix Ia = model.IA[i]
          - (model.mCustomJoints[kI]->U
              * model.mCustomJoints[kI]->Dinv
              * model.mCustomJoints[kI]->U.transpose());
        SpatialVector pa =  model.pA[i] 
          + Ia * model.c[i]
          + (model.mCustomJoints[kI]->U
              * model.mCustomJoints[kI]->Dinv
              * model.mCustomJoints[kI]->u);

#ifdef EIGEN_CORE_H
        model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
          * Ia
          * model.X_lambda[i].toMatrix();
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
        model.IA[lambda] += model.X_lambda[i].toMatrixTranspose()
          * Ia
          * model.X_lambda[i].toMatrix();
        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = "
          << model.pA[lambda].transpose()
          << std::endl;
      }
    }
  }

  //  ClearLogOutput();

  model.a[0] = spatial_gravity * -1.;

  for (i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];
    SpatialTransform X_lambda = model.X_lambda[i];

    model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
    LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
      model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
      QDDot[q_index] = qdd_temp[0];
      QDDot[q_index + 1] = qdd_temp[1];
      QDDot[q_index + 2] = qdd_temp[2];
      model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI = model.mJoints[i].custom_joint_index;
      unsigned int dofI=model.mCustomJoints[kI]->mDoFCount;

      VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv
        * (  model.mCustomJoints[kI]->u
            - model.mCustomJoints[kI]->U.transpose()
            * model.a[i]);

      for(int z=0; z<dofI; ++z){
        QDDot[q_index+z] = qdd_temp[z];
      }

      model.a[i] = model.a[i]
        + model.mCustomJoints[kI]->S * qdd_temp;
    } 
  }

  LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

RBDL_DLLAPI void ForwardDynamicsLagrangian (
    Model &model,
    const VectorNd &Q,
    const VectorNd &QDot,
    const VectorNd &Tau,
    VectorNd &QDDot,
    Math::LinearSolver linear_solver,
    std::vector<SpatialVector> *f_ext,
    Math::MatrixNd *H,
    Math::VectorNd *C) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  bool free_H = false;
  bool free_C = false;

  if (H == NULL) {
    H = new MatrixNd (MatrixNd::Zero(model.dof_count, model.dof_count));
    free_H = true;
  }

  if (C == NULL) {
    C = new VectorNd (VectorNd::Zero(model.dof_count));
    free_C = true;
  }

  // we set QDDot to zero to compute C properly with the InverseDynamics
  // method.
  QDDot.setZero();

  InverseDynamics (model, Q, QDot, QDDot, (*C), f_ext);
  CompositeRigidBodyAlgorithm (model, Q, *H, false);

  LOG << "A = " << std::endl << *H << std::endl;
  LOG << "b = " << std::endl << *C * -1. + Tau << std::endl;

  switch (linear_solver) {
    case (LinearSolverPartialPivLU) :
      QDDot = H->partialPivLu().solve (*C * -1. + Tau);
      break;
    case (LinearSolverColPivHouseholderQR) :
      QDDot = H->colPivHouseholderQr().solve (*C * -1. + Tau);
      break;
    case (LinearSolverHouseholderQR) :
      QDDot = H->householderQr().solve (*C * -1. + Tau);
      break;
    case (LinearSolverLLT) :
      QDDot = H->llt().solve (*C * -1. + Tau);
      break;
    default:
      LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
      assert (0);
      break;
  }

  if (free_C) {
    delete C;
  }

  if (free_H) {
    delete H;
  }

  LOG << "x = " << QDDot << std::endl;
}

RBDL_DLLAPI void CalcMInvTimesTau ( Model &model,
    const VectorNd &Q,
    const VectorNd &Tau,
    VectorNd &QDDot,
    bool update_kinematics) {

  LOG << "Q          = " << Q.transpose() << std::endl;
  LOG << "---" << std::endl;

  // Reset the velocity of the root body
  model.v[0].setZero();
  model.a[0].setZero();

  if (update_kinematics) {
    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      jcalc_X_lambda_S (model, model.mJointUpdateOrder[i], Q);

      model.v_J[i].setZero();
      model.v[i].setZero();
      model.c[i].setZero();
      model.pA[i].setZero();
      model.I[i].setSpatialMatrix (model.IA[i]);
    }
  }

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    model.pA[i].setZero();
  }

  // ClearLogOutput();

  if (update_kinematics) {
    // Compute Articulate Body Inertias
    for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
      unsigned int q_index = model.mJoints[i].q_index;

      if (model.mJoints[i].mDoFCount == 1
          && model.mJoints[i].mJointType != JointTypeCustom) {
        model.U[i] = model.IA[i] * model.S[i];
        model.d[i] = model.S[i].dot(model.U[i]);
        //      LOG << "u[" << i << "] = " << model.u[i] << std::endl;
        unsigned int lambda = model.lambda[i];

        if (lambda != 0) {
          SpatialMatrix Ia = model.IA[i] - 
            model.U[i] * (model.U[i] / model.d[i]).transpose();
#ifdef EIGEN_CORE_H
          model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
            * Ia
            * model.X_lambda[i].toMatrix();
#else
          model.IA[lambda] += model.X_lambda[i].toMatrixTranspose()
            * Ia
            * model.X_lambda[i].toMatrix();
#endif
        }
      } else if (model.mJoints[i].mDoFCount == 3
          && model.mJoints[i].mJointType != JointTypeCustom) {

        model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];

#ifdef EIGEN_CORE_H
        model.multdof3_Dinv[i] = 
          (model.multdof3_S[i].transpose()*model.multdof3_U[i]).inverse().eval();
#else
        model.multdof3_Dinv[i] = 
          (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse();
#endif
        //      LOG << "mCustomJoints[kI]->u[" << i << "] = "
        //<< model.mCustomJoints[kI]->u[i].transpose() << std::endl;

        unsigned int lambda = model.lambda[i];

        if (lambda != 0) {
          SpatialMatrix Ia = model.IA[i]
            - ( model.multdof3_U[i]
                * model.multdof3_Dinv[i]
                * model.multdof3_U[i].transpose());
#ifdef EIGEN_CORE_H
          model.IA[lambda].noalias() +=
            model.X_lambda[i].toMatrixTranspose()
            * Ia
            * model.X_lambda[i].toMatrix();
#else
          model.IA[lambda] +=
            model.X_lambda[i].toMatrixTranspose()
            * Ia * model.X_lambda[i].toMatrix();
#endif
        }
      } else if (model.mJoints[i].mJointType == JointTypeCustom) {
        unsigned int kI     = model.mJoints[i].custom_joint_index;
        unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
        model.mCustomJoints[kI]->U = model.IA[i] * model.mCustomJoints[kI]->S;

#ifdef EIGEN_CORE_H
        model.mCustomJoints[kI]->Dinv = (model.mCustomJoints[kI]->S.transpose()
            * model.mCustomJoints[kI]->U
            ).inverse().eval();
#else
        model.mCustomJoints[kI]->Dinv=(model.mCustomJoints[kI]->S.transpose()
            * model.mCustomJoints[kI]->U
            ).inverse();
#endif
        //      LOG << "mCustomJoints[kI]->u[" << i << "] = "
        //<< model.mCustomJoints[kI]->u.transpose() << std::endl;
        unsigned int lambda = model.lambda[i];

        if (lambda != 0) {
          SpatialMatrix Ia = model.IA[i] 
            - ( model.mCustomJoints[kI]->U
                * model.mCustomJoints[kI]->Dinv
                * model.mCustomJoints[kI]->U.transpose());
#ifdef EIGEN_CORE_H
          model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose()
            * Ia
            * model.X_lambda[i].toMatrix();
#else
          model.IA[lambda] += model.X_lambda[i].toMatrixTranspose()
            * Ia * model.X_lambda[i].toMatrix();
#endif
        }
      }
    }
  }

  // compute articulated bias forces
  for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int q_index = model.mJoints[i].q_index;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {

      model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
      // LOG << "u[" << i << "] = " << model.u[i] << std::endl;
      unsigned int lambda = model.lambda[i];
      if (lambda != 0) {
        SpatialVector pa = model.pA[i] + model.U[i] * model.u[i] / model.d[i];

#ifdef EIGEN_CORE_H
        model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = "
          << model.pA[lambda].transpose() << std::endl;
      }
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {

      Vector3d tau_temp ( Tau[q_index],
          Tau[q_index + 1],
          Tau[q_index + 2]);
      model.multdof3_u[i] = tau_temp 
        - model.multdof3_S[i].transpose()*model.pA[i];
      //      LOG << "multdof3_u[" << i << "] = "
      // << model.multdof3_u[i].transpose() << std::endl;
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        SpatialVector pa = model.pA[i]
          + model.multdof3_U[i]
          * model.multdof3_Dinv[i]
          * model.multdof3_u[i];

#ifdef EIGEN_CORE_H
        model.pA[lambda].noalias() +=
          model.X_lambda[i].applyTranspose(pa);
#else
        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = "
          << model.pA[lambda].transpose() << std::endl;
      }
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;
      VectorNd tau_temp(Tau.block(q_index,0,dofI,1));

      model.mCustomJoints[kI]->u = 
        tau_temp - ( model.mCustomJoints[kI]->S.transpose()* model.pA[i]);
      //      LOG << "mCustomJoints[kI]->u"
      // << model.mCustomJoints[kI]->u.transpose() << std::endl;
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        SpatialVector pa = model.pA[i]
          + (   model.mCustomJoints[kI]->U
              * model.mCustomJoints[kI]->Dinv
              * model.mCustomJoints[kI]->u);

#ifdef EIGEN_CORE_H
        model.pA[lambda].noalias() +=
          model.X_lambda[i].applyTranspose(pa);
#else
        model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
        LOG << "pA[" << lambda << "] = "
          << model.pA[lambda].transpose() << std::endl;
      }
    }
  }

  //  ClearLogOutput();

  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    unsigned int q_index = model.mJoints[i].q_index;
    unsigned int lambda = model.lambda[i];
    SpatialTransform X_lambda = model.X_lambda[i];

    model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
    LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

    if (model.mJoints[i].mDoFCount == 1
        && model.mJoints[i].mJointType != JointTypeCustom) {
      QDDot[q_index] = (1./model.d[i])*(model.u[i]-model.U[i].dot(model.a[i]));
      model.a[i]     = model.a[i] + model.S[i] * QDDot[q_index];
    } else if (model.mJoints[i].mDoFCount == 3
        && model.mJoints[i].mJointType != JointTypeCustom) {
      Vector3d qdd_temp = 
        model.multdof3_Dinv[i] * (model.multdof3_u[i]
            - model.multdof3_U[i].transpose()*model.a[i]);

      QDDot[q_index]      = qdd_temp[0];
      QDDot[q_index + 1]  = qdd_temp[1];
      QDDot[q_index + 2]  = qdd_temp[2];
      model.a[i]          = model.a[i] + model.multdof3_S[i] * qdd_temp;
    } else if (model.mJoints[i].mJointType == JointTypeCustom) {
      unsigned int kI     = model.mJoints[i].custom_joint_index;
      unsigned int dofI   = model.mCustomJoints[kI]->mDoFCount;

      VectorNd qdd_temp = model.mCustomJoints[kI]->Dinv
        * (  model.mCustomJoints[kI]->u 
            - model.mCustomJoints[kI]->U.transpose() * model.a[i]);

      for(unsigned z = 0; z < dofI; ++z){
        QDDot[q_index+z]      = qdd_temp[z];
      }

      model.a[i] =    model.a[i]
        + model.mCustomJoints[kI]->S * qdd_temp;
    }
  }

  LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

} /* namespace RigidBodyDynamics */
