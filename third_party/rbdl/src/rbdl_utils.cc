/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "rbdl/rbdl_utils.h"

#include "rbdl/rbdl_math.h"
#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

#include <sstream>
#include <iomanip>

namespace RigidBodyDynamics {

namespace Utils {

using namespace std;
using namespace Math;

string get_dof_name (const SpatialVector &joint_dof) {
  if (joint_dof == SpatialVector (1., 0., 0., 0., 0., 0.)) 
    return "RX";
  else if (joint_dof == SpatialVector (0., 1., 0., 0., 0., 0.))
    return "RY";
  else if (joint_dof == SpatialVector (0., 0., 1., 0., 0., 0.))
    return "RZ";
  else if (joint_dof == SpatialVector (0., 0., 0., 1., 0., 0.))
    return "TX";
  else if (joint_dof == SpatialVector (0., 0., 0., 0., 1., 0.))
    return "TY";
  else if (joint_dof == SpatialVector (0., 0., 0., 0., 0., 1.))
    return "TZ";

  ostringstream dof_stream(ostringstream::out);
  dof_stream << "custom_axis (" << joint_dof.transpose() << ")";
  return dof_stream.str();
}

string get_body_name (const RigidBodyDynamics::Model &model, unsigned int body_id) {
  if (model.mBodies[body_id].mIsVirtual) {
    // if there is not a unique child we do not know what to do...
    if (model.mu[body_id].size() != 1)
      return "";

    return get_body_name (model, model.mu[body_id][0]);
  }

  return model.GetBodyName(body_id);
}

RBDL_DLLAPI std::string GetModelDOFOverview (const Model &model) {
  stringstream result ("");

  unsigned int q_index = 0;
  for (unsigned int i = 1; i < model.mBodies.size(); i++) {
    if (model.mJoints[i].mDoFCount == 1) {
      result << setfill(' ') << setw(3) << q_index << ": " << get_body_name(model, i) << "_" << get_dof_name (model.S[i]) << endl;
      q_index++;
    } else {
      for (unsigned int j = 0; j < model.mJoints[i].mDoFCount; j++) {
        result << setfill(' ') << setw(3) << q_index << ": " << get_body_name(model, i) << "_" << get_dof_name (model.mJoints[i].mJointAxes[j]) << endl;
        q_index++;
      }
    }
  }

  return result.str();
}

std::string print_hierarchy (const RigidBodyDynamics::Model &model, unsigned int body_index = 0, int indent = 0) {
  stringstream result ("");

  for (int j = 0; j < indent; j++)
    result << "  ";

  result << get_body_name (model, body_index);

  if (body_index > 0)
    result << " [ ";

  while (model.mBodies[body_index].mIsVirtual) {
    if (model.mu[body_index].size() == 0) {
      result << " end";
      break;
    } else if (model.mu[body_index].size() > 1) {
      cerr << endl << "Error: Cannot determine multi-dof joint as massless body with id " << body_index << " (name: " << model.GetBodyName(body_index) << ") has more than one child:" << endl;
      for (unsigned int ci = 0; ci < model.mu[body_index].size(); ci++) {
        cerr << "  id: " << model.mu[body_index][ci] << " name: " << model.GetBodyName(model.mu[body_index][ci]) << endl;
      }
      abort();
    }

    result << get_dof_name(model.S[body_index]) << ", ";

    body_index = model.mu[body_index][0];
  }

  if (body_index > 0)
    result << get_dof_name(model.S[body_index]) << " ]";
  result << endl;

  unsigned int child_index = 0;
  for (child_index = 0; child_index < model.mu[body_index].size(); child_index++) {
    result << print_hierarchy (model, model.mu[body_index][child_index], indent + 1);
  }

  // print fixed children
  for (unsigned int fbody_index = 0; fbody_index < model.mFixedBodies.size(); fbody_index++) {
    if (model.mFixedBodies[fbody_index].mMovableParent == body_index) {
      for (int j = 0; j < indent + 1; j++)
        result << "  ";

      result << model.GetBodyName(model.fixed_body_discriminator + fbody_index) << " [fixed]" << endl;
    }
  }


  return result.str();
}

RBDL_DLLAPI std::string GetModelHierarchy (const Model &model) {
  stringstream result ("");

  result << print_hierarchy (model);

  return result.str();
}

RBDL_DLLAPI std::string GetNamedBodyOriginsOverview (Model &model) {
  stringstream result ("");

  VectorNd Q (VectorNd::Zero(model.dof_count));
  UpdateKinematicsCustom (model, &Q, NULL, NULL);

  for (unsigned int body_id = 0; body_id < model.mBodies.size(); body_id++) {
    std::string body_name = model.GetBodyName (body_id);

    if (body_name.size() == 0) 
      continue;

    Vector3d position = CalcBodyToBaseCoordinates (model, Q, body_id, Vector3d (0., 0., 0.), false);

    result << body_name << ": " << position.transpose() << endl;
  }

  return result.str();
}

RBDL_DLLAPI void CalcCenterOfMass (
  Model &model,
  const Math::VectorNd &q,
  const Math::VectorNd &qdot,
  const Math::VectorNd *qddot,
  double &mass,
  Math::Vector3d &com,
  Math::Vector3d *com_velocity,
  Math::Vector3d *com_acceleration, 
  Math::Vector3d *angular_momentum,
  Math::Vector3d *change_of_angular_momentum,
  bool update_kinematics) {
  // If we want to compute com_acceleration or change of angular momentum
  // we must have qddot provided.
  assert( (com_acceleration == NULL && change_of_angular_momentum == NULL) 
      || (qddot != NULL) );

  if (update_kinematics)
    UpdateKinematicsCustom (model, &q, &qdot, qddot);

  for (size_t i = 1; i < model.mBodies.size(); i++) {
    model.Ic[i] = model.I[i];
    model.hc[i] = model.Ic[i].toMatrix() * model.v[i];
    model.hdotc[i] = model.Ic[i] * model.a[i] + crossf(model.v[i], model.Ic[i] * model.v[i]);
  }

  if (qddot && (com_acceleration || change_of_angular_momentum)) {
    for (size_t i = 1; i < model.mBodies.size(); i++) {
      model.hdotc[i] = model.Ic[i] * model.a[i] + crossf(model.v[i], model.Ic[i] * model.v[i]);
    }
  }

  SpatialRigidBodyInertia Itot (0., Vector3d (0., 0., 0.), Matrix3d::Zero(3,3));
  SpatialVector htot (SpatialVector::Zero(6));
  SpatialVector hdot_tot (SpatialVector::Zero(6));

  for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int lambda = model.lambda[i];

    if (lambda != 0) {
      model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose (model.Ic[i]);
      model.hc[lambda] = model.hc[lambda] + model.X_lambda[i].applyTranspose (model.hc[i]);
    } else {
      Itot = Itot + model.X_lambda[i].applyTranspose (model.Ic[i]);
      htot = htot + model.X_lambda[i].applyTranspose (model.hc[i]);
    }
  }

  if (qddot && (com_acceleration || change_of_angular_momentum)) {
    for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
      unsigned int lambda = model.lambda[i];

      if (lambda != 0) {
        model.hdotc[lambda] = model.hdotc[lambda] + model.X_lambda[i].applyTranspose (model.hdotc[i]);
      } else {
        hdot_tot = hdot_tot + model.X_lambda[i].applyTranspose (model.hdotc[i]);
      }
    }
  }

  mass = Itot.m;
  com = Itot.h / mass;
  LOG << "mass = " << mass << " com = " << com.transpose() << " htot = " << htot.transpose() << std::endl;

  if (com_velocity) {
    *com_velocity = Vector3d (htot[3] / mass, htot[4] / mass, htot[5] / mass);
  }

  if (angular_momentum) {
    htot = Xtrans (com).applyAdjoint (htot);
    angular_momentum->set (htot[0], htot[1], htot[2]);
  }

  if (com_acceleration) {
    *com_acceleration = Vector3d (hdot_tot[3] / mass, hdot_tot[4] / mass, hdot_tot[5] / mass);
  }

  if (change_of_angular_momentum) {
    hdot_tot = Xtrans (com).applyAdjoint (hdot_tot);
    change_of_angular_momentum->set (hdot_tot[0], hdot_tot[1], hdot_tot[2]);
  }
}

RBDL_DLLAPI void CalcZeroMomentPoint (
  Model &model,
  const Math::VectorNd &q,
  const Math::VectorNd &qdot,
  const Math::VectorNd &qddot,
  Vector3d* zmp,
  const Math::Vector3d &normal,
  const Math::Vector3d &point,
  bool update_kinematics
) {
  if (zmp == NULL) {
    cerr << "ZMP (output) is 'NULL'!" << endl;
    abort();
  }

  // update kinematics if required
  // NOTE UpdateKinematics computes model.a[i] and model.v[i] required for
  //      change of momentum
  if (update_kinematics) {
    UpdateKinematicsCustom (model, &q, &qdot, &qddot);
  }

  // compute change of momentum of each single body (same as in RNEA/InverseDynamics)
  for (size_t i = 1; i < model.mBodies.size(); i++) {
    model.Ic[i] = model.I[i];
    model.hdotc[i] = model.Ic[i] * model.a[i] + crossf(model.v[i], model.Ic[i] * model.v[i]);
  }

  SpatialRigidBodyInertia I_tot (0., Vector3d (0., 0., 0.), Matrix3d::Zero(3,3));
  SpatialVector h_tot (SpatialVector::Zero(6));
  SpatialVector hdot_tot (SpatialVector::Zero(6));

  // compute total change of momentum and CoM wrt to root body (idx = 0)
  // by recursively summing up local change of momentum
  for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int lambda = model.lambda[i];

    if (lambda != 0) {
      model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose (model.Ic[i]);
      model.hc[lambda] = model.hc[lambda] + model.X_lambda[i].applyTranspose (model.hc[i]);
      model.hdotc[lambda] = model.hdotc[lambda] + model.X_lambda[i].applyTranspose (model.hdotc[i]);
    } else {
      I_tot = I_tot + model.X_lambda[i].applyTranspose (model.Ic[i]);
      h_tot = h_tot + model.X_lambda[i].applyTranspose (model.hc[i]);
      hdot_tot = hdot_tot + model.X_lambda[i].applyTranspose (model.hdotc[i]);
    }
  }

  // compute CoM from mass and total inertia
  const double mass = I_tot.m;
  const Vector3d com = I_tot.h / mass;

  // project angular momentum onto CoM
  SpatialTransform Xcom = Xtrans (com);
  hdot_tot = Xcom.applyAdjoint (hdot_tot);

  // compute net external force at CoM by removing effects due to gravity
  hdot_tot = hdot_tot - mass * SpatialVector (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

  // express total change of momentum in world coordinates
  hdot_tot = Xcom.inverse().applyAdjoint (hdot_tot);

  // project purified change of momentum onto surface
  // z = n x n_0
  //     -------
  //     n * f
  Vector3d n_0 = hdot_tot.block<3,1>(0,0);
  Vector3d f = hdot_tot.block<3,1>(3,0);
  *zmp = normal.cross(n_0) / normal.dot(f);

  // double distance = (hdot_tot - point).dot(normal);
  // zmp = hdot_tot - distance * normal;

  return;
}

RBDL_DLLAPI double CalcPotentialEnergy (
    Model &model, 
    const Math::VectorNd &q, 
    bool update_kinematics) {
  double mass;
  Vector3d com;
  CalcCenterOfMass (
      model, 
      q,
      VectorNd::Zero (model.qdot_size),
      NULL,
      mass,
      com,
      NULL,
      NULL,
      NULL,
      NULL,
      update_kinematics);

  Vector3d g = - Vector3d (model.gravity[0], model.gravity[1], model.gravity[2]);
  LOG << "pot_energy: " << " mass = " << mass << " com = " << com.transpose() << std::endl;

  return mass * com.dot(g);
}

RBDL_DLLAPI double CalcKineticEnergy (
    Model &model, 
    const Math::VectorNd &q, 
    const Math::VectorNd &qdot, 
    bool update_kinematics) {
  if (update_kinematics)
    UpdateKinematicsCustom (model, &q, &qdot, NULL);

  double result = 0.;

  for (size_t i = 1; i < model.mBodies.size(); i++) {
    result += 0.5 * model.v[i].transpose() * (model.I[i] * model.v[i]);
  }
  return result;
}

}
}
