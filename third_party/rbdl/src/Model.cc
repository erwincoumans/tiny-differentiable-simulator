/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "rbdl/rbdl_mathutils.h"

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Body.h"
#include "rbdl/Joint.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Model::Model() {
  Body root_body;
  Joint root_joint;

  Vector3d zero_position (0., 0., 0.);
  SpatialVector zero_spatial (0., 0., 0., 0., 0., 0.);

  // structural information
  lambda.push_back(0);
  lambda_q.push_back(0);
  mu.push_back(std::vector<unsigned int>());
  dof_count = 0;
  q_size = 0;
  qdot_size = 0;
  previously_added_body_id = 0;

  gravity = Vector3d (0., -9.81, 0.);

  // state information
  v.push_back(zero_spatial);
  a.push_back(zero_spatial);

  // Joints
  mJoints.push_back(root_joint);
  S.push_back (zero_spatial);
  X_T.push_back(SpatialTransform());

  v_J.push_back (zero_spatial);
  c_J.push_back (zero_spatial);

  // Spherical joints
  multdof3_S.push_back (Matrix63::Zero());
  multdof3_U.push_back (Matrix63::Zero());
  multdof3_Dinv.push_back (Matrix3d::Zero());
  multdof3_u.push_back (Vector3d::Zero());
  multdof3_w_index.push_back (0);

  // Dynamic variables
  c.push_back(zero_spatial);
  IA.push_back(SpatialMatrix::Identity());
  pA.push_back(zero_spatial);
  U.push_back(zero_spatial);

  u = VectorNd::Zero(1);
  d = VectorNd::Zero(1);

  f.push_back (zero_spatial);
  SpatialRigidBodyInertia rbi(0., 
      Vector3d (0., 0., 0.), 
      Matrix3d::Zero(3,3));
  Ic.push_back (rbi);
  I.push_back(rbi);
  hc.push_back (zero_spatial);
  hdotc.push_back (zero_spatial);

  // Bodies
  X_lambda.push_back(SpatialTransform());
  X_base.push_back(SpatialTransform());

  mBodies.push_back(root_body);
  mBodyNameMap["ROOT"] = 0;

  fixed_body_discriminator = std::numeric_limits<unsigned int>::max() / 2;
}

unsigned int AddBodyFixedJoint (
    Model &model,
    const unsigned int parent_id,
    const SpatialTransform &joint_frame,
    const Joint &joint,
    const Body &body,
    std::string body_name) {
  FixedBody fbody = FixedBody::CreateFromBody (body);
  fbody.mMovableParent = parent_id;
  fbody.mParentTransform = joint_frame;

  if (model.IsFixedBodyId(parent_id)) {
    FixedBody fixed_parent = 
      model.mFixedBodies[parent_id - model.fixed_body_discriminator];

    fbody.mMovableParent = fixed_parent.mMovableParent;
    fbody.mParentTransform = joint_frame * fixed_parent.mParentTransform;
  }

  // merge the two bodies
  Body parent_body = model.mBodies[fbody.mMovableParent];
  parent_body.Join (fbody.mParentTransform, body);
  model.mBodies[fbody.mMovableParent] = parent_body;
  model.I[fbody.mMovableParent] = 
    SpatialRigidBodyInertia::createFromMassComInertiaC ( 
        parent_body.mMass, 
        parent_body.mCenterOfMass, 
        parent_body.mInertia);

  model.mFixedBodies.push_back (fbody);

  if (model.mFixedBodies.size() > 
      std::numeric_limits<unsigned int>::max() - model.fixed_body_discriminator) {
    std::cerr << "Error: cannot add more than " 
      << std::numeric_limits<unsigned int>::max() 
      - model.mFixedBodies.size() 
      << " fixed bodies. You need to modify "
      << "Model::fixed_body_discriminator for this." 
      << std::endl;
    assert (0);
    abort();
  }

  if (body_name.size() != 0) {
    if (model.mBodyNameMap.find(body_name) != model.mBodyNameMap.end()) {
      std::cerr << "Error: Body with name '" 
        << body_name 
        << "' already exists!" 
        << std::endl;
      assert (0);
      abort();
    }
    model.mBodyNameMap[body_name] = model.mFixedBodies.size() 
      + model.fixed_body_discriminator - 1;
  }

  return model.mFixedBodies.size() + model.fixed_body_discriminator - 1;
}

unsigned int AddBodyMultiDofJoint (
    Model &model,
    const unsigned int parent_id,
    const SpatialTransform &joint_frame,
    const Joint &joint,
    const Body &body,
    std::string body_name) {
  // Here we emulate multi DoF joints by simply adding nullbodies. This
  // allows us to use fixed size elements for S,v,a, etc. which is very
  // fast in Eigen.
  unsigned int joint_count = 0;
  if (joint.mJointType == JointType1DoF)
    joint_count = 1;
  else if (joint.mJointType == JointType2DoF)
    joint_count = 2;
  else if (joint.mJointType == JointType3DoF)
    joint_count = 3;
  else if (joint.mJointType == JointType4DoF)
    joint_count = 4;
  else if (joint.mJointType == JointType5DoF)
    joint_count = 5;
  else if (joint.mJointType == JointType6DoF)
    joint_count = 6;
  else if (joint.mJointType == JointTypeFloatingBase)
    // no action required
  {}
  else {
    std::cerr << "Error: Invalid joint type: " 
      << joint.mJointType 
      << std::endl;

    assert (0 && !"Invalid joint type!");
  }

  Body null_body (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
  null_body.mIsVirtual = true;

  unsigned int null_parent = parent_id;
  SpatialTransform joint_frame_transform;

  if (joint.mJointType == JointTypeFloatingBase) {
    null_parent = model.AddBody (parent_id, 
        joint_frame, 
        JointTypeTranslationXYZ, 
        null_body);

    return model.AddBody (null_parent, 
        SpatialTransform(), 
        JointTypeSpherical, 
        body, 
        body_name);
  }

  Joint single_dof_joint;
  unsigned int j;

  // Here we add multiple virtual bodies that have no mass or inertia for
  // which each is attached to the model with a single degree of freedom
  // joint.
  for (j = 0; j < joint_count; j++) {
    single_dof_joint = Joint (joint.mJointAxes[j]);

    if (single_dof_joint.mJointType == JointType1DoF) {
      Vector3d rotation (
          joint.mJointAxes[j][0],
          joint.mJointAxes[j][1],
          joint.mJointAxes[j][2]);
      Vector3d translation (
          joint.mJointAxes[j][3],
          joint.mJointAxes[j][4],
          joint.mJointAxes[j][5]);

      if (rotation == Vector3d (0., 0., 0.)) {
        single_dof_joint = Joint (JointTypePrismatic, translation);
      } else if (translation == Vector3d (0., 0., 0.)) {
        single_dof_joint = Joint (JointTypeRevolute, rotation);
      }
    }

    // the first joint has to be transformed by joint_frame, all the
    // others must have a null transformation
    if (j == 0)
      joint_frame_transform = joint_frame;
    else
      joint_frame_transform = SpatialTransform();

    if (j == joint_count - 1) 
      // if we are at the last we must add the real body
      break;
    else {
      // otherwise we just add an intermediate body
      null_parent = model.AddBody (null_parent, 
          joint_frame_transform, 
          single_dof_joint,
          null_body);
    }
  }

  return model.AddBody (null_parent, 
      joint_frame_transform, 
      single_dof_joint, 
      body,
      body_name);
}

unsigned int Model::AddBody(
    const unsigned int parent_id,
    const SpatialTransform &joint_frame,
    const Joint &joint,
    const Body &body,
    std::string body_name) {
  assert (lambda.size() > 0);
  assert (joint.mJointType != JointTypeUndefined);

  if (joint.mJointType == JointTypeFixed) {
    previously_added_body_id = AddBodyFixedJoint (*this, 
        parent_id, 
        joint_frame, 
        joint, 
        body, 
        body_name);

    return previously_added_body_id;
  } else if ( (joint.mJointType == JointTypeSpherical) 
      || (joint.mJointType == JointTypeEulerZYX) 
      || (joint.mJointType == JointTypeEulerXYZ) 
      || (joint.mJointType == JointTypeEulerYXZ) 
      || (joint.mJointType == JointTypeEulerZXY) 
      || (joint.mJointType == JointTypeTranslationXYZ) 
      || (joint.mJointType == JointTypeCustom) 
      ) {
    // no action required
  } else if (joint.mJointType != JointTypePrismatic 
      && joint.mJointType != JointTypeRevolute
      && joint.mJointType != JointTypeRevoluteX
      && joint.mJointType != JointTypeRevoluteY
      && joint.mJointType != JointTypeRevoluteZ
      && joint.mJointType != JointTypeHelical
      ) {
    previously_added_body_id = AddBodyMultiDofJoint (*this, 
        parent_id, 
        joint_frame, 
        joint, 
        body,
        body_name);
    return previously_added_body_id;
  }

  // If we add the body to a fixed body we have to make sure that we
  // actually add it to its movable parent.
  unsigned int movable_parent_id = parent_id;
  SpatialTransform movable_parent_transform;

  if (IsFixedBodyId(parent_id)) {
    unsigned int fbody_id = parent_id - fixed_body_discriminator;
    movable_parent_id = mFixedBodies[fbody_id].mMovableParent;
    movable_parent_transform = mFixedBodies[fbody_id].mParentTransform;
  }

  // structural information
  lambda.push_back(movable_parent_id);
  unsigned int lambda_q_last = mJoints[mJoints.size() - 1].q_index;

  if (mJoints[mJoints.size() - 1].mDoFCount > 0
      && mJoints[mJoints.size() - 1].mJointType != JointTypeCustom) {
    lambda_q_last = lambda_q_last + mJoints[mJoints.size() - 1].mDoFCount;
  } else if (mJoints[mJoints.size() - 1].mJointType == JointTypeCustom) {
    unsigned int custom_index = mJoints[mJoints.size() - 1].custom_joint_index;
    lambda_q_last = lambda_q_last 
      + mCustomJoints[mCustomJoints.size() - 1]->mDoFCount;
  }

  for (unsigned int i = 0; i < joint.mDoFCount; i++) {
    lambda_q.push_back(lambda_q_last + i);
  }
  mu.push_back(std::vector<unsigned int>());
  mu.at(movable_parent_id).push_back(mBodies.size());

  // Bodies
  X_lambda.push_back(SpatialTransform());
  X_base.push_back(SpatialTransform());
  mBodies.push_back(body);

  if (body_name.size() != 0) {
    if (mBodyNameMap.find(body_name) != mBodyNameMap.end()) {
      std::cerr << "Error: Body with name '" 
        << body_name 
        << "' already exists!"
        << std::endl;
      assert (0);
      abort();
    }
    mBodyNameMap[body_name] = mBodies.size() - 1;
  }

  // state information
  v.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
  a.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

  // Joints
  unsigned int prev_joint_index = mJoints.size() - 1;
  mJoints.push_back(joint);

  if (mJoints[prev_joint_index].mJointType != JointTypeCustom) {
    mJoints[mJoints.size() - 1].q_index = 
      mJoints[prev_joint_index].q_index + mJoints[prev_joint_index].mDoFCount;
  } else {
    mJoints[mJoints.size() - 1].q_index = 
      mJoints[prev_joint_index].q_index + mJoints[prev_joint_index].mDoFCount; 
  }

  S.push_back (joint.mJointAxes[0]);

  // Joint state variables
  v_J.push_back (joint.mJointAxes[0]);
  c_J.push_back (SpatialVector(0., 0., 0., 0., 0., 0.));

  // workspace for joints with 3 dof
  multdof3_S.push_back (Matrix63::Zero(6,3));
  multdof3_U.push_back (Matrix63::Zero());
  multdof3_Dinv.push_back (Matrix3d::Zero());
  multdof3_u.push_back (Vector3d::Zero());
  multdof3_w_index.push_back (0);

  dof_count = dof_count + joint.mDoFCount;

  // update the w components of the Quaternions. They are stored at the end
  // of the q vector
  int multdof3_joint_counter = 0;
  int mCustomJoint_joint_counter = 0;
  for (unsigned int i = 1; i < mJoints.size(); i++) {
    if (mJoints[i].mJointType == JointTypeSpherical) {
      multdof3_w_index[i] = dof_count + multdof3_joint_counter;
      multdof3_joint_counter++;
    }
  }

  q_size = dof_count
    + multdof3_joint_counter;

  qdot_size = qdot_size + joint.mDoFCount;

  // we have to invert the transformation as it is later always used from the
  // child bodies perspective.
  X_T.push_back(joint_frame * movable_parent_transform);

  // Dynamic variables
  c.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
  IA.push_back(SpatialMatrix::Zero(6,6));
  pA.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
  U.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

  d = VectorNd::Zero (mBodies.size());
  u = VectorNd::Zero (mBodies.size());

  f.push_back (SpatialVector (0., 0., 0., 0., 0., 0.));

  SpatialRigidBodyInertia rbi = 
    SpatialRigidBodyInertia::createFromMassComInertiaC (body.mMass, 
        body.mCenterOfMass,
        body.mInertia);

  Ic.push_back (rbi);
  I.push_back (rbi);
  hc.push_back (SpatialVector(0., 0., 0., 0., 0., 0.));
  hdotc.push_back (SpatialVector(0., 0., 0., 0., 0., 0.));

  if (mBodies.size() == fixed_body_discriminator) {
    std::cerr << "Error: cannot add more than " 
      << fixed_body_discriminator 
      <<  " movable bodies. You need to modify "
      "Model::fixed_body_discriminator for this."
      << std::endl;
    assert (0);
    abort();
  }

  previously_added_body_id = mBodies.size() - 1;

  mJointUpdateOrder.clear();

  // update the joint order computation
  std::vector<std::pair<JointType, unsigned int> > joint_types;
  for (unsigned int i = 0; i < mJoints.size(); i++) {
    joint_types.push_back(
        std::pair<JointType, unsigned int> (mJoints[i].mJointType,i));
    mJointUpdateOrder.push_back (mJoints[i].mJointType);
  }

  mJointUpdateOrder.clear();
  JointType current_joint_type = JointTypeUndefined;
  while (joint_types.size() != 0) {
    current_joint_type = joint_types[0].first;

    std::vector<std::pair<JointType, unsigned int> >::iterator type_iter = 
      joint_types.begin();

    while (type_iter != joint_types.end()) {
      if (type_iter->first == current_joint_type) {
        mJointUpdateOrder.push_back (type_iter->second);
        type_iter = joint_types.erase (type_iter);
      } else {
        ++type_iter;
      }
    }
  }

  //  for (unsigned int i = 0; i < mJointUpdateOrder.size(); i++) {
  //    std::cout << "i = " << i << ": joint_id = " << mJointUpdateOrder[i] 
  // << " joint_type = " << mJoints[mJointUpdateOrder[i]].mJointType << std::endl;
  //  }

  return previously_added_body_id;
}

unsigned int Model::AppendBody (
    const Math::SpatialTransform &joint_frame,
    const Joint &joint,
    const Body &body,
    std::string body_name) {
  return Model::AddBody (previously_added_body_id, 
      joint_frame,
      joint, 
      body,
      body_name);
}

unsigned int Model::AddBodyCustomJoint (
    const unsigned int parent_id,
    const Math::SpatialTransform &joint_frame,
    CustomJoint *custom_joint,
    const Body &body,
    std::string body_name) {
  Joint proxy_joint (JointTypeCustom, custom_joint->mDoFCount);
  proxy_joint.custom_joint_index = mCustomJoints.size();
  //proxy_joint.mDoFCount = custom_joint->mDoFCount; //MM added. Otherwise
  //model.q_size = 0, which is not good.

  mCustomJoints.push_back (custom_joint);

  unsigned int body_id = AddBody (parent_id, 
      joint_frame, 
      proxy_joint, 
      body,
      body_name);

  return body_id;
}

