/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#ifdef WIN32
void sincos(double angle, double* s, double* c)
{
    if (s)
        *s = ::sin(angle);
    if (c)
        *c = ::cos(angle);
}
#endif //WIN32

namespace RigidBodyDynamics {

using namespace Math;

RBDL_DLLAPI void jcalc (
    Model &model,
    unsigned int joint_id,
    const VectorNd &q,
    const VectorNd &qdot
    ) {
  // exception if we calculate it for the root body
  assert (joint_id > 0);

  if (model.mJoints[joint_id].mJointType == JointTypeRevoluteX) {
    double s, c;
    sincos (q[model.mJoints[joint_id].q_index], &s, &c);

    model.X_lambda[joint_id].E = Matrix3d (
        model.X_T[joint_id].E(0, 0),
        model.X_T[joint_id].E(0, 1),
        model.X_T[joint_id].E(0, 2),

         c * model.X_T[joint_id].E(1, 0) + s * model.X_T[joint_id].E(2, 0),
         c * model.X_T[joint_id].E(1, 1) + s * model.X_T[joint_id].E(2, 1),
         c * model.X_T[joint_id].E(1, 2) + s * model.X_T[joint_id].E(2, 2),

        -s * model.X_T[joint_id].E(1, 0) + c * model.X_T[joint_id].E(2, 0),
        -s * model.X_T[joint_id].E(1, 1) + c * model.X_T[joint_id].E(2, 1),
        -s * model.X_T[joint_id].E(1, 2) + c * model.X_T[joint_id].E(2, 2));

    model.X_lambda[joint_id].r = model.X_T[joint_id].r;

    model.v_J[joint_id][0] = qdot[model.mJoints[joint_id].q_index];
  } else if (model.mJoints[joint_id].mJointType == JointTypeRevoluteY) {
    double s, c;
    sincos (q[model.mJoints[joint_id].q_index], &s, &c);

    model.X_lambda[joint_id].E = Matrix3d (
        c * model.X_T[joint_id].E(0, 0) + -s * model.X_T[joint_id].E(2, 0),
        c * model.X_T[joint_id].E(0, 1) + -s * model.X_T[joint_id].E(2, 1),
        c * model.X_T[joint_id].E(0, 2) + -s * model.X_T[joint_id].E(2, 2),

        model.X_T[joint_id].E(1, 0),
        model.X_T[joint_id].E(1, 1),
        model.X_T[joint_id].E(1, 2),

        s * model.X_T[joint_id].E(0, 0) + c * model.X_T[joint_id].E(2, 0),
        s * model.X_T[joint_id].E(0, 1) + c * model.X_T[joint_id].E(2, 1),
        s * model.X_T[joint_id].E(0, 2) + c * model.X_T[joint_id].E(2, 2));

    model.X_lambda[joint_id].r = model.X_T[joint_id].r;

    model.v_J[joint_id][1] = qdot[model.mJoints[joint_id].q_index];
  } else if (model.mJoints[joint_id].mJointType == JointTypeRevoluteZ) {
    double s, c;
    sincos (q[model.mJoints[joint_id].q_index], &s, &c);

    model.X_lambda[joint_id].E = Matrix3d (
         c * model.X_T[joint_id].E(0, 0) + s * model.X_T[joint_id].E(1, 0),
         c * model.X_T[joint_id].E(0, 1) + s * model.X_T[joint_id].E(1, 1),
         c * model.X_T[joint_id].E(0, 2) + s * model.X_T[joint_id].E(1, 2),

        -s * model.X_T[joint_id].E(0, 0) + c * model.X_T[joint_id].E(1, 0),
        -s * model.X_T[joint_id].E(0, 1) + c * model.X_T[joint_id].E(1, 1),
        -s * model.X_T[joint_id].E(0, 2) + c * model.X_T[joint_id].E(1, 2),

        model.X_T[joint_id].E(2, 0),
        model.X_T[joint_id].E(2, 1),
        model.X_T[joint_id].E(2, 2));

    model.X_lambda[joint_id].r = model.X_T[joint_id].r;

    model.v_J[joint_id][2] = qdot[model.mJoints[joint_id].q_index];
  } else if (model.mJoints[joint_id].mJointType == JointTypeHelical) {
    SpatialTransform X_J = jcalc_XJ (model, joint_id, q);
    jcalc_X_lambda_S(model, joint_id, q);
    double Jqd = qdot[model.mJoints[joint_id].q_index];
    model.v_J[joint_id] = model.S[joint_id] * Jqd;
    
    Vector3d St = model.S[joint_id].block(0,0,3,1);
    Vector3d c = X_J.E * model.mJoints[joint_id].mJointAxes[0].block(3,0,3,1);
    c = St.cross(c);
    c *= -Jqd * Jqd;    
    model.c_J[joint_id] = SpatialVector(0,0,0,c[0],c[1],c[2]);
    model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
  } else if (model.mJoints[joint_id].mDoFCount == 1 &&
      model.mJoints[joint_id].mJointType != JointTypeCustom) {
    SpatialTransform X_J = jcalc_XJ (model, joint_id, q);
    model.v_J[joint_id] = 
      model.S[joint_id] * qdot[model.mJoints[joint_id].q_index];
    model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
  } else if (model.mJoints[joint_id].mJointType == JointTypeSpherical) {
    SpatialTransform X_J = SpatialTransform (model.GetQuaternion (joint_id, q).toMatrix(), 
          Vector3d (0., 0., 0.));

    model.multdof3_S[joint_id](0,0) = 1.;
    model.multdof3_S[joint_id](1,1) = 1.;
    model.multdof3_S[joint_id](2,2) = 1.;

    Vector3d omega (qdot[model.mJoints[joint_id].q_index],
        qdot[model.mJoints[joint_id].q_index+1],
        qdot[model.mJoints[joint_id].q_index+2]);

    model.v_J[joint_id] = SpatialVector (
        omega[0], omega[1], omega[2],
        0., 0., 0.);
    model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerZYX) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    SpatialTransform X_J (Matrix3d(
        c0 * c1, s0 * c1, -s1,
        c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
        c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
        ),
        Vector3d::Zero());

    model.multdof3_S[joint_id](0,0) = -s1;
    model.multdof3_S[joint_id](0,2) = 1.;

    model.multdof3_S[joint_id](1,0) = c1 * s2;
    model.multdof3_S[joint_id](1,1) = c2;

    model.multdof3_S[joint_id](2,0) = c1 * c2;
    model.multdof3_S[joint_id](2,1) = - s2;

    double qdot0 = qdot[model.mJoints[joint_id].q_index];
    double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
    double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

    model.v_J[joint_id] = 
      model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

    model.c_J[joint_id].set(
        -c1*qdot0*qdot1,
        -s1*s2*qdot0*qdot1 + c1*c2*qdot0*qdot2 - s2*qdot1*qdot2,
        -s1*c2*qdot0*qdot1 - c1*s2*qdot0*qdot2 - c2*qdot1*qdot2,
        0.,0., 0.);
    model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerXYZ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    SpatialTransform X_J (Matrix3d(
        c2 * c1, s2 * c0 + c2 * s1 * s0, s2 * s0 - c2 * s1 * c0,
        -s2 * c1, c2 * c0 - s2 * s1 * s0, c2 * s0 + s2 * s1 * c0,
        s1, -c1 * s0, c1 * c0
        ),
        Vector3d::Zero());

    model.multdof3_S[joint_id](0,0) = c2 * c1;
    model.multdof3_S[joint_id](0,1) = s2;

    model.multdof3_S[joint_id](1,0) = -s2 * c1;
    model.multdof3_S[joint_id](1,1) = c2;

    model.multdof3_S[joint_id](2,0) = s1;
    model.multdof3_S[joint_id](2,2) = 1.;

    double qdot0 = qdot[model.mJoints[joint_id].q_index];
    double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
    double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

    model.v_J[joint_id] = 
      model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

    model.c_J[joint_id].set(
        -s2*c1*qdot2*qdot0 - c2*s1*qdot1*qdot0 + c2*qdot2*qdot1,
        -c2*c1*qdot2*qdot0 + s2*s1*qdot1*qdot0 - s2*qdot2*qdot1,
        c1*qdot1*qdot0,
        0., 0., 0.
        );
    model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerYXZ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    SpatialTransform X_J (Matrix3d(
        c2 * c0 + s2 * s1 * s0, s2 * c1, -c2 * s0 + s2 * s1 * c0,
        -s2 * c0 + c2 * s1 * s0, c2 * c1,  s2 * s0 + c2 * s1 * c0,
        c1 * s0,    - s1,                 c1 * c0),
        Vector3d::Zero());

    model.multdof3_S[joint_id](0,0) = s2 * c1;
    model.multdof3_S[joint_id](0,1) = c2;

    model.multdof3_S[joint_id](1,0) = c2 * c1;
    model.multdof3_S[joint_id](1,1) = -s2;

    model.multdof3_S[joint_id](2,0) = -s1;
    model.multdof3_S[joint_id](2,2) = 1.;

    double qdot0 = qdot[model.mJoints[joint_id].q_index];
    double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
    double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

    model.v_J[joint_id] = 
      model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

    model.c_J[joint_id].set(
        c2*c1*qdot2*qdot0 - s2*s1*qdot1*qdot0 - s2*qdot2*qdot1,
        -s2*c1*qdot2*qdot0 - c2*s1*qdot1*qdot0 - c2*qdot2*qdot1,
        -c1*qdot1*qdot0,
        0., 0., 0.
        );
    model.X_lambda[joint_id] = X_J * model.X_T[joint_id];
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerZXY) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    model.X_lambda[joint_id] = SpatialTransform (
        Matrix3d(
          -s0 * s1 * s2 + c0 * c2, s0 * c2 + s1 * s2 * c0, -s2 * c1,
          -s0 * c1, c0 * c1, s1,
          s0 * s1 * c2 + s2 * c0, s0 * s2 - s1 * c0 * c2, c1 * c2
          ),
        Vector3d::Zero())
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](0,0) = -s2 * c1;
    model.multdof3_S[joint_id](0,1) = c2;

    model.multdof3_S[joint_id](1,0) = s1;
    model.multdof3_S[joint_id](1,2) = 1;

    model.multdof3_S[joint_id](2,0) = c1 * c2;
    model.multdof3_S[joint_id](2,1) = s2;

    double qdot0 = qdot[model.mJoints[joint_id].q_index];
    double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
    double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

    model.v_J[joint_id] = 
      model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

    model.c_J[joint_id].set(
        (-c1 * c2 * qdot2 + s1 * s2 * qdot1) * qdot0 - s2 * qdot1 * qdot2,
        c1 * qdot1 * qdot0,
        (-s1 * c2 * qdot1 - c1 * s2 * qdot2) * qdot0 + c2 * qdot2 * qdot1,
        0., 0., 0.
        );
  } else if(model.mJoints[joint_id].mJointType == JointTypeTranslationXYZ){
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    model.multdof3_S[joint_id](3,0) = 1.;
    model.multdof3_S[joint_id](4,1) = 1.;
    model.multdof3_S[joint_id](5,2) = 1.;

    double qdot0 = qdot[model.mJoints[joint_id].q_index];
    double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
    double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

    model.v_J[joint_id] = 
      model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

    model.c_J[joint_id].set(0., 0., 0., 0., 0., 0.);
    model.X_lambda[joint_id].E = model.X_T[joint_id].E;
    model.X_lambda[joint_id].r = model.X_T[joint_id].r + model.X_T[joint_id].E.transpose() * Vector3d (q0, q1, q2);
  } else if (model.mJoints[joint_id].mJointType == JointTypeCustom) {
    const Joint &joint = model.mJoints[joint_id];
    CustomJoint *custom_joint = 
      model.mCustomJoints[joint.custom_joint_index];
    custom_joint->jcalc (model, joint_id, q, qdot);
  } else {
    std::cerr << "Error: invalid joint type " << model.mJoints[joint_id].mJointType << " at id " << joint_id << std::endl;
    abort();
  }
}

RBDL_DLLAPI Math::SpatialTransform jcalc_XJ (
    Model &model,
    unsigned int joint_id,
    const Math::VectorNd &q) {
  // exception if we calculate it for the root body
  assert (joint_id > 0);

  if (model.mJoints[joint_id].mDoFCount == 1
      && model.mJoints[joint_id].mJointType != JointTypeCustom) {
    if (model.mJoints[joint_id].mJointType == JointTypeRevolute) {
      return Xrot (q[model.mJoints[joint_id].q_index], Vector3d (
            model.mJoints[joint_id].mJointAxes[0][0],
            model.mJoints[joint_id].mJointAxes[0][1],
            model.mJoints[joint_id].mJointAxes[0][2]
            ));
    } else if (model.mJoints[joint_id].mJointType == JointTypePrismatic) {
      return Xtrans ( Vector3d (
            model.mJoints[joint_id].mJointAxes[0][3]
            * q[model.mJoints[joint_id].q_index],
            model.mJoints[joint_id].mJointAxes[0][4] 
            * q[model.mJoints[joint_id].q_index],
            model.mJoints[joint_id].mJointAxes[0][5] 
            * q[model.mJoints[joint_id].q_index]
            )
          );
    } else if (model.mJoints[joint_id].mJointType == JointTypeHelical) {
      SpatialTransform rot = Xrot(
          q[model.mJoints[joint_id].q_index], Vector3d (
            model.mJoints[joint_id].mJointAxes[0][0],
            model.mJoints[joint_id].mJointAxes[0][1],
            model.mJoints[joint_id].mJointAxes[0][2]
            ));
      SpatialTransform trans = Xtrans ( Vector3d (
            model.mJoints[joint_id].mJointAxes[0][3]
            * q[model.mJoints[joint_id].q_index],
            model.mJoints[joint_id].mJointAxes[0][4] 
            * q[model.mJoints[joint_id].q_index],
            model.mJoints[joint_id].mJointAxes[0][5] 
            * q[model.mJoints[joint_id].q_index]
            )
          );
      return rot * trans;
    }
  }
  std::cerr << "Error: invalid joint type: " << model.mJoints[joint_id].mJointType << std::endl;
  abort();
  return SpatialTransform();
}

RBDL_DLLAPI void jcalc_X_lambda_S (
    Model &model,
    unsigned int joint_id,
    const VectorNd &q
    ) {
  // exception if we calculate it for the root body
  assert (joint_id > 0);

  if (model.mJoints[joint_id].mJointType == JointTypeRevoluteX) {
    double s, c;
    sincos (q[model.mJoints[joint_id].q_index], &s, &c);

    model.X_lambda[joint_id].E = Matrix3d (
        model.X_T[joint_id].E(0, 0),
        model.X_T[joint_id].E(0, 1),
        model.X_T[joint_id].E(0, 2),

         c * model.X_T[joint_id].E(1, 0) + s * model.X_T[joint_id].E(2, 0),
         c * model.X_T[joint_id].E(1, 1) + s * model.X_T[joint_id].E(2, 1),
         c * model.X_T[joint_id].E(1, 2) + s * model.X_T[joint_id].E(2, 2),

        -s * model.X_T[joint_id].E(1, 0) + c * model.X_T[joint_id].E(2, 0),
        -s * model.X_T[joint_id].E(1, 1) + c * model.X_T[joint_id].E(2, 1),
        -s * model.X_T[joint_id].E(1, 2) + c * model.X_T[joint_id].E(2, 2));

    model.X_lambda[joint_id].r = model.X_T[joint_id].r;

    model.S[joint_id][0] = 1.0;
  } else if (model.mJoints[joint_id].mJointType == JointTypeRevoluteY) {
    double s, c;
    sincos (q[model.mJoints[joint_id].q_index], &s, &c);

    model.X_lambda[joint_id].E = Matrix3d (
        c * model.X_T[joint_id].E(0, 0) + -s * model.X_T[joint_id].E(2, 0),
        c * model.X_T[joint_id].E(0, 1) + -s * model.X_T[joint_id].E(2, 1),
        c * model.X_T[joint_id].E(0, 2) + -s * model.X_T[joint_id].E(2, 2),

        model.X_T[joint_id].E(1, 0),
        model.X_T[joint_id].E(1, 1),
        model.X_T[joint_id].E(1, 2),

        s * model.X_T[joint_id].E(0, 0) + c * model.X_T[joint_id].E(2, 0),
        s * model.X_T[joint_id].E(0, 1) + c * model.X_T[joint_id].E(2, 1),
        s * model.X_T[joint_id].E(0, 2) + c * model.X_T[joint_id].E(2, 2));

    model.X_lambda[joint_id].r = model.X_T[joint_id].r;

    model.S[joint_id][1] = 1.;
  } else if (model.mJoints[joint_id].mJointType == JointTypeRevoluteZ) {
    double s, c;
    sincos (q[model.mJoints[joint_id].q_index], &s, &c);

    model.X_lambda[joint_id].E = Matrix3d (
         c * model.X_T[joint_id].E(0, 0) + s * model.X_T[joint_id].E(1, 0),
         c * model.X_T[joint_id].E(0, 1) + s * model.X_T[joint_id].E(1, 1),
         c * model.X_T[joint_id].E(0, 2) + s * model.X_T[joint_id].E(1, 2),

        -s * model.X_T[joint_id].E(0, 0) + c * model.X_T[joint_id].E(1, 0),
        -s * model.X_T[joint_id].E(0, 1) + c * model.X_T[joint_id].E(1, 1),
        -s * model.X_T[joint_id].E(0, 2) + c * model.X_T[joint_id].E(1, 2),

        model.X_T[joint_id].E(2, 0),
        model.X_T[joint_id].E(2, 1),
        model.X_T[joint_id].E(2, 2));

    model.X_lambda[joint_id].r = model.X_T[joint_id].r;

    model.S[joint_id][2] = 1.;
  } else if (model.mJoints[joint_id].mJointType == JointTypeHelical){
    SpatialTransform XJ = jcalc_XJ (model, joint_id, q);
    model.X_lambda[joint_id] = XJ * model.X_T[joint_id];
    // Set the joint axis
    Vector3d trans = XJ.E * model.mJoints[joint_id].mJointAxes[0].block(3,0,3,1);
    
    model.S[joint_id] = SpatialVector(model.mJoints[joint_id].mJointAxes[0][0],
           model.mJoints[joint_id].mJointAxes[0][1],
           model.mJoints[joint_id].mJointAxes[0][2],
           trans[0], trans[1], trans[2]);
  } else if (model.mJoints[joint_id].mDoFCount == 1
      && model.mJoints[joint_id].mJointType != JointTypeCustom){
    model.X_lambda[joint_id] = 
      jcalc_XJ (model, joint_id, q) * model.X_T[joint_id];
    model.S[joint_id] = model.mJoints[joint_id].mJointAxes[0];
  } else if (model.mJoints[joint_id].mJointType == JointTypeSpherical) {
    model.X_lambda[joint_id] = SpatialTransform (
        model.GetQuaternion (joint_id, q).toMatrix(),
        Vector3d (0., 0., 0.))
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](0,0) = 1.;
    model.multdof3_S[joint_id](1,1) = 1.;
    model.multdof3_S[joint_id](2,2) = 1.;
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerZYX) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    model.X_lambda[joint_id] = SpatialTransform ( 
        Matrix3d(
          c0 * c1, s0 * c1, -s1,
          c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
          c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
          ),
        Vector3d (0., 0., 0.))
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](0,0) = -s1;
    model.multdof3_S[joint_id](0,2) = 1.;

    model.multdof3_S[joint_id](1,0) = c1 * s2;
    model.multdof3_S[joint_id](1,1) = c2;

    model.multdof3_S[joint_id](2,0) = c1 * c2;
    model.multdof3_S[joint_id](2,1) = - s2;
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerXYZ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    model.X_lambda[joint_id] = SpatialTransform (
        Matrix3d(
          c2 * c1, s2 * c0 + c2 * s1 * s0, s2 * s0 - c2 * s1 * c0,
          -s2 * c1, c2 * c0 - s2 * s1 * s0, c2 * s0 + s2 * s1 * c0,
          s1, -c1 * s0, c1 * c0
          ),
        Vector3d (0., 0., 0.))
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](0,0) = c2 * c1;
    model.multdof3_S[joint_id](0,1) = s2;

    model.multdof3_S[joint_id](1,0) = -s2 * c1;
    model.multdof3_S[joint_id](1,1) = c2;

    model.multdof3_S[joint_id](2,0) = s1;
    model.multdof3_S[joint_id](2,2) = 1.;
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerYXZ ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    model.X_lambda[joint_id] = SpatialTransform (
        Matrix3d(
          c2 * c0 + s2 * s1 * s0, s2 * c1, -c2 * s0 + s2 * s1 * c0,
          -s2 * c0 + c2 * s1 * s0, c2 * c1, s2 * s0 + c2 * s1 * c0,
          c1 * s0, - s1, c1 * c0
          ),
        Vector3d (0., 0., 0.))
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](0,0) = s2 * c1;
    model.multdof3_S[joint_id](0,1) = c2;

    model.multdof3_S[joint_id](1,0) = c2 * c1;
    model.multdof3_S[joint_id](1,1) = -s2;

    model.multdof3_S[joint_id](2,0) = -s1;
    model.multdof3_S[joint_id](2,2) = 1.;
  } else if (model.mJoints[joint_id].mJointType == JointTypeEulerZXY ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    double s0 = sin (q0);
    double c0 = cos (q0);
    double s1 = sin (q1);
    double c1 = cos (q1);
    double s2 = sin (q2);
    double c2 = cos (q2);

    model.X_lambda[joint_id] = SpatialTransform (
        Matrix3d(
          -s0 * s1 * s2 + c0 * c2, s0 * c2 + s1 * s2 * c0, -s2 * c1,
          -s0 * c1, c0 * c1, s1,
          s0 * s1 * c2 + s2 * c0, s0 * s2 - s1 * c0 * c2, c1 * c2
          ),
        Vector3d::Zero())
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](0,0) = -s2 * c1;
    model.multdof3_S[joint_id](0,1) = c2;

    model.multdof3_S[joint_id](1,0) = s1;
    model.multdof3_S[joint_id](1,2) = 1;

    model.multdof3_S[joint_id](2,0) = c1 * c2;
    model.multdof3_S[joint_id](2,1) = s2;
  } else if (model.mJoints[joint_id].mJointType == JointTypeTranslationXYZ) {
    double q0 = q[model.mJoints[joint_id].q_index];
    double q1 = q[model.mJoints[joint_id].q_index + 1];
    double q2 = q[model.mJoints[joint_id].q_index + 2];

    model.X_lambda[joint_id] = SpatialTransform (
        Matrix3d::Identity (3,3),
        Vector3d (q0, q1, q2))
      * model.X_T[joint_id];

    model.multdof3_S[joint_id](3,0) = 1.;
    model.multdof3_S[joint_id](4,1) = 1.;
    model.multdof3_S[joint_id](5,2) = 1.;
  } else if (model.mJoints[joint_id].mJointType == JointTypeCustom) {
    const Joint &joint = model.mJoints[joint_id];
    CustomJoint *custom_joint 
      = model.mCustomJoints[joint.custom_joint_index];

    custom_joint->jcalc_X_lambda_S (model, joint_id, q);
  } else {
    std::cerr << "Error: invalid joint type!" << std::endl;
    abort();
  }
}
}
