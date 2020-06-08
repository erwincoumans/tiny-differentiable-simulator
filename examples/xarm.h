/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef XARM6_H
#define XARM6_H

#include <vector>

#include "tiny_matrix3x3.h"
#include "tiny_multi_body.h"
#include "tiny_spatial_transform.h"

template <typename TinyScalar, typename TinyConstants>
void init_xarm6(TinyMultiBody<TinyScalar, TinyConstants>& mb) {
  typedef TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef TinySymmetricSpatialDyad<TinyScalar, TinyConstants>
      TinySymmetricSpatialDyad;

  {
    // link1, <origin rpy="0 0 0" xyz="0 0 0.267"/>
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_Z);
    l.m_X_T.m_rotation.set_identity();
    l.m_X_T.m_translation.setValue(TinyConstants::zero(), TinyConstants::zero(),
                                   TinyConstants::fraction(267, 1000));
    TinyScalar mass = TinyConstants::fraction(216, 100);
    TinyVector3 com;
    com.setValue(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::zero());

    // <mass value = "2.16" / >
    // <inertia ixx = "0.00539427" ixy = "0" ixz = "0" iyy = "0.0048979" iyz =
    // "0" izz = "0.00311573" / >
    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = TinyConstants::fraction(539427, 100000000);
    inertia_C(1, 1) = TinyConstants::fraction(48979, 10000000);
    inertia_C(2, 2) = TinyConstants::fraction(311573, 100000000);
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    // l.m_I.print("inertia");
    mb.attach(l);
  }
  {
    // link2, <origin rpy="-1.5708 0 0" xyz="0 0 0"/>
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_Z);
    // <origin rpy = "0 0 0" xyz = "0 0 0" / >
    // <mass value = "1.71" / >
    // <inertia ixx = "0.0248674" ixy = "0" ixz = "0" iyy = "0.00485548" iyz =
    // "0" izz = "0.02387827" / >
    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = TinyConstants::fraction(248674, 10000000);
    inertia_C(1, 1) = TinyConstants::fraction(485548, 100000000);
    inertia_C(2, 2) = TinyConstants::fraction(2387827, 100000000);
    TinyScalar mass = TinyConstants::fraction(171, 100);
    TinyVector3 com;
    com.setValue(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::zero());
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    // l.m_I.print("inertia");

    l.m_X_T.m_rotation.setEulerZYX(-TinyConstants::half_pi(),
                                   TinyConstants::zero(),
                                   TinyConstants::zero());
    l.m_X_T.m_translation.setValue(TinyConstants::zero(), TinyConstants::zero(),
                                   TinyConstants::zero());
    mb.attach(l);
  }
  {
    // link3, <origin rpy="0 0 0" xyz="0.0535 -0.2845 0"/>
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_Z);
    // <mass value = "1.384" / >
    // <inertia ixx="0.0053694" ixy = "0" ixz = "0" iyy = "0.0032423" iyz = "0"
    // izz = "0.00501731" / >
    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = TinyConstants::fraction(53694, 10000000);
    inertia_C(1, 1) = TinyConstants::fraction(32423, 10000000);
    inertia_C(2, 2) = TinyConstants::fraction(501731, 100000000);
    TinyScalar mass = TinyConstants::fraction(1384, 1000);
    TinyVector3 com;
    com.setValue(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::zero());
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    // l.m_I.print("inertia");

    l.m_X_T.m_rotation.setEulerZYX(TinyConstants::zero(), TinyConstants::zero(),
                                   TinyConstants::zero());
    l.m_X_T.m_translation.setValue(TinyConstants::fraction(535, 10000),
                                   TinyConstants::fraction(-2845, 10000),
                                   TinyConstants::zero());
    mb.attach(l);
  }
  {
    // link4, <origin rpy="-1.5708 0 0" xyz="0.0775 0.3425 0"/>
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_Z);

    //<mass value = "1.115" / >
    //<inertia ixx = "0.00439263" ixy = "5.028E-05" ixz = "1.374E-05" iyy =
    //"0.0040077" iyz = "0.00045338" izz = "0.00110321" / >

    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = TinyConstants::fraction(439263, 100000000);
    inertia_C(1, 0) = TinyConstants::fraction(5028, 100000000);
    inertia_C(0, 1) = TinyConstants::fraction(5028, 100000000);
    inertia_C(2, 0) = TinyConstants::fraction(1374, 100000000);
    inertia_C(0, 2) = TinyConstants::fraction(1374, 100000000);
    inertia_C(1, 1) = TinyConstants::fraction(40077, 10000000);
    inertia_C(2, 1) = TinyConstants::fraction(45338, 100000000);
    inertia_C(1, 2) = TinyConstants::fraction(45338, 100000000);
    inertia_C(2, 2) = TinyConstants::fraction(110321, 100000000);
    TinyScalar mass = TinyConstants::fraction(1115, 1000);
    TinyVector3 com;
    com.setValue(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::zero());
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    // l.m_I.print("inertia");

    l.m_X_T.m_rotation.setEulerZYX(-TinyConstants::half_pi(),
                                   TinyConstants::zero(),
                                   TinyConstants::zero());
    l.m_X_T.m_translation.setValue(TinyConstants::fraction(775, 10000),
                                   TinyConstants::fraction(3425, 10000),
                                   TinyConstants::zero());
    mb.attach(l);
  }
  {
    // link5, <origin rpy="1.5708 0 0" xyz="0 0 0"/>
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_Z);

    //<mass value = "1.275" / >
    //<inertia ixx = "0.001202758" ixy = "0.000492428" ixz = "-0.00039147" iyy =
    //"0.0022876" iyz = "-1.235E-04" izz = "0.0026866" / >

    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = TinyConstants::fraction(1202758, 1000000000);
    inertia_C(1, 0) = TinyConstants::fraction(492428, 1000000000);
    inertia_C(0, 1) = TinyConstants::fraction(492428, 1000000000);
    inertia_C(2, 0) = TinyConstants::fraction(-39147, 100000000);
    inertia_C(0, 2) = TinyConstants::fraction(-39147, 100000000);
    inertia_C(1, 1) = TinyConstants::fraction(22876, 10000000);
    inertia_C(2, 1) = TinyConstants::fraction(-1235, 10000000);
    inertia_C(1, 2) = TinyConstants::fraction(-1235, 10000000);
    inertia_C(2, 2) = TinyConstants::fraction(26866, 10000000);

    TinyScalar mass = TinyConstants::fraction(1275, 1000);
    TinyVector3 com;
    com.setValue(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::zero());
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    // l.m_I.print("inertia");

    l.m_X_T.m_rotation.setEulerZYX(
        TinyConstants::half_pi(), TinyConstants::zero(), TinyConstants::zero());
    l.m_X_T.m_translation.setValue(TinyConstants::zero(), TinyConstants::zero(),
                                   TinyConstants::zero());
    mb.attach(l);
  }
  {
    // link6, <origin rpy="-1.5708 0 0" xyz="0.076 0.097 0"/>
    TinyLink<TinyScalar, TinyConstants> l;
    l.set_joint_type(JOINT_REVOLUTE_Z);

    //<mass value = "0.1096" / >
    //<inertia ixx = "4.5293E-05" ixy = "0" ixz = "0" iyy = "4.8111E-05" iyz =
    //"0" izz = "7.9715E-05" / >

    TinyMatrix3x3 inertia_C;
    inertia_C.set_identity();
    inertia_C(0, 0) = TinyConstants::fraction(45293, 1000000000);
    inertia_C(1, 1) = TinyConstants::fraction(48111, 1000000000);
    inertia_C(2, 2) = TinyConstants::fraction(79715, 1000000000);
    TinyScalar mass = TinyConstants::fraction(1096, 10000);
    TinyVector3 com;
    com.setValue(TinyConstants::zero(), TinyConstants::zero(),
                 TinyConstants::zero());
    l.m_I = TinySymmetricSpatialDyad::computeInertiaDyad(mass, com, inertia_C);
    // l.m_I.print("inertia");

    l.m_X_T.m_rotation.setEulerZYX(-TinyConstants::half_pi(),
                                   TinyConstants::zero(),
                                   TinyConstants::zero());
    l.m_X_T.m_translation.setValue(TinyConstants::fraction(76, 1000),
                                   TinyConstants::fraction(97, 1000),
                                   TinyConstants::zero());
    mb.attach(l);
  }
}

#endif  // XARM6_H
