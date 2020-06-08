// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>

#include "examples/motion_import.h"
#include "examples/tiny_urdf_parser.h"
#include "fix64_scalar.h"
#include "tiny_double_utils.h"
#include "tiny_matrix3x3.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_multi_body.h"
#include "tiny_pose.h"
#include "tiny_quaternion.h"
#include "tiny_raycast.h"
#include "tiny_rigid_body.h"
#include "tiny_urdf_structures.h"
#include "tiny_urdf_to_multi_body.h"
#include "tiny_vector3.h"
#include "tiny_world.h"

template <typename TinyScalar, typename TinyConstants>
struct UrdfToMultiBody2 {
  typedef ::TinyUrdfStructures<TinyScalar, TinyConstants> TinyUrdfStructures;

  void convert(TinyUrdfStructures *urdf_structures,
               TinyWorld<TinyScalar, TinyConstants> *world,
               TinyMultiBody<double, DoubleUtils> *mb) {
    TinyUrdfToMultiBody<double, DoubleUtils>::convert_to_multi_body(
        *urdf_structures, *world, *mb);

    mb->initialize();
  }
};

namespace py = pybind11;

PYBIND11_MODULE(pytinydiffsim, m) {
  m.doc() = R"pbdoc(
        tiny differentiable physics python plugin
        -----------------------

        .. currentmodule:: pytinydiffsim

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  py::class_<TinyVector3<double, DoubleUtils>>(m, "TinyVector3")
      .def(py::init<double, double, double>())
      .def("set_zero", &TinyVector3<double, DoubleUtils>::set_zero)
      .def_readwrite("x", &TinyVector3<double, DoubleUtils>::m_x)
      .def_readwrite("y", &TinyVector3<double, DoubleUtils>::m_y)
      .def_readwrite("z", &TinyVector3<double, DoubleUtils>::m_z)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(-py::self)
      .def("__repr__",
           [](const TinyVector3<double, DoubleUtils> &a) {
             return "[" + std::to_string(a.m_x) + " " + std::to_string(a.m_y) +
                    " " + std::to_string(a.m_z) + "]";
           })
      .def("__getitem__", [](const TinyVector3<double, DoubleUtils> &a,
                             int i) { return a[i]; })
      .def("__setitem__", [](TinyVector3<double, DoubleUtils> &a, int i,
                             double v) { a[i] = v; });

  py::class_<TinyGeometry<double, DoubleUtils>,
             std::unique_ptr<TinyGeometry<double, DoubleUtils>>>
      geom(m, "TinyGeometry");

  geom.def(py::init<int>())
      .def("get_type", &TinyGeometry<double, DoubleUtils>::get_type);

  py::class_<TinySphere<double, DoubleUtils>,
             std::unique_ptr<TinySphere<double, DoubleUtils>>>(m, "TinySphere",
                                                               geom)
      .def(py::init<double>())
      .def("get_radius", &TinySphere<double, DoubleUtils>::get_radius);

  py::class_<TinyPlane<double, DoubleUtils>,
             std::unique_ptr<TinyPlane<double, DoubleUtils>>>(m, "TinyPlane",
                                                              geom)
      .def(py::init<>())
      .def("get_normal", &TinyPlane<double, DoubleUtils>::get_normal);

  py::class_<TinyRigidBody<double, DoubleUtils>,
             std::unique_ptr<TinyRigidBody<double, DoubleUtils>>>(
      m, "TinyRigidBody")
      .def(py::init<double, TinyGeometry<double, DoubleUtils> *>(),
           py::return_value_policy::reference_internal)
      .def_readwrite("world_pose",
                     &TinyRigidBody<double, DoubleUtils>::m_world_pose)
      .def_readwrite("collision_geometry",
                     &TinyRigidBody<double, DoubleUtils>::m_geometry)
      .def_readwrite("linear_velocity",
                     &TinyRigidBody<double, DoubleUtils>::m_linear_velocity)
      .def_readwrite("angular_velocity",
                     &TinyRigidBody<double, DoubleUtils>::m_angular_velocity)
      .def_readwrite("local_inertia",
                     &TinyRigidBody<double, DoubleUtils>::m_local_inertia)
      .def_readwrite("total_force",
                     &TinyRigidBody<double, DoubleUtils>::m_total_force)
      .def_readwrite("total_torque",
                     &TinyRigidBody<double, DoubleUtils>::m_total_torque)
      .def_readwrite("user_index",
                     &TinyRigidBody<double, DoubleUtils>::m_user_index)

      .def("apply_gravity", &TinyRigidBody<double, DoubleUtils>::apply_gravity)
      .def("apply_force_impulse",
           &TinyRigidBody<double, DoubleUtils>::apply_force_impulse)

      .def("apply_central_force",
           &TinyRigidBody<double, DoubleUtils>::apply_central_force)
      .def("apply_impulse", &TinyRigidBody<double, DoubleUtils>::apply_impulse)
      .def("clear_forces", &TinyRigidBody<double, DoubleUtils>::clear_forces)
      .def("integrate", &TinyRigidBody<double, DoubleUtils>::integrate);

  py::class_<TinyMatrix3x3<double, DoubleUtils>>(m, "TinyMatrix3x3")
      .def(py::init<>())
      .def(py::init<TinyQuaternion<double, DoubleUtils>>())
      .def("get_row", &TinyMatrix3x3<double, DoubleUtils>::getRow)
      .def("set_identity", &TinyMatrix3x3<double, DoubleUtils>::set_identity);

  py::class_<TinyQuaternion<double, DoubleUtils>>(m, "TinyQuaternion")
      .def(py::init<double, double, double, double>())
      .def("set_identity", &TinyQuaternion<double, DoubleUtils>::set_identity)
      .def("get_euler_rpy", &TinyQuaternion<double, DoubleUtils>::get_euler_rpy)
      .def("get_euler_rpy2",
           &TinyQuaternion<double, DoubleUtils>::get_euler_rpy2)
      .def("set_euler_rpy", &TinyQuaternion<double, DoubleUtils>::set_euler_rpy)

      .def_readwrite("x", &TinyQuaternion<double, DoubleUtils>::m_x)
      .def_readwrite("y", &TinyQuaternion<double, DoubleUtils>::m_y)
      .def_readwrite("z", &TinyQuaternion<double, DoubleUtils>::m_z)
      .def_readwrite("w", &TinyQuaternion<double, DoubleUtils>::m_w)
      .def("__repr__",
           [](const TinyQuaternion<double, DoubleUtils> &q) {
             return "[" + std::to_string(q.m_x) + " " + std::to_string(q.m_y) +
                    " " + std::to_string(q.m_z) + " " + std::to_string(q.m_w) +
                    "]";
           })
      .def("__getitem__", [](const TinyQuaternion<double, DoubleUtils> &a,
                             int i) { return a[i]; })
      .def("__setitem__", [](TinyQuaternion<double, DoubleUtils> &a, int i,
                             double v) { a[i] = v; });

  py::class_<TinyPose<double, DoubleUtils>>(m, "TinyPose")
      .def(py::init<TinyVector3<double, DoubleUtils>,
                    TinyQuaternion<double, DoubleUtils>>())
      .def_readwrite("position", &TinyPose<double, DoubleUtils>::m_position)
      .def_readwrite("orientation",
                     &TinyPose<double, DoubleUtils>::m_orientation)
      .def("inverse_transform",
           &TinyPose<double, DoubleUtils>::inverse_transform);

  py::class_<TinySpatialTransform<double, DoubleUtils>>(m,
                                                        "TinySpatialTransform")
      .def(py::init<>())
      .def("set_identity",
           &TinySpatialTransform<double, DoubleUtils>::set_identity)
      .def_readwrite("translation",
                     &TinySpatialTransform<double, DoubleUtils>::m_translation)
      .def_readwrite("rotation",
                     &TinySpatialTransform<double, DoubleUtils>::m_rotation)
      .def(py::self * py::self)
      .def("get_inverse",
           &TinySpatialTransform<double, DoubleUtils>::get_inverse);

  py::class_<TinySpatialMotionVector<double, DoubleUtils>>(
      m, "TinySpatialMotionVector")
      .def(py::init<int>())
      .def_readwrite("topVec",
                     &TinySpatialMotionVector<double, DoubleUtils>::m_topVec)
      .def_readwrite(
          "bottomVec",
          &TinySpatialMotionVector<double, DoubleUtils>::m_bottomVec);

  py::class_<TinySymmetricSpatialDyad<double, DoubleUtils>>(
      m, "TinySymmetricSpatialDyad")
      .def(py::init<>())
      .def("set_identity",
           &TinySymmetricSpatialDyad<double, DoubleUtils>::setIdentity)
      .def("compute_inertia_dyad",
           &TinySymmetricSpatialDyad<double, DoubleUtils>::computeInertiaDyad)
      .def("mul", &TinySymmetricSpatialDyad<double, DoubleUtils>::mul)
      .def("shift", &TinySymmetricSpatialDyad<double, DoubleUtils>::shift)
      .def("inverse", &TinySymmetricSpatialDyad<double, DoubleUtils>::inverse)
      .def_readwrite(
          "topLeftMat",
          &TinySymmetricSpatialDyad<double, DoubleUtils>::m_topLeftMat)
      .def_readwrite(
          "topRightMat",
          &TinySymmetricSpatialDyad<double, DoubleUtils>::m_topRightMat)
      .def_readwrite(
          "bottomLeftMat",
          &TinySymmetricSpatialDyad<double, DoubleUtils>::m_bottomLeftMat)
      .def_readwrite(
          "bottomRightMat",
          &TinySymmetricSpatialDyad<double, DoubleUtils>::m_bottomRightMat)
      .def_readwrite(
          "center_of_mass",
          &TinySymmetricSpatialDyad<double, DoubleUtils>::m_center_of_mass)
      .def(py::self -= py::self);

  py::enum_<TinyJointType>(m, "TinyJointType")
      .value("JOINT_FIXED", JOINT_FIXED, "JOINT_FIXED")
      .value("JOINT_PRISMATIC_X", JOINT_PRISMATIC_X, "JOINT_PRISMATIC_X")
      .value("JOINT_PRISMATIC_Y", JOINT_PRISMATIC_Y, "JOINT_PRISMATIC_Y")
      .value("JOINT_PRISMATIC_Z", JOINT_PRISMATIC_Z, "JOINT_PRISMATIC_Z")
      .value("JOINT_PRISMATIC_AXIS", JOINT_PRISMATIC_AXIS,
             "JOINT_PRISMATIC_AXIS")
      .value("JOINT_REVOLUTE_X", JOINT_REVOLUTE_X, "JOINT_REVOLUTE_X")
      .value("JOINT_REVOLUTE_Y", JOINT_REVOLUTE_Y, "JOINT_REVOLUTE_Y")
      .value("JOINT_REVOLUTE_Z", JOINT_REVOLUTE_Z, "JOINT_REVOLUTE_Z")
      .value("JOINT_REVOLUTE_AXIS", JOINT_REVOLUTE_AXIS, "JOINT_REVOLUTE_AXIS")
      .value("JOINT_INVALID", JOINT_INVALID, "JOINT_INVALID")
      .export_values();

  py::class_<TinyLink<double, DoubleUtils>,
             std::unique_ptr<TinyLink<double, DoubleUtils>>>(m, "TinyLink")
      .def(py::init<TinyJointType, TinySpatialTransform<double, DoubleUtils> &,
                    const TinySymmetricSpatialDyad<double, DoubleUtils> &>())
      .def("jcalc", &TinyLink<double, DoubleUtils>::jcalc1)
      .def("set_joint_type", &TinyLink<double, DoubleUtils>::set_joint_type)
      .def_readwrite("stiffness", &TinyLink<double, DoubleUtils>::m_stiffness)
      .def_readwrite("joint_type", &TinyLink<double, DoubleUtils>::m_joint_type)
      .def_readwrite("damping", &TinyLink<double, DoubleUtils>::m_damping);

  py::class_<TinyMultiBody<double, DoubleUtils>,
             std::unique_ptr<TinyMultiBody<double, DoubleUtils>>>(
      m, "TinyMultiBody")
      .def(py::init<bool>())
      .def("initialize", &TinyMultiBody<double, DoubleUtils>::initialize)
      .def("set_base_position",
           &TinyMultiBody<double, DoubleUtils>::set_position)
      .def("get_world_transform",
           &TinyMultiBody<double, DoubleUtils>::get_world_transform)

      .def("attach_link", &TinyMultiBody<double, DoubleUtils>::attach_link)
      .def("forward_kinematics",
           &TinyMultiBody<double, DoubleUtils>::forward_kinematics1)
      .def("forward_dynamics",
           py::overload_cast<const TinyVector3<double, DoubleUtils> &>(
               &TinyMultiBody<double, DoubleUtils>::forward_dynamics))
      .def("integrate", py::overload_cast<double>(
                            &TinyMultiBody<double, DoubleUtils>::integrate))
      .def("integrate_q", &TinyMultiBody<double, DoubleUtils>::integrate_q)
      .def("body_to_world", &TinyMultiBody<double, DoubleUtils>::body_to_world)
      .def("world_to_body", &TinyMultiBody<double, DoubleUtils>::world_to_body)
      .def("point_jacobian",
           &TinyMultiBody<double, DoubleUtils>::point_jacobian1)
      .def("bias_forces", &TinyMultiBody<double, DoubleUtils>::bias_forces)
      .def_readwrite("q", &TinyMultiBody<double, DoubleUtils>::m_q)
      .def_readwrite("links", &TinyMultiBody<double, DoubleUtils>::m_links)
      .def_readwrite("qd", &TinyMultiBody<double, DoubleUtils>::m_qd)
      .def_readwrite("qdd", &TinyMultiBody<double, DoubleUtils>::m_qdd)
      .def_readwrite("tau", &TinyMultiBody<double, DoubleUtils>::m_tau);

  py::class_<TinyCollisionDispatcher<double, DoubleUtils>>(
      m, "TinyCollisionDispatcher")
      .def(py::init<>())
      .def("compute_contacts",
           &TinyCollisionDispatcher<double, DoubleUtils>::compute_contacts);

  py::class_<TinyContactPoint<double, DoubleUtils>> contact(m,
                                                            "TinyContactPoint");
  contact.def(py::init<>())
      .def_readwrite(
          "world_normal_on_b",
          &TinyContactPoint<double, DoubleUtils>::m_world_normal_on_b)
      .def_readwrite("world_point_on_a",
                     &TinyContactPoint<double, DoubleUtils>::m_world_point_on_a)
      .def_readwrite("world_point_on_b",
                     &TinyContactPoint<double, DoubleUtils>::m_world_point_on_b)
      .def_readwrite("distance",
                     &TinyContactPoint<double, DoubleUtils>::m_distance);

  py::class_<TinyContactPointRigidBody<double, DoubleUtils>>(
      m, "TinyContactPointRigidBody", contact)
      .def(py::init<>())
      .def_readwrite(
          "rigid_body_a",
          &TinyContactPointRigidBody<double, DoubleUtils>::m_rigid_body_a)
      .def_readwrite(
          "rigid_body_b",
          &TinyContactPointRigidBody<double, DoubleUtils>::m_rigid_body_b)
      .def_readwrite(
          "restitution",
          &TinyContactPointRigidBody<double, DoubleUtils>::m_restitution)
      .def_readwrite(
          "friction",
          &TinyContactPointRigidBody<double, DoubleUtils>::m_friction);

  py::class_<TinyContactPointMultiBody<double, DoubleUtils>>(
      m, "TinyContactPointMultiBody", contact)
      .def(py::init<>())
      .def_readwrite(
          "multi_body_a",
          &TinyContactPointMultiBody<double, DoubleUtils>::m_multi_body_a)
      .def_readwrite(
          "multi_body_b",
          &TinyContactPointMultiBody<double, DoubleUtils>::m_multi_body_b)
      .def_readwrite(
          "restitution",
          &TinyContactPointMultiBody<double, DoubleUtils>::m_restitution)
      .def_readwrite(
          "friction",
          &TinyContactPointMultiBody<double, DoubleUtils>::m_friction)
      .def_readwrite("link_a",
                     &TinyContactPointMultiBody<double, DoubleUtils>::m_link_a)
      .def_readwrite("link_b",
                     &TinyContactPointMultiBody<double, DoubleUtils>::m_link_b);

  py::class_<TinyConstraintSolver<double, DoubleUtils>>(m,
                                                        "TinyConstraintSolver")
      .def(py::init<>())
      .def("resolve_collision",
           &TinyConstraintSolver<double, DoubleUtils>::resolveCollision);

  py::class_<TinyMultiBodyConstraintSolver<double, DoubleUtils>>(
      m, "TinyMultiBodyConstraintSolver")
      .def(py::init<>())
      .def("resolve_collision",
           &TinyMultiBodyConstraintSolver<double,
                                          DoubleUtils>::resolveCollision);
  py::class_<TinyUrdfParser<double, DoubleUtils>>(m, "TinyUrdfParser")
      .def(py::init<>())
      .def("load_urdf", &TinyUrdfParser<double, DoubleUtils>::load_urdf);

  py::enum_<TinyVelocitySmoothingMethod>(m, "TinyVelocitySmoothingMethod",
                                         py::arithmetic())
      .value("SMOOTH_VEL_NONE", SMOOTH_VEL_NONE)
      .value("SMOOTH_VEL_SIGMOID", SMOOTH_VEL_SIGMOID)
      .value("SMOOTH_VEL_TANH", SMOOTH_VEL_TANH)
      .value("SMOOTH_VEL_ABS", SMOOTH_VEL_ABS)
      .export_values();

  typedef TinyMultiBodyConstraintSolverSpring<double, DoubleUtils> TMBCSS;
  py::class_<TMBCSS>(m, "TinyMultiBodyConstraintSolverSpring")
      .def(py::init<>())
      .def("resolve_collision", &TMBCSS::resolveCollision)
      .def_readwrite("spring_k", &TMBCSS::spring_k)
      .def_readwrite("damper_d", &TMBCSS::damper_d)
      .def_readwrite("hard_contact_condition", &TMBCSS::hard_contact_condition)
      .def_readwrite("exponent_n", &TMBCSS::exponent_n)
      .def_readwrite("exponent_n_air", &TMBCSS::exponent_n_air)
      .def_readwrite("exponent_vel_air", &TMBCSS::exponent_vel_air)
      .def_readwrite("smoothing_method", &TMBCSS::smoothing_method)
      .def_readwrite("smooth_alpha_vel", &TMBCSS::smooth_alpha_vel)
      .def_readwrite("smooth_alpha_normal", &TMBCSS::smooth_alpha_normal)
      .def_readwrite("mu_static", &TMBCSS::mu_static)
      .def_readwrite("andersson_vs", &TMBCSS::andersson_vs)
      .def_readwrite("andersson_p", &TMBCSS::andersson_p)
      .def_readwrite("andersson_ktanh", &TMBCSS::andersson_ktanh)
      .def_readwrite("v_transition", &TMBCSS::v_transition)
      .def_readwrite("friction_model", &TMBCSS::friction_model)
      .def("compute_contact_force", &TMBCSS::compute_contact_force)
      .def("compute_friction_force", &TMBCSS::compute_friction_force);

  py::class_<TinyWorld<double, DoubleUtils>>(m, "TinyWorld")
      .def(py::init<>())
      .def("step", &TinyWorld<double, DoubleUtils>::step)
      .def_property("gravity", &TinyWorld<double, DoubleUtils>::get_gravity,
                    &TinyWorld<double, DoubleUtils>::set_gravity)
      .def("compute_contacts_rigid_body",
           &TinyWorld<double, DoubleUtils>::compute_contacts_rigid_body)
      .def("compute_contacts_multi_body",
           &TinyWorld<double, DoubleUtils>::compute_contacts_multi_body)
      .def("get_collision_dispatcher",
           &TinyWorld<double, DoubleUtils>::get_collision_dispatcher)
      .def_readwrite("friction",
                     &TinyWorld<double, DoubleUtils>::default_friction)
      .def_readwrite("restitution",
                     &TinyWorld<double, DoubleUtils>::default_restitution);

  py::class_<TinyRaycastResult<double, DoubleUtils>>(m, "TinyRaycastResult")
      .def(py::init<>())
      .def_readwrite("hit_fraction",
                     &TinyRaycastResult<double, DoubleUtils>::m_hit_fraction)
      .def_readwrite("collider_index",
                     &TinyRaycastResult<double, DoubleUtils>::m_collider_index);

  py::class_<TinyRaycast<double, DoubleUtils>>(m, "TinyRaycast")
      .def(py::init<>())
      .def("cast_rays", &TinyRaycast<double, DoubleUtils>::cast_rays)
      .def("volume", &TinyRaycast<double, DoubleUtils>::volume)
      .def("intersection_volume",
           &TinyRaycast<double, DoubleUtils>::intersection_volume);

  ///////////////////////////////////////////////////////////////////////////////////////////

  py::class_<Fix64Scalar>(m, "Fix64Scalar")
      .def(py::init<>())
      .def("zero", &Fix64Scalar::zero)
      .def("fraction", &Fix64Scalar::fraction_internal)
      .def("getScalar", &Fix64Scalar::getScalar)
      .def("sqrt", &Fix64Scalar::sqrt1)
      .def("sin", &Fix64Scalar::sin1)
      .def("cos", &Fix64Scalar::cos1)
      .def("abs", &Fix64Scalar::Fix64Abs)
      .def("max", &Fix64Scalar::Fix64Max)
      .def("half", &Fix64Scalar::half)
      .def("pi", &Fix64Scalar::pi)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self * py::self)
      .def(py::self / py::self)
      .def(py::self < py::self)
      .def(py::self == py::self)
      .def("__repr__",
           [](const Fix64Scalar &a) { return std::to_string(a.getScalar()); });
  py::class_<TinyVector3<Fix64Scalar, Fix64Scalar>>(m, "Fix64Vector3")
      .def(py::init<Fix64Scalar, Fix64Scalar, Fix64Scalar>())
      .def("set_zero", &TinyVector3<Fix64Scalar, Fix64Scalar>::set_zero)
      .def_readwrite("x", &TinyVector3<Fix64Scalar, Fix64Scalar>::m_x)
      .def_readwrite("y", &TinyVector3<Fix64Scalar, Fix64Scalar>::m_y)
      .def_readwrite("z", &TinyVector3<Fix64Scalar, Fix64Scalar>::m_z)
      .def(py::self + py::self)
      .def(py::self - py::self)

      .def("__repr__", [](const TinyVector3<Fix64Scalar, Fix64Scalar> &a) {
        double x = Fix64Scalar::getDouble(a.m_x);
        double y = Fix64Scalar::getDouble(a.m_y);
        double z = Fix64Scalar::getDouble(a.m_z);

        return "[" + std::to_string(x) + "," + std::to_string(y) + "," +
               std::to_string(z) + "]";
      });

  py::class_<TinyUrdfInertial<double, DoubleUtils>>(m, "TinyUrdfInertial")
      .def(py::init<>())
      .def_readwrite("mass", &TinyUrdfInertial<double, DoubleUtils>::mass)
      .def_readwrite("inertia_xxyyzz",
                     &TinyUrdfInertial<double, DoubleUtils>::inertia_xxyyzz)
      .def_readwrite("origin_xyz",
                     &TinyUrdfInertial<double, DoubleUtils>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &TinyUrdfInertial<double, DoubleUtils>::origin_rpy);

  py::class_<TinyUrdfCollisionSphere<double, DoubleUtils>>(
      m, "TinyUrdfCollisionSphere")
      .def(py::init<>())
      .def_readwrite("radius",
                     &TinyUrdfCollisionSphere<double, DoubleUtils>::m_radius);

  py::class_<TinyUrdfCollisionBox<double, DoubleUtils>>(m,
                                                        "TinyUrdfCollisionBox")
      .def(py::init<>())
      .def_readwrite("extents",
                     &TinyUrdfCollisionBox<double, DoubleUtils>::m_extents);

  py::class_<TinyUrdfCollisionCapsule<double, DoubleUtils>>(
      m, "TinyUrdfCollisionCapsule")
      .def(py::init<>())
      .def_readwrite("radius",
                     &TinyUrdfCollisionCapsule<double, DoubleUtils>::m_radius)
      .def_readwrite("length",
                     &TinyUrdfCollisionCapsule<double, DoubleUtils>::m_length);

  py::class_<TinyUrdfCollisionPlane<double, DoubleUtils>>(
      m, "TinyUrdfCollisionPlane")
      .def(py::init<>())
      .def_readwrite("constant",
                     &TinyUrdfCollisionPlane<double, DoubleUtils>::m_constant)
      .def_readwrite("normal",
                     &TinyUrdfCollisionPlane<double, DoubleUtils>::m_normal);

  py::class_<TinyUrdfCollisionMesh<double, DoubleUtils>>(
      m, "TinyUrdfCollisionMesh")
      .def(py::init<>())
      .def_readwrite("file_name",
                     &TinyUrdfCollisionMesh<double, DoubleUtils>::m_file_name)
      .def_readwrite("scale",
                     &TinyUrdfCollisionMesh<double, DoubleUtils>::m_scale);

  py::class_<TinyUrdfGeometry<double, DoubleUtils>>(m, "TinyUrdfGeometry")
      .def(py::init<>())
      .def_readwrite("geom_type",
                     &TinyUrdfGeometry<double, DoubleUtils>::geom_type)
      .def_readwrite("sphere", &TinyUrdfGeometry<double, DoubleUtils>::m_sphere)
      .def_readwrite("box", &TinyUrdfGeometry<double, DoubleUtils>::m_box)
      .def_readwrite("plane", &TinyUrdfGeometry<double, DoubleUtils>::m_plane)
      .def_readwrite("capsule",
                     &TinyUrdfGeometry<double, DoubleUtils>::m_capsule)
      .def_readwrite("mesh", &TinyUrdfGeometry<double, DoubleUtils>::m_mesh);

  py::class_<TinyUrdfCollision<double, DoubleUtils>>(m, "TinyUrdfCollision")
      .def(py::init<>())

      .def_readwrite("origin_xyz",
                     &TinyUrdfCollision<double, DoubleUtils>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &TinyUrdfCollision<double, DoubleUtils>::origin_rpy)
      .def_readwrite("geometry",
                     &TinyUrdfCollision<double, DoubleUtils>::geometry);

  py::class_<TinyUrdfVisual<double, DoubleUtils>>(m, "TinyUrdfVisual")
      .def(py::init<>())
      .def_readwrite("origin_xyz",
                     &TinyUrdfVisual<double, DoubleUtils>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &TinyUrdfVisual<double, DoubleUtils>::origin_rpy)
      .def_readwrite("geometry", &TinyUrdfVisual<double, DoubleUtils>::geometry)
      .def_readwrite(
          "sync_visual_body_uid1",
          &TinyUrdfVisual<double, DoubleUtils>::sync_visual_body_uid1)
      .def_readwrite(
          "sync_visual_body_uid2",
          &TinyUrdfVisual<double, DoubleUtils>::sync_visual_body_uid2);

  py::class_<TinyUrdfJoint<double, DoubleUtils>>(m, "TinyUrdfJoint")
      .def(py::init<>())

      .def_readwrite("joint_name",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_name)
      .def_readwrite("joint_type",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_type)
      .def_readwrite("joint_lower_limit",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_lower_limit)
      .def_readwrite("joint_upper_limit",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_upper_limit)
      .def_readwrite("parent_name",
                     &TinyUrdfJoint<double, DoubleUtils>::parent_name)
      .def_readwrite("child_name",
                     &TinyUrdfJoint<double, DoubleUtils>::child_name)
      .def_readwrite("joint_origin_xyz",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_origin_xyz)
      .def_readwrite("joint_origin_rpy",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_origin_rpy)
      .def_readwrite("joint_axis_xyz",
                     &TinyUrdfJoint<double, DoubleUtils>::joint_axis_xyz);

  py::class_<TinyUrdfLink<double, DoubleUtils>>(m, "TinyUrdfLink")
      .def(py::init<>())
      .def_readwrite("link_name", &TinyUrdfLink<double, DoubleUtils>::link_name)
      .def_readwrite("urdf_inertial",
                     &TinyUrdfLink<double, DoubleUtils>::urdf_inertial)
      .def_readwrite("urdf_visual_shapes",
                     &TinyUrdfLink<double, DoubleUtils>::urdf_visual_shapes)
      .def_readwrite("urdf_collision_shapes",
                     &TinyUrdfLink<double, DoubleUtils>::urdf_collision_shapes)
      .def_readwrite("parent_index",
                     &TinyUrdfLink<double, DoubleUtils>::m_parent_index);

  py::class_<TinyUrdfStructures<double, DoubleUtils>>(m, "TinyUrdfStructures")
      .def(py::init<>())
      .def_readwrite("robot_name",
                     &TinyUrdfStructures<double, DoubleUtils>::m_robot_name)
      .def_readwrite("base_links",
                     &TinyUrdfStructures<double, DoubleUtils>::m_base_links)
      .def_readwrite("links", &TinyUrdfStructures<double, DoubleUtils>::m_links)
      .def_readwrite("joints",
                     &TinyUrdfStructures<double, DoubleUtils>::m_joints)
      .def_readwrite(
          "name_to_link_index",
          &TinyUrdfStructures<double, DoubleUtils>::m_name_to_link_index);

  py::class_<UrdfToMultiBody2<double, DoubleUtils>>(m, "UrdfToMultiBody2")
      .def(py::init<>())
      .def("convert2", &UrdfToMultiBody2<double, DoubleUtils>::convert);

  py::class_<Motion>(m, "Motion")
      .def(py::init([](const std::string &filename) {
        Motion motion;
        Motion::load_from_file(filename, &motion);
        return motion;
      }))
      .def_readwrite("frames", &Motion::frames)
      .def_readonly("frame_duration", &Motion::frame_duration)
      .def_property_readonly("total_duration", &Motion::total_duration)
      .def("calculate_frame", &Motion::calculate_frame);

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
  m.attr("SPHERE_TYPE") = py::int_(int(TINY_SPHERE_TYPE));
  m.attr("BOX_TYPE") = py::int_(int(TINY_BOX_TYPE));
  m.attr("PLANE_TYPE") = py::int_(int(TINY_PLANE_TYPE));
  m.attr("CAPSULE_TYPE") = py::int_(int(TINY_CAPSULE_TYPE));
  m.attr("MESH_TYPE") = py::int_(int(TINY_MESH_TYPE));

  m.attr("JOINT_FIXED") = py::int_(int(JOINT_FIXED));
  m.attr("JOINT_PRISMATIC_X") = py::int_(int(JOINT_PRISMATIC_X));
  m.attr("JOINT_PRISMATIC_Y") = py::int_(int(JOINT_PRISMATIC_Y));
  m.attr("JOINT_PRISMATIC_Z") = py::int_(int(JOINT_PRISMATIC_Z));
  m.attr("JOINT_PRISMATIC_AXIS") = py::int_(int(JOINT_PRISMATIC_AXIS));
  m.attr("JOINT_REVOLUTE_X") = py::int_(int(JOINT_REVOLUTE_X));
  m.attr("JOINT_REVOLUTE_Y") = py::int_(int(JOINT_REVOLUTE_Y));
  m.attr("JOINT_REVOLUTE_Z") = py::int_(int(JOINT_REVOLUTE_Z));
  m.attr("JOINT_REVOLUTE_AXIS") = py::int_(int(JOINT_REVOLUTE_AXIS));
}
