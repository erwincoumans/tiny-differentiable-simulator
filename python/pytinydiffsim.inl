


  m.doc() = R"pbdoc(
        tiny differentiable physics python plugin
        -----------------------

        .. currentmodule:: pytinydiffsim

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  

  py::class_<TinyVector3<MyScalar, MyTinyConstants>>(m, "TinyVector3")
      .def(py::init<MyScalar, MyScalar, MyScalar>())
      .def("set_zero", &TinyVector3<MyScalar, MyTinyConstants>::set_zero)
      .def_readwrite("x", &TinyVector3<MyScalar, MyTinyConstants>::m_x)
      .def_readwrite("y", &TinyVector3<MyScalar, MyTinyConstants>::m_y)
      .def_readwrite("z", &TinyVector3<MyScalar, MyTinyConstants>::m_z)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(-py::self)
      .def("__repr__",
           [](const TinyVector3<MyScalar, MyTinyConstants> &a) {
             return "[" + std::to_string(MyTinyConstants::getDouble(a.m_x)) + " " + std::to_string(MyTinyConstants::getDouble(a.m_y)) +
                    " " + std::to_string(MyTinyConstants::getDouble(a.m_z)) + "]";
           })
      .def("__getitem__", [](const TinyVector3<MyScalar, MyTinyConstants> &a,
                             int i) { return a[i]; })
      .def("__setitem__", [](TinyVector3<MyScalar, MyTinyConstants> &a, int i,
          MyScalar v) { a[i] = v; });


  py::class_<TinyGeometry<MyScalar, MyTinyConstants>,
             std::unique_ptr<TinyGeometry<MyScalar, MyTinyConstants>>>
      geom(m, "TinyGeometry");

  geom.def(py::init<int>())
      .def("get_type", &TinyGeometry<MyScalar, MyTinyConstants>::get_type);

  py::class_<TinySphere<MyScalar, MyTinyConstants>,
             std::unique_ptr<TinySphere<MyScalar, MyTinyConstants>>>(m, "TinySphere",
                                                               geom)
      .def(py::init<MyScalar>())
      .def("get_radius", &TinySphere<MyScalar, MyTinyConstants>::get_radius);

  py::class_<TinyPlane<MyScalar, MyTinyConstants>,
             std::unique_ptr<TinyPlane<MyScalar, MyTinyConstants>>>(m, "TinyPlane",
                                                              geom)
      .def(py::init<>())
      .def("get_normal", &TinyPlane<MyScalar, MyTinyConstants>::get_normal);


  py::class_<TinyRigidBody<MyScalar, MyTinyConstants>,
             std::unique_ptr<TinyRigidBody<MyScalar, MyTinyConstants>>>(
      m, "TinyRigidBody")
      .def(py::init<MyScalar, TinyGeometry<MyScalar, MyTinyConstants> *>(),
           py::return_value_policy::reference_internal)
      .def_readwrite("world_pose",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_world_pose)
      .def_readwrite("collision_geometry",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_geometry)
      .def_readwrite("linear_velocity",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_linear_velocity)
      .def_readwrite("angular_velocity",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_angular_velocity)
      .def_readwrite("local_inertia",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_local_inertia)
      .def_readwrite("total_force",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_total_force)
      .def_readwrite("total_torque",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_total_torque)
      .def_readwrite("user_index",
                     &TinyRigidBody<MyScalar, MyTinyConstants>::m_user_index)

      .def("apply_gravity", &TinyRigidBody<MyScalar, MyTinyConstants>::apply_gravity)
      .def("apply_force_impulse",
           &TinyRigidBody<MyScalar, MyTinyConstants>::apply_force_impulse)

      .def("apply_central_force",
           &TinyRigidBody<MyScalar, MyTinyConstants>::apply_central_force)
      .def("apply_impulse", &TinyRigidBody<MyScalar, MyTinyConstants>::apply_impulse)
      .def("clear_forces", &TinyRigidBody<MyScalar, MyTinyConstants>::clear_forces)
      .def("integrate", &TinyRigidBody<MyScalar, MyTinyConstants>::integrate);



  py::class_<TinyMatrixXxX<MyScalar, MyTinyConstants>>(m, "TinyMatrixXxX")
      .def(py::init<int, int>())
      .def("inversed", &TinyMatrixXxX<MyScalar, MyTinyConstants>::inversed)
      .def("set_zero", &TinyMatrixXxX<MyScalar, MyTinyConstants>::set_zero)
      .def("print", &TinyMatrixXxX<MyScalar, MyTinyConstants>::print)
      .def("get_at", &TinyMatrixXxX<MyScalar, MyTinyConstants>::get_at)
      .def("__getitem__", [](const TinyMatrixXxX<MyScalar, MyTinyConstants>& a,
          int row, int col) { return a(row,col); })
      .def_readonly("num_rows", &TinyMatrixXxX<MyScalar, MyTinyConstants>::m_rows)
      .def_readonly("num_columns", &TinyMatrixXxX<MyScalar, MyTinyConstants>::m_cols)
      
      ;

  py::class_<TinyMatrix3x3<MyScalar, MyTinyConstants>>(m, "TinyMatrix3x3")
      .def(py::init<>())
      .def(py::init<TinyQuaternion<MyScalar, MyTinyConstants>>())
      .def("get_at", &TinyMatrix3x3<MyScalar, MyTinyConstants>::get_at)
      .def("get_row",&TinyMatrix3x3<MyScalar, MyTinyConstants>::getRow)
      .def("set_identity", &TinyMatrix3x3<MyScalar, MyTinyConstants>::set_identity);

  py::class_<TinyMatrix3xX<MyScalar, MyTinyConstants>>(m, "TinyMatrix3xX")
      .def(py::init<>())
      .def_readonly("num_rows", &TinyMatrix3xX<MyScalar, MyTinyConstants>::m_rows)
      .def_readonly("num_columns", &TinyMatrix3xX<MyScalar, MyTinyConstants>::m_cols)
      .def("print", &TinyMatrix3xX<MyScalar, MyTinyConstants>::print)
      .def("get_at", &TinyMatrix3xX<MyScalar, MyTinyConstants>::get_at);

  py::class_<TinyQuaternion<MyScalar, MyTinyConstants>>(m, "TinyQuaternion")
      .def(py::init<MyScalar, MyScalar, MyScalar, MyScalar>())
      .def("set_identity", &TinyQuaternion<MyScalar, MyTinyConstants>::set_identity)
      .def("get_euler_rpy", &TinyQuaternion<MyScalar, MyTinyConstants>::get_euler_rpy)
      .def("get_euler_rpy2",
           &TinyQuaternion<MyScalar, MyTinyConstants>::get_euler_rpy2)
      .def("set_euler_rpy", &TinyQuaternion<MyScalar, MyTinyConstants>::set_euler_rpy)

      .def_readwrite("x", &TinyQuaternion<MyScalar, MyTinyConstants>::m_x)
      .def_readwrite("y", &TinyQuaternion<MyScalar, MyTinyConstants>::m_y)
      .def_readwrite("z", &TinyQuaternion<MyScalar, MyTinyConstants>::m_z)
      .def_readwrite("w", &TinyQuaternion<MyScalar, MyTinyConstants>::m_w)
      .def("__repr__",
           [](const TinyQuaternion<MyScalar, MyTinyConstants> &q) {
             return "[" + std::to_string(MyTinyConstants::getDouble(q.m_x)) + 
                    " " + std::to_string(MyTinyConstants::getDouble(q.m_y)) +
                    " " + std::to_string(MyTinyConstants::getDouble(q.m_z)) + 
                    " " + std::to_string(MyTinyConstants::getDouble(q.m_w)) + "]";
           })
      .def("__getitem__", [](const TinyQuaternion<MyScalar, MyTinyConstants> &a,
                             int i) { return a[i]; })
      .def("__setitem__", [](TinyQuaternion<MyScalar, MyTinyConstants> &a, int i,
                             MyScalar v) { a[i] = v; });

  py::class_<TinyPose<MyScalar, MyTinyConstants>>(m, "TinyPose")
      .def(py::init<TinyVector3<MyScalar, MyTinyConstants>,
                    TinyQuaternion<MyScalar, MyTinyConstants>>())
      .def_readwrite("position", &TinyPose<MyScalar, MyTinyConstants>::m_position)
      .def_readwrite("orientation",
                     &TinyPose<MyScalar, MyTinyConstants>::m_orientation)
      .def("inverse_transform",
           &TinyPose<MyScalar, MyTinyConstants>::inverse_transform);

  py::class_<TinySpatialTransform<MyScalar, MyTinyConstants>>(m,
                                                        "TinySpatialTransform")
      .def(py::init<>())
      .def("set_identity",
           &TinySpatialTransform<MyScalar, MyTinyConstants>::set_identity)
      .def_readwrite("translation",
                     &TinySpatialTransform<MyScalar, MyTinyConstants>::m_translation)
      .def_readwrite("rotation",
                     &TinySpatialTransform<MyScalar, MyTinyConstants>::m_rotation)
      .def(py::self * py::self)
      .def("get_inverse",
           &TinySpatialTransform<MyScalar, MyTinyConstants>::get_inverse);

  py::class_<TinySpatialMotionVector<MyScalar, MyTinyConstants>>(
      m, "TinySpatialMotionVector")
      .def(py::init<int>())
      .def_readwrite("topVec",
                     &TinySpatialMotionVector<MyScalar, MyTinyConstants>::m_topVec)
      .def_readwrite(
          "bottomVec",
          &TinySpatialMotionVector<MyScalar, MyTinyConstants>::m_bottomVec);

  py::class_<TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>>(
      m, "TinySymmetricSpatialDyad")
      .def(py::init<>())
      .def("set_identity",
           &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::setIdentity)
      .def("compute_inertia_dyad",
           &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::computeInertiaDyad)
      .def("mul", &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::mul)
      .def("shift", &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::shift)
      .def("inverse", &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::inverse)
      .def_readwrite(
          "topLeftMat",
          &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::m_topLeftMat)
      .def_readwrite(
          "topRightMat",
          &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::m_topRightMat)
      .def_readwrite(
          "bottomLeftMat",
          &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::m_bottomLeftMat)
      .def_readwrite(
          "bottomRightMat",
          &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::m_bottomRightMat)
      .def_readwrite(
          "center_of_mass",
          &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::m_center_of_mass)
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

  py::class_<TinyLink<MyScalar, MyTinyConstants>,
             std::unique_ptr<TinyLink<MyScalar, MyTinyConstants>>>(m, "TinyLink")
      .def(py::init<TinyJointType, TinySpatialTransform<MyScalar, MyTinyConstants> &,
                    const TinySymmetricSpatialDyad<MyScalar, MyTinyConstants> &>())
      .def("jcalc", &TinyLink<MyScalar, MyTinyConstants>::jcalc1)
      .def("set_joint_type", &TinyLink<MyScalar, MyTinyConstants>::set_joint_type)
      .def_readwrite("stiffness", &TinyLink<MyScalar, MyTinyConstants>::m_stiffness)
      .def_readwrite("joint_type", &TinyLink<MyScalar, MyTinyConstants>::m_joint_type)
      .def_readwrite("damping", &TinyLink<MyScalar, MyTinyConstants>::m_damping);

  py::class_<TinyMultiBody<MyScalar, MyTinyConstants>,
             std::unique_ptr<TinyMultiBody<MyScalar, MyTinyConstants>>>(
      m, "TinyMultiBody")
      .def(py::init<bool>())
      .def("initialize", &TinyMultiBody<MyScalar, MyTinyConstants>::initialize)
      .def("set_base_position",
           &TinyMultiBody<MyScalar, MyTinyConstants>::set_position)
      .def("get_world_transform",
           &TinyMultiBody<MyScalar, MyTinyConstants>::get_world_transform)
      .def("mass_matrix", &TinyMultiBody<MyScalar, MyTinyConstants>::mass_matrix1)
      .def("attach_link", &TinyMultiBody<MyScalar, MyTinyConstants>::attach_link)
      .def("forward_kinematics",
           &TinyMultiBody<MyScalar, MyTinyConstants>::forward_kinematics1)
      .def("forward_dynamics",
           py::overload_cast<const TinyVector3<MyScalar, MyTinyConstants> &>(
               &TinyMultiBody<MyScalar, MyTinyConstants>::forward_dynamics))
      .def("integrate", py::overload_cast<MyScalar>(
                            &TinyMultiBody<MyScalar, MyTinyConstants>::integrate))
      .def("integrate_q", &TinyMultiBody<MyScalar, MyTinyConstants>::integrate_q)
      .def("body_to_world", &TinyMultiBody<MyScalar, MyTinyConstants>::body_to_world)
      .def("world_to_body", &TinyMultiBody<MyScalar, MyTinyConstants>::world_to_body)
      .def("point_jacobian",
           &TinyMultiBody<MyScalar, MyTinyConstants>::point_jacobian1)
      .def("bias_forces", &TinyMultiBody<MyScalar, MyTinyConstants>::bias_forces)
      .def_property_readonly("num_dofs", &TinyMultiBody<MyScalar, MyTinyConstants>::dof)
      .def_property_readonly("num_dofs_qd", &TinyMultiBody<MyScalar, MyTinyConstants>::dof_qd)
      .def_readwrite("q", &TinyMultiBody<MyScalar, MyTinyConstants>::m_q)
      .def_readwrite("links", &TinyMultiBody<MyScalar, MyTinyConstants>::m_links)
      .def_readwrite("qd", &TinyMultiBody<MyScalar, MyTinyConstants>::m_qd)
      .def_readwrite("qdd", &TinyMultiBody<MyScalar, MyTinyConstants>::m_qdd)
      .def_readwrite("tau", &TinyMultiBody<MyScalar, MyTinyConstants>::m_tau);

  py::class_<TinyCollisionDispatcher<MyScalar, MyTinyConstants>>(
      m, "TinyCollisionDispatcher")
      .def(py::init<>())
      .def("compute_contacts",
           &TinyCollisionDispatcher<MyScalar, MyTinyConstants>::compute_contacts);

  py::class_<TinyContactPoint<MyScalar, MyTinyConstants>> contact(m,
                                                            "TinyContactPoint");
  contact.def(py::init<>())
      .def_readwrite(
          "world_normal_on_b",
          &TinyContactPoint<MyScalar, MyTinyConstants>::m_world_normal_on_b)
      .def_readwrite("world_point_on_a",
                     &TinyContactPoint<MyScalar, MyTinyConstants>::m_world_point_on_a)
      .def_readwrite("world_point_on_b",
                     &TinyContactPoint<MyScalar, MyTinyConstants>::m_world_point_on_b)
      .def_readwrite("distance",
                     &TinyContactPoint<MyScalar, MyTinyConstants>::m_distance);

  py::class_<TinyContactPointRigidBody<MyScalar, MyTinyConstants>>(
      m, "TinyContactPointRigidBody", contact)
      .def(py::init<>())
      .def_readwrite(
          "rigid_body_a",
          &TinyContactPointRigidBody<MyScalar, MyTinyConstants>::m_rigid_body_a)
      .def_readwrite(
          "rigid_body_b",
          &TinyContactPointRigidBody<MyScalar, MyTinyConstants>::m_rigid_body_b)
      .def_readwrite(
          "restitution",
          &TinyContactPointRigidBody<MyScalar, MyTinyConstants>::m_restitution)
      .def_readwrite(
          "friction",
          &TinyContactPointRigidBody<MyScalar, MyTinyConstants>::m_friction);

  py::class_<TinyContactPointMultiBody<MyScalar, MyTinyConstants>>(
      m, "TinyContactPointMultiBody", contact)
      .def(py::init<>())
      .def_readwrite(
          "multi_body_a",
          &TinyContactPointMultiBody<MyScalar, MyTinyConstants>::m_multi_body_a)
      .def_readwrite(
          "multi_body_b",
          &TinyContactPointMultiBody<MyScalar, MyTinyConstants>::m_multi_body_b)
      .def_readwrite(
          "restitution",
          &TinyContactPointMultiBody<MyScalar, MyTinyConstants>::m_restitution)
      .def_readwrite(
          "friction",
          &TinyContactPointMultiBody<MyScalar, MyTinyConstants>::m_friction)
      .def_readwrite("link_a",
                     &TinyContactPointMultiBody<MyScalar, MyTinyConstants>::m_link_a)
      .def_readwrite("link_b",
                     &TinyContactPointMultiBody<MyScalar, MyTinyConstants>::m_link_b);

  py::class_<TinyConstraintSolver<MyScalar, MyTinyConstants>>(m,
                                                        "TinyConstraintSolver")
      .def(py::init<>())
      .def("resolve_collision",
           &TinyConstraintSolver<MyScalar, MyTinyConstants>::resolveCollision);

  py::class_<TinyMultiBodyConstraintSolver<MyScalar, MyTinyConstants>>(
      m, "TinyMultiBodyConstraintSolver")
      .def(py::init<>())
      .def("resolve_collision",
           &TinyMultiBodyConstraintSolver<MyScalar,
                                          MyTinyConstants>::resolveCollision);
  py::class_<TinyUrdfParser<MyScalar, MyTinyConstants>>(m, "TinyUrdfParser")
      .def(py::init<>())
      .def("load_urdf", &TinyUrdfParser<MyScalar, MyTinyConstants>::load_urdf);

  py::enum_<TinyVelocitySmoothingMethod>(m, "TinyVelocitySmoothingMethod",
                                         py::arithmetic())
      .value("SMOOTH_VEL_NONE", SMOOTH_VEL_NONE)
      .value("SMOOTH_VEL_SIGMOID", SMOOTH_VEL_SIGMOID)
      .value("SMOOTH_VEL_TANH", SMOOTH_VEL_TANH)
      .value("SMOOTH_VEL_ABS", SMOOTH_VEL_ABS)
      .export_values();

  typedef TinyMultiBodyConstraintSolverSpring<MyScalar, MyTinyConstants> TMBCSS;
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

  py::class_<TinyWorld<MyScalar, MyTinyConstants>>(m, "TinyWorld")
      .def(py::init<>())
      .def("step", &TinyWorld<MyScalar, MyTinyConstants>::step)
      .def_property("gravity", &TinyWorld<MyScalar, MyTinyConstants>::get_gravity,
                    &TinyWorld<MyScalar, MyTinyConstants>::set_gravity)
      .def("compute_contacts_rigid_body",
           &TinyWorld<MyScalar, MyTinyConstants>::compute_contacts_rigid_body)
      .def("compute_contacts_multi_body",
           &TinyWorld<MyScalar, MyTinyConstants>::compute_contacts_multi_body)
      .def("get_collision_dispatcher",
           &TinyWorld<MyScalar, MyTinyConstants>::get_collision_dispatcher)
      .def_readwrite("friction",
                     &TinyWorld<MyScalar, MyTinyConstants>::default_friction)
      .def_readwrite("restitution",
                     &TinyWorld<MyScalar, MyTinyConstants>::default_restitution);

  py::class_<TinyRaycastResult<MyScalar, MyTinyConstants>>(m, "TinyRaycastResult")
      .def(py::init<>())
      .def_readwrite("hit_fraction",
                     &TinyRaycastResult<MyScalar, MyTinyConstants>::m_hit_fraction)
      .def_readwrite("collider_index",
                     &TinyRaycastResult<MyScalar, MyTinyConstants>::m_collider_index);

  py::class_<TinyRaycast<MyScalar, MyTinyConstants>>(m, "TinyRaycast")
      .def(py::init<>())
      .def("cast_rays", &TinyRaycast<MyScalar, MyTinyConstants>::cast_rays)
      .def("volume", &TinyRaycast<MyScalar, MyTinyConstants>::volume)
      .def("intersection_volume",
           &TinyRaycast<MyScalar, MyTinyConstants>::intersection_volume);

  ///////////////////////////////////////////////////////////////////////////////////////////


  py::class_<TinyUrdfInertial<MyScalar, MyTinyConstants>>(m, "TinyUrdfInertial")
      .def(py::init<>())
      .def_readwrite("mass", &TinyUrdfInertial<MyScalar, MyTinyConstants>::mass)
      .def_readwrite("inertia_xxyyzz",
                     &TinyUrdfInertial<MyScalar, MyTinyConstants>::inertia_xxyyzz)
      .def_readwrite("origin_xyz",
                     &TinyUrdfInertial<MyScalar, MyTinyConstants>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &TinyUrdfInertial<MyScalar, MyTinyConstants>::origin_rpy);

  py::class_<TinyUrdfCollisionSphere<MyScalar, MyTinyConstants>>(
      m, "TinyUrdfCollisionSphere")
      .def(py::init<>())
      .def_readwrite("radius",
                     &TinyUrdfCollisionSphere<MyScalar, MyTinyConstants>::m_radius);

  py::class_<TinyUrdfCollisionBox<MyScalar, MyTinyConstants>>(m,
                                                        "TinyUrdfCollisionBox")
      .def(py::init<>())
      .def_readwrite("extents",
                     &TinyUrdfCollisionBox<MyScalar, MyTinyConstants>::m_extents);

  py::class_<TinyUrdfCollisionCapsule<MyScalar, MyTinyConstants>>(
      m, "TinyUrdfCollisionCapsule")
      .def(py::init<>())
      .def_readwrite("radius",
                     &TinyUrdfCollisionCapsule<MyScalar, MyTinyConstants>::m_radius)
      .def_readwrite("length",
                     &TinyUrdfCollisionCapsule<MyScalar, MyTinyConstants>::m_length);

  py::class_<TinyUrdfCollisionPlane<MyScalar, MyTinyConstants>>(
      m, "TinyUrdfCollisionPlane")
      .def(py::init<>())
      .def_readwrite("constant",
                     &TinyUrdfCollisionPlane<MyScalar, MyTinyConstants>::m_constant)
      .def_readwrite("normal",
                     &TinyUrdfCollisionPlane<MyScalar, MyTinyConstants>::m_normal);

  py::class_<TinyUrdfCollisionMesh<MyScalar, MyTinyConstants>>(
      m, "TinyUrdfCollisionMesh")
      .def(py::init<>())
      .def_readwrite("file_name",
                     &TinyUrdfCollisionMesh<MyScalar, MyTinyConstants>::m_file_name)
      .def_readwrite("scale",
                     &TinyUrdfCollisionMesh<MyScalar, MyTinyConstants>::m_scale);

  py::class_<TinyUrdfGeometry<MyScalar, MyTinyConstants>>(m, "TinyUrdfGeometry")
      .def(py::init<>())
      .def_readwrite("geom_type",
                     &TinyUrdfGeometry<MyScalar, MyTinyConstants>::geom_type)
      .def_readwrite("sphere", &TinyUrdfGeometry<MyScalar, MyTinyConstants>::m_sphere)
      .def_readwrite("box", &TinyUrdfGeometry<MyScalar, MyTinyConstants>::m_box)
      .def_readwrite("plane", &TinyUrdfGeometry<MyScalar, MyTinyConstants>::m_plane)
      .def_readwrite("capsule",
                     &TinyUrdfGeometry<MyScalar, MyTinyConstants>::m_capsule)
      .def_readwrite("mesh", &TinyUrdfGeometry<MyScalar, MyTinyConstants>::m_mesh);

  py::class_<TinyUrdfCollision<MyScalar, MyTinyConstants>>(m, "TinyUrdfCollision")
      .def(py::init<>())

      .def_readwrite("origin_xyz",
                     &TinyUrdfCollision<MyScalar, MyTinyConstants>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &TinyUrdfCollision<MyScalar, MyTinyConstants>::origin_rpy)
      .def_readwrite("geometry",
                     &TinyUrdfCollision<MyScalar, MyTinyConstants>::geometry);

  py::class_<TinyUrdfVisual<MyScalar, MyTinyConstants>>(m, "TinyUrdfVisual")
      .def(py::init<>())
      .def_readwrite("origin_xyz",
                     &TinyUrdfVisual<MyScalar, MyTinyConstants>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &TinyUrdfVisual<MyScalar, MyTinyConstants>::origin_rpy)
      .def_readwrite("geometry", &TinyUrdfVisual<MyScalar, MyTinyConstants>::geometry)
      .def_readwrite(
          "sync_visual_body_uid1",
          &TinyUrdfVisual<MyScalar, MyTinyConstants>::sync_visual_body_uid1)
      .def_readwrite(
          "sync_visual_body_uid2",
          &TinyUrdfVisual<MyScalar, MyTinyConstants>::sync_visual_body_uid2);

  py::class_<TinyUrdfJoint<MyScalar, MyTinyConstants>>(m, "TinyUrdfJoint")
      .def(py::init<>())

      .def_readwrite("joint_name",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_name)
      .def_readwrite("joint_type",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_type)
      .def_readwrite("joint_lower_limit",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_lower_limit)
      .def_readwrite("joint_upper_limit",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_upper_limit)
      .def_readwrite("parent_name",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::parent_name)
      .def_readwrite("child_name",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::child_name)
      .def_readwrite("joint_origin_xyz",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_origin_xyz)
      .def_readwrite("joint_origin_rpy",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_origin_rpy)
      .def_readwrite("joint_axis_xyz",
                     &TinyUrdfJoint<MyScalar, MyTinyConstants>::joint_axis_xyz);

  py::class_<TinyUrdfLink<MyScalar, MyTinyConstants>>(m, "TinyUrdfLink")
      .def(py::init<>())
      .def_readwrite("link_name", &TinyUrdfLink<MyScalar, MyTinyConstants>::link_name)
      .def_readwrite("urdf_inertial",
                     &TinyUrdfLink<MyScalar, MyTinyConstants>::urdf_inertial)
      .def_readwrite("urdf_visual_shapes",
                     &TinyUrdfLink<MyScalar, MyTinyConstants>::urdf_visual_shapes)
      .def_readwrite("urdf_collision_shapes",
                     &TinyUrdfLink<MyScalar, MyTinyConstants>::urdf_collision_shapes)
      .def_readwrite("parent_index",
                     &TinyUrdfLink<MyScalar, MyTinyConstants>::m_parent_index);

  py::class_<TinyUrdfStructures<MyScalar, MyTinyConstants>>(m, "TinyUrdfStructures")
      .def(py::init<>())
      .def_readwrite("robot_name",
                     &TinyUrdfStructures<MyScalar, MyTinyConstants>::m_robot_name)
      .def_readwrite("base_links",
                     &TinyUrdfStructures<MyScalar, MyTinyConstants>::m_base_links)
      .def_readwrite("links", &TinyUrdfStructures<MyScalar, MyTinyConstants>::m_links)
      .def_readwrite("joints",
                     &TinyUrdfStructures<MyScalar, MyTinyConstants>::m_joints)
      .def_readwrite(
          "name_to_link_index",
          &TinyUrdfStructures<MyScalar, MyTinyConstants>::m_name_to_link_index);

  py::class_<UrdfToMultiBody2<MyScalar, MyTinyConstants>>(m, "UrdfToMultiBody2")
      .def(py::init<>())
      .def("convert2", &UrdfToMultiBody2<MyScalar, MyTinyConstants>::convert);

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

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


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

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
