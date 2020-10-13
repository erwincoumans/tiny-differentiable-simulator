


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


  py::class_<Geometry<MyAlgebra>,
             std::unique_ptr<Geometry<MyAlgebra>>>
      geom(m, "TinyGeometry");

  geom.def(py::init<int>())
      .def("get_type", &Geometry<MyAlgebra>::get_type);

  py::class_<Sphere<MyAlgebra>,
             std::unique_ptr<Sphere<MyAlgebra>>>(m, "TinySphere",
                                                               geom)
      .def(py::init<MyScalar>())
      .def("get_radius", &Sphere<MyAlgebra>::get_radius)
      .def("compute_local_inertia", &Sphere<MyAlgebra>::compute_local_inertia)
      ;

  py::class_<Plane<MyAlgebra>,
             std::unique_ptr<Plane<MyAlgebra>>>(m, "TinyPlane",
                                                              geom)
      .def(py::init<>())
      .def("get_normal", &Plane<MyAlgebra>::get_normal);


  py::class_<RigidBody<MyAlgebra>,
             std::unique_ptr<RigidBody<MyAlgebra>>>(
      m, "TinyRigidBody")
      .def(py::init<MyScalar, Geometry<MyAlgebra> *>(),
           py::return_value_policy::reference_internal)
      .def_readwrite("world_pose",
                     &RigidBody<MyAlgebra>::world_pose_)
      .def_readwrite("collision_geometry",
                     &RigidBody<MyAlgebra>::geometry_)
      .def_readwrite("linear_velocity",
                     &RigidBody<MyAlgebra>::linear_velocity_)
      .def_readwrite("angular_velocity",
                     &RigidBody<MyAlgebra>::angular_velocity_)
      .def_readwrite("local_inertia",
                     &RigidBody<MyAlgebra>::local_inertia_)
      .def_readwrite("total_force",
                     &RigidBody<MyAlgebra>::total_force_)
      .def_readwrite("total_torque",
                     &RigidBody<MyAlgebra>::total_torque_)
      .def_readwrite("user_index",
                     &RigidBody<MyAlgebra>::user_index_)

      .def("apply_gravity", &RigidBody<MyAlgebra>::apply_gravity)
      .def("apply_force_impulse",
           &RigidBody<MyAlgebra>::apply_force_impulse)

      .def("apply_central_force",
           &RigidBody<MyAlgebra>::apply_central_force)
      .def("apply_impulse", &RigidBody<MyAlgebra>::apply_impulse)
      .def("clear_forces", &RigidBody<MyAlgebra>::clear_forces)
      .def("integrate", &RigidBody<MyAlgebra>::integrate);


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
      .def(py::init<MyScalar, MyScalar, MyScalar, 
          MyScalar, MyScalar, MyScalar, 
          MyScalar, MyScalar, MyScalar>())
      .def(py::init<TinyQuaternion<MyScalar, MyTinyConstants>>())
      .def("get_at", &TinyMatrix3x3<MyScalar, MyTinyConstants>::get_at)
      .def("set_at", &TinyMatrix3x3<MyScalar, MyTinyConstants>::set_at)
      .def("get_row",&TinyMatrix3x3<MyScalar, MyTinyConstants>::getRow)
      .def("set_identity", &TinyMatrix3x3<MyScalar, MyTinyConstants>::set_identity)
      .def("setRotation", &TinyMatrix3x3<MyScalar, MyTinyConstants>::setRotation)
      .def("getRotation", &TinyMatrix3x3<MyScalar, MyTinyConstants>::getRotation2)
      ;

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

  py::class_<Pose<MyAlgebra>>(m, "TinyPose")
      .def(py::init<TinyVector3<MyScalar, MyTinyConstants>,
                    TinyQuaternion<MyScalar, MyTinyConstants>>())
      .def_readwrite("position", &Pose<MyAlgebra>::position_)
      .def_readwrite("orientation",
                     &Pose<MyAlgebra>::orientation_)
      .def("inverse_transform",
           &Pose<MyAlgebra>::inverse_transform);

  py::class_<Transform<MyAlgebra>>(m,
                                                        "TinySpatialTransform")
      .def(py::init<>())
      .def("set_identity",
           &Transform<MyAlgebra>::set_identity)
      .def_readwrite("translation",
                     &Transform<MyAlgebra>::translation)
      .def_readwrite("rotation",
                     &Transform<MyAlgebra>::rotation)
      .def(py::self * py::self)
      .def("get_inverse",
           &Transform<MyAlgebra>::inverse);

  py::class_<MotionVector<MyAlgebra>>(
      m, "TinySpatialMotionVector")
      //.def(py::init<int>())
      .def_readwrite("topVec",
                     &MotionVector<MyAlgebra>::top)
      .def_readwrite(
          "bottomVec",
          &MotionVector<MyAlgebra>::bottom);

  m.def("fraction",&fraction);

  m.def("get_debug_double", &MyTinyConstants::getDouble<MyScalar>);
#if 0    
  m.def("compute_inertia_dyad",
      &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::computeInertiaDyad);

  py::class_<TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>>(
      m, "TinySymmetricSpatialDyad")
      .def(py::init<>())
      .def("set_identity",
           &TinySymmetricSpatialDyad<MyScalar, MyTinyConstants>::setIdentity)

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
#endif

  py::enum_<JointType>(m, "TinyJointType")
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

  py::enum_<GeometryTypes>(m, "TinyGeometryTypes")
      .value("SPHERE_TYPE", TINY_SPHERE_TYPE, "SPHERE_TYPE")
      .value("BOX_TYPE", TINY_PLANE_TYPE, "BOX_TYPE")
      .value("PLANE_TYPE", TINY_PLANE_TYPE, "PLANE_TYPE")
      .value("CAPSULE_TYPE", TINY_CAPSULE_TYPE, "CAPSULE_TYPE")
      .value("MESH_TYPE", TINY_MESH_TYPE, "MESH_TYPE")
      .export_values();

  py::class_<Link<MyAlgebra>,
             std::unique_ptr<Link<MyAlgebra>>>(m, "TinyLink")
      .def(py::init<JointType, const Transform<MyAlgebra> &,
                    const RigidBodyInertia<MyAlgebra> &>())
      .def("jcalc", &Link<MyAlgebra>::jcalc1)
      .def("set_joint_type", &Link<MyAlgebra>::set_joint_type)
      .def_readwrite("stiffness", &Link<MyAlgebra>::stiffness)
      .def_readwrite("joint_type", &Link<MyAlgebra>::joint_type)
      .def_readwrite("damping", &Link<MyAlgebra>::damping);

  py::class_<MultiBody<MyAlgebra>,
             std::unique_ptr<MultiBody<MyAlgebra>>>(
      m, "TinyMultiBody")
      .def(py::init<bool>())
      .def("initialize", &MultiBody<MyAlgebra>::initialize)
      .def("set_base_position",
           &MultiBody<MyAlgebra>::set_position)
      .def("get_world_transform",
           &MultiBody<MyAlgebra>::get_world_transform)
      .def("mass_matrix", &MyMassMatrix)
#if 0
      .def("attach_link", &MultiBody<MyAlgebra>::attach_link)
      .def("forward_kinematics",
           &MultiBody<MyAlgebra>::forward_kinematics1)
      .def("forward_dynamics",
           py::overload_cast<const TinyVector3<MyScalar, MyTinyConstants> &>(
               &MultiBody<MyAlgebra>::forward_dynamics))
      .def("integrate", py::overload_cast<MyScalar>(
                            &MultiBody<MyAlgebra>::integrate))
      .def("integrate_q", &MultiBody<MyAlgebra>::integrate_q)
      .def("body_to_world", &MultiBody<MyAlgebra>::body_to_world)
      .def("world_to_body", &MultiBody<MyAlgebra>::world_to_body)
      .def("point_jacobian",
           &MultiBody<MyAlgebra>::point_jacobian1)
      .def("bias_forces", &MultiBody<MyAlgebra>::bias_forces)
#endif
      //.def_property_readonly("num_dofs", &MultiBody<MyAlgebra>::dof)
      //.def_property_readonly("num_dofs_qd", &MultiBody<MyAlgebra>::dof_qd)
      //.def_readwrite("q", &MultiBody<MyAlgebra>::q)
      //.def_readwrite("links", &MultiBody<MyAlgebra>::links)
      //.def_readwrite("qd", &MultiBody<MyAlgebra>::qd)
      //.def_readwrite("qdd", &MultiBody<MyAlgebra>::qdd)
      //.def_readwrite("tau", &MultiBody<MyAlgebra>::tau)
      ;


  py::class_<CollisionDispatcher<MyAlgebra>>(
      m, "TinyCollisionDispatcher")
      .def(py::init<>())
      .def("compute_contacts", &CollisionDispatcher<MyAlgebra>::compute_contacts2);


  py::class_<ContactPoint<MyAlgebra>> contact(m,
                                                            "TinyContactPoint");
  contact.def(py::init<>())
      .def_readwrite(
          "world_normal_on_b",
          &ContactPoint<MyAlgebra>::world_normal_on_b)
      .def_readwrite("world_point_on_a",
                     &ContactPoint<MyAlgebra>::world_point_on_a)
      .def_readwrite("world_point_on_b",
                     &ContactPoint<MyAlgebra>::world_point_on_b)
      .def_readwrite("distance",
                     &ContactPoint<MyAlgebra>::distance);

  py::class_<RigidBodyContactPoint<MyAlgebra>>(
      m, "TinyContactPointRigidBody", contact)
      .def(py::init<>())
      .def_readwrite(
          "rigid_body_a",
          &RigidBodyContactPoint<MyAlgebra>::rigid_body_a)
      .def_readwrite(
          "rigid_body_b",
          &RigidBodyContactPoint<MyAlgebra>::rigid_body_b)
      .def_readwrite(
          "restitution",
          &RigidBodyContactPoint<MyAlgebra>::restitution)
      .def_readwrite(
          "friction",
          &RigidBodyContactPoint<MyAlgebra>::friction);

  py::class_<MultiBodyContactPoint<MyAlgebra>>(
      m, "TinyContactPointMultiBody", contact)
      .def(py::init<>())
      .def_readwrite(
          "multi_body_a",
          &MultiBodyContactPoint<MyAlgebra>::multi_body_a)
      .def_readwrite(
          "multi_body_b",
          &MultiBodyContactPoint<MyAlgebra>::multi_body_b)
      .def_readwrite(
          "restitution",
          &MultiBodyContactPoint<MyAlgebra>::restitution)
      .def_readwrite(
          "friction",
          &MultiBodyContactPoint<MyAlgebra>::friction)
      .def_readwrite("link_a",
                     &MultiBodyContactPoint<MyAlgebra>::link_a)
      .def_readwrite("link_b",
                     &MultiBodyContactPoint<MyAlgebra>::link_b);

  py::class_<RigidBodyConstraintSolver<MyAlgebra>>(m,
                                                        "TinyConstraintSolver")
      .def(py::init<>())
      .def("resolve_collision",
           &RigidBodyConstraintSolver<MyAlgebra>::resolve_collision);

  py::class_<MultiBodyConstraintSolver<MyAlgebra>>(
      m, "TinyMultiBodyConstraintSolver")
      .def(py::init<>())
      .def("resolve_collision",
           &MultiBodyConstraintSolver<MyAlgebra>::resolve_collision);

  py::enum_<VelocitySmoothingMethod>(m, "TinyVelocitySmoothingMethod",
                                         py::arithmetic())
      .value("SMOOTH_VEL_NONE", SMOOTH_VEL_NONE)
      .value("SMOOTH_VEL_SIGMOID", SMOOTH_VEL_SIGMOID)
      .value("SMOOTH_VEL_TANH", SMOOTH_VEL_TANH)
      .value("SMOOTH_VEL_ABS", SMOOTH_VEL_ABS)
      .export_values();

  typedef MultiBodyConstraintSolverSpring<MyAlgebra> TMBCSS;
  py::class_<TMBCSS>(m, "TinyMultiBodyConstraintSolverSpring")
      .def(py::init<>())
      .def("resolve_collision", &TMBCSS::resolve_collision)
      .def_readwrite("spring_k", &TMBCSS::spring_k_)
      .def_readwrite("damper_d", &TMBCSS::damper_d_)
      .def_readwrite("hard_contact_condition", &TMBCSS::hard_contact_condition_)
      .def_readwrite("exponent_n", &TMBCSS::exponent_n_)
      .def_readwrite("exponent_n_air", &TMBCSS::exponent_n_air_)
      .def_readwrite("exponent_vel_air", &TMBCSS::exponent_vel_air_)
      .def_readwrite("smoothing_method", &TMBCSS::smoothing_method_)
      .def_readwrite("smooth_alpha_vel", &TMBCSS::smooth_alpha_vel_)
      .def_readwrite("smooth_alpha_normal", &TMBCSS::smooth_alpha_normal_)
      .def_readwrite("mu_static", &TMBCSS::mu_static_)
      .def_readwrite("andersson_vs", &TMBCSS::andersson_vs_)
      .def_readwrite("andersson_p", &TMBCSS::andersson_p_)
      .def_readwrite("andersson_ktanh", &TMBCSS::andersson_ktanh_)
      .def_readwrite("v_transition", &TMBCSS::v_transition_)
      .def_readwrite("friction_model", &TMBCSS::friction_model_)
      .def("compute_contact_force", &TMBCSS::compute_contact_force)
      .def("compute_friction_force", &TMBCSS::compute_friction_force);

  py::class_<World<MyAlgebra>>(m, "TinyWorld")
      .def(py::init<>())
      .def("step", &World<MyAlgebra>::step)
      .def_property("gravity", &World<MyAlgebra>::get_gravity,
                    &World<MyAlgebra>::set_gravity)
      .def("compute_contacts_rigid_body",
           &World<MyAlgebra>::compute_contacts_rigid_body)
      .def("compute_contacts_multi_body",
           &World<MyAlgebra>::compute_contacts_multi_body)
      //.def("get_collision_dispatcher",
      //     &World<MyAlgebra>::get_collision_dispatcher)
      .def_readwrite("friction",
                     &World<MyAlgebra>::default_friction)
      .def_readwrite("restitution",
                     &World<MyAlgebra>::default_restitution);

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
#if 0
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
#endif

  py::class_<UrdfCollisionSphere<MyAlgebra>>(
      m, "TinyUrdfCollisionSphere")
      .def(py::init<>())
      .def_readwrite("radius",
                     &UrdfCollisionSphere<MyAlgebra>::radius);

  py::class_<UrdfCollisionBox<MyAlgebra>>(m,
                                                        "TinyUrdfCollisionBox")
      .def(py::init<>())
      .def_readwrite("extents",
                     &UrdfCollisionBox < MyAlgebra>::extents);

  py::class_<UrdfCollisionCapsule<MyAlgebra>>(
      m, "TinyUrdfCollisionCapsule")
      .def(py::init<>())
      .def_readwrite("radius",
                     &UrdfCollisionCapsule<MyAlgebra>::radius)
      .def_readwrite("length",
                     &UrdfCollisionCapsule<MyAlgebra>::length);

  py::class_<UrdfCollisionPlane<MyAlgebra>>(
      m, "TinyUrdfCollisionPlane")
      .def(py::init<>())
      .def_readwrite("constant",
                     &UrdfCollisionPlane<MyAlgebra>::constant)
      .def_readwrite("normal",
                     &UrdfCollisionPlane<MyAlgebra>::normal);

  py::class_<UrdfCollisionMesh<MyAlgebra>>(
      m, "TinyUrdfCollisionMesh")
      .def(py::init<>())
      .def_readwrite("file_name",
                     &UrdfCollisionMesh<MyAlgebra>::file_name)
      .def_readwrite("scale",
                     &UrdfCollisionMesh<MyAlgebra>::scale);

  py::class_<UrdfGeometry<MyAlgebra>>(m, "TinyUrdfGeometry")
      .def(py::init<>())
      .def_readwrite("geom_type",
                     &UrdfGeometry<MyAlgebra>::geom_type)
      .def_readwrite("sphere", &UrdfGeometry<MyAlgebra>::sphere)
      .def_readwrite("box", &UrdfGeometry<MyAlgebra>::box)
      .def_readwrite("plane", &UrdfGeometry<MyAlgebra>::plane)
      .def_readwrite("capsule",
                     &UrdfGeometry<MyAlgebra>::capsule)
      .def_readwrite("mesh", &UrdfGeometry<MyAlgebra>::mesh);

  py::class_<UrdfCollision<MyAlgebra>>(m, "TinyUrdfCollision")
      .def(py::init<>())

      .def_readwrite("origin_xyz",
                     &UrdfCollision<MyAlgebra>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &UrdfCollision<MyAlgebra>::origin_rpy)
      .def_readwrite("geometry",
                     &UrdfCollision<MyAlgebra>::geometry);

  py::class_<UrdfVisual<MyAlgebra>>(m, "TinyUrdfVisual")
      .def(py::init<>())
      .def_readwrite("origin_xyz",
                     &UrdfVisual<MyAlgebra>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &UrdfVisual<MyAlgebra>::origin_rpy)
      .def_readwrite("geometry", &UrdfVisual<MyAlgebra>::geometry)
      .def_readwrite(
          "sync_visual_body_uid1",
          &UrdfVisual<MyAlgebra>::sync_visual_body_uid1)
      .def_readwrite(
          "sync_visual_body_uid2",
          &UrdfVisual<MyAlgebra>::sync_visual_body_uid2);

  py::class_<UrdfJoint<MyAlgebra>>(m, "TinyUrdfJoint")
      .def(py::init<>())

      .def_readwrite("joint_name",
                     &UrdfJoint<MyAlgebra>::joint_name)
      .def_readwrite("joint_type",
                     &UrdfJoint<MyAlgebra>::joint_type)
      .def_readwrite("joint_lower_limit",
                     &UrdfJoint<MyAlgebra>::joint_lower_limit)
      .def_readwrite("joint_upper_limit",
                     &UrdfJoint<MyAlgebra>::joint_upper_limit)
      .def_readwrite("parent_name",
                     &UrdfJoint<MyAlgebra>::parent_name)
      .def_readwrite("child_name",
                     &UrdfJoint<MyAlgebra>::child_name)
      .def_readwrite("joint_origin_xyz",
                     &UrdfJoint<MyAlgebra>::joint_origin_xyz)
      .def_readwrite("joint_origin_rpy",
                     &UrdfJoint<MyAlgebra>::joint_origin_rpy)
      .def_readwrite("joint_axis_xyz",
                     &UrdfJoint<MyAlgebra>::joint_axis_xyz);


  py::class_<UrdfLink<MyAlgebra>>(m, "TinyUrdfLink")
      .def(py::init<>())
      .def_readwrite("link_name", &UrdfLink<MyAlgebra>::link_name)
      .def_readwrite("urdf_inertial",
                     &UrdfLink<MyAlgebra>::urdf_inertial)
      .def_readwrite("urdf_visual_shapes",
                     &UrdfLink<MyAlgebra>::urdf_visual_shapes)
      .def_readwrite("urdf_collision_shapes",
                     &UrdfLink<MyAlgebra>::urdf_collision_shapes)
      .def_readwrite("parent_index",
                     &UrdfLink<MyAlgebra>::parent_index);

  py::class_<UrdfParser<MyAlgebra> >(m, "TinyUrdfParser")
      .def(py::init<>())
      .def("load_urdf", &UrdfParser<MyAlgebra>::load_urdf);

  py::class_<UrdfStructures<MyAlgebra>>(m, "TinyUrdfStructures")
      .def(py::init<>())
      .def_readwrite("robot_name",
                     &UrdfStructures<MyAlgebra>::robot_name)
      .def_readwrite("base_links",
                     &UrdfStructures<MyAlgebra>::base_links)
      .def_readwrite("links", &UrdfStructures<MyAlgebra>::links)
      .def_readwrite("joints",
                     &UrdfStructures<MyAlgebra>::joints)
      .def_readwrite(
          "name_to_link_index",
          &UrdfStructures<MyAlgebra>::name_to_link_index);

  py::class_<UrdfToMultiBody2<MyAlgebra>>(m, "UrdfToMultiBody2")
      .def(py::init<>())
      .def("convert2", &UrdfToMultiBody2<MyAlgebra>::convert);

  

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif


  
#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
