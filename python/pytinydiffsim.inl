


  m.doc() = R"pbdoc(
        tiny differentiable physics python plugin
        -----------------------

        .. currentmodule:: pytinydiffsim

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  /* Define Algebra independent Vectors and Matrices */
  typedef typename MyAlgebra::VectorX VectorX;
  py::class_<VectorX>(m, "VectorX")
      .def(py::init<int>())
      .def("set_zero", [](VectorX& a) {
          MyAlgebra::set_zero(a);
      })
      .def("size",
      [](const VectorX& a) {
          return MyAlgebra::size(a);
      })
      .def("__getitem__", [](const VectorX& a, int i) { return a[i]; })
      .def("__setitem__", [](VectorX& a, int i, MyScalar v) { a[i] = v; })
      .def("__setitem__", [](VectorX& a, int i, double v) { a[i] = MyAlgebra::from_double(v); })
      .def("__repr__",
      [](const VectorX& a) {
          std::string values;
          for (int i = 0; i < a.size(); i++)
          {
              values += std::to_string(MyAlgebra::to_double(a[i])) + ",";
          }
          return "[" + values + "]";
      })
      ;
      
  typedef typename MyAlgebra::Vector3 Vector3;
  py::class_<Vector3>(m, "Vector3")
      .def(py::init([](MyScalar x, MyScalar y, MyScalar z) {
          return std::unique_ptr<Vector3>(new Vector3(x, y, z));
      }))
      .def(py::init([](InnerScalar x, InnerScalar y, InnerScalar z) {
          return std::unique_ptr<Vector3>(new Vector3(MyScalar(x), MyScalar(y), MyScalar(z)));
      }))
      .def("set_zero", [](Vector3& a) {
          MyAlgebra::set_zero(a);
      })
      .def("sqnorm", [](Vector3& a) { return MyAlgebra::sqnorm(a); })
      .def_property("x", 
          [](Vector3& a) {
              return a[0];
          },
          [](Vector3& a, MyScalar v) {
              a[0] = v;
          }
      )
      .def_property("y", 
          [](Vector3& a) {
              return a[1];
          },
          [](Vector3& a, MyScalar v) {
              a[1] = v;
          }
      )
      .def_property("z", 
          [](Vector3& a) {
              return a[2];
          },
          [](Vector3& a, MyScalar v) {
              a[2] = v;
          }
      )
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(-py::self)
      .def("__getitem__", [](const Vector3& a, int i) { return a[i]; })
      .def("__setitem__", [](Vector3& a, int i, MyScalar v) { a[i] = v; })
      .def("__setitem__", [](Vector3& a, int i, double v) { a[i] = MyAlgebra::from_double(v); })
      .def("__repr__",
      [](const Vector3& a) {
          std::string values;
          for (int i = 0; i < 3; i++)
          {
              values += std::to_string(MyAlgebra::to_double(a[i])) + ",";
          }
          return "[" + values + "]";
      })
      ;
      
  typedef typename MyAlgebra::Quaternion Quaternion;
  py::class_<Quaternion>(m, "Quaternion")
      .def(py::init([](const MyScalar x, const MyScalar y, const MyScalar z, const MyScalar w) {
          return MyAlgebra::quat_from_xyzw(x, y, z, w);
      }))
      .def(py::init([](const InnerScalar x, const InnerScalar y, const InnerScalar z, const InnerScalar w) {
          return MyAlgebra::quat_from_xyzw(MyScalar(x), MyScalar(y), MyScalar(z), MyScalar(w));
      }))
      .def("set_identity", [](Quaternion &q) {
          MyAlgebra::set_identity(q);
      })
      .def("inverse", [](const Quaternion &q) {
          return MyAlgebra::inverse(q);
      })
      .def("get_euler_rpy", [](const Quaternion &q) {
          return MyAlgebra::get_euler_rpy(q);
      })
      .def("get_euler_rpy2", [](const Quaternion &q) {
          return MyAlgebra::get_euler_rpy2(q);
      })
      .def("set_euler_rpy", [](Quaternion &q, const Vector3 &rpy) {
          MyAlgebra::set_euler_rpy(q, rpy);
      })
      .def("normalized", [](const Quaternion &q) {
          return MyAlgebra::normalize(q);
      })
      .def(py::self * py::self)
      .def_property_readonly("x", [](const Quaternion &q) { return q.x(); })
      .def_property_readonly("y", [](const Quaternion &q) { return q.y(); })
      .def_property_readonly("z", [](const Quaternion &q) { return q.z(); })
      .def_property_readonly("w", [](const Quaternion &q) { return q.w(); })
      .def("__repr__",
           [](const Quaternion &q) {
             return "[x=" + std::to_string(MyTinyConstants::getDouble(q.x())) + 
                    " y=" + std::to_string(MyTinyConstants::getDouble(q.y())) +
                    " z=" + std::to_string(MyTinyConstants::getDouble(q.z())) + 
                    " w=" + std::to_string(MyTinyConstants::getDouble(q.w())) + "]";
           })
      ;

  typedef typename MyAlgebra::MatrixX Matrix;
  py::class_<Matrix>(m, "Matrix")
      .def(py::init<int, int>())
//      .def("inversed", [](const Matrix& a) {  // Inconsisten API between Eigen and Tiny.
//          MyAlgebra::inverse(a);
//      })
      .def("set_zero", [](Matrix& a) {
          MyAlgebra::set_zero(a);
      })
//      .def("print", [](const Matrix& a, const std::string &title) {
//          MyAlgebra::print(title, a);
//      })
      .def("mult", [](const Matrix& a, const Matrix& b){
          return MyAlgebra::mult(a, b);
      })
      .def("mult", [](const Matrix& a, const VectorX& b){
          return MyAlgebra::mult(a, b);
      })
      .def("mult", [](const Matrix& a, const Vector3& b){
          return MyAlgebra::mult(a, b);
      })
      .def("transpose", [](const Matrix& a) {
          return MyAlgebra::transpose(a);
      })
      .def_static("eye", &MyAlgebra::eye)
      .def("__getitem__", [](const Matrix& a, py::tuple t) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          return a(row, col);
      })
      .def("__setitem__", [](Matrix& a, py::tuple t, MyScalar v) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          a(row, col) = v;
      })
      .def_property_readonly("num_rows", [](const Matrix& a) {
          return MyAlgebra::num_rows(a);
      })
      .def_property_readonly("num_columns", [](const Matrix& a) {
          return MyAlgebra::num_cols(a);
      })
      .def("__repr__",
      [](const Matrix& a) {
          std::string values;
          for (int i = 0; i < MyAlgebra::num_rows(a); i++) {
              for (int j = 0; j < MyAlgebra::num_cols(a); j++) {
                  values += std::to_string(MyAlgebra::to_double(a(i, j))) + ",";
              }
              if (i < MyAlgebra::num_rows(a) - 1)
                  values += "\n";
          }
          return "[" + values + "]";
      })
      ;

  
  typedef typename MyAlgebra::Matrix3 Matrix3;
  py::class_<Matrix3>(m, "Matrix3")
      .def(py::init<>())
      .def(py::init([](const Quaternion &quat) {
          return std::unique_ptr<Matrix3>(new Matrix3(quat));
      }))
//      .def(py::init<MyScalar, MyScalar, MyScalar, 
//          MyScalar, MyScalar, MyScalar, 
//          MyScalar, MyScalar, MyScalar>())
      .def("get_at", [](const Matrix3& a, const int row, const int col) {
          return a(row, col);
      })
      .def("__getitem__", [](const Matrix3& a, py::tuple t) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          return a(row, col);
      })
      .def("__setitem__", [](Matrix3& a, py::tuple t, MyScalar v) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          a(row, col) = v;
      })
//      .def("set_at", &TinyMatrix3x3<MyScalar, MyTinyConstants>::set_at)
      .def("get_row", [](const Matrix3& a, const int row) {
          return MyAlgebra::get_row(a, row);
      })
//      .def("transposed", &TinyMatrix3x3<MyScalar, MyTinyConstants>::transpose)
//      .def("set_identity", &TinyMatrix3x3<MyScalar, MyTinyConstants>::set_identity)
//      .def("setRotation", &TinyMatrix3x3<MyScalar, MyTinyConstants>::setRotation)
//      .def("getRotation", &TinyMatrix3x3<MyScalar, MyTinyConstants>::getRotation2)
      ;

  typedef typename MyAlgebra::Matrix3X Matrix3X;
  py::class_<Matrix3X>(m, "Matrix3X")
      .def(py::init<>())
      .def_property_readonly("num_rows", [](const Matrix3X& a) {
          return MyAlgebra::num_rows(a);
      })
      .def_property_readonly("num_columns", [](const Matrix3X& a) {
          return MyAlgebra::num_cols(a);
      })
     .def("get_at", [](const Matrix3X& a, const int row, const int col) {
          return a( row, col);
      })
      //.def("print", &Matrix3X::print)
      .def("__getitem__", [](const Matrix3X& a, py::tuple t) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          return a(row, col);
      })
      .def("__setitem__", [](Matrix3X& a, py::tuple t, MyScalar v) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          a(row, col) = v;
      })
      ;

  typedef typename MyAlgebra::Matrix6x3 Matrix6x3;
  py::class_<Matrix6x3>(m, "Matrix6x3")
      .def(py::init<>())
  
     .def("get_at", [](const Matrix6x3& a, const int row, const int col) {
          return a( row, col);
      })
      //.def("print", &Matrix6x3::print)
      .def("__getitem__", [](const Matrix6x3& a, py::tuple t) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          return a(row, col);
      })
      .def("__setitem__", [](Matrix6x3& a, py::tuple t, MyScalar v) {
          if (t.size() != 2)
              throw std::runtime_error("Invalid indexing!");
          int row = t[0].cast<int>();
          int col = t[1].cast<int>();
          a(row, col) = v;
      })
      ;
/*
  py::class_<TinyVectorX<MyScalar, MyTinyConstants>>(m, "TinyVectorX")
      .def(py::init<int>())
      .def("set_zero", &TinyVectorX<MyScalar, MyTinyConstants>::set_zero)
      .def("size", &TinyVectorX<MyScalar, MyTinyConstants>::size)
      .def("std", &TinyVectorX<MyScalar, MyTinyConstants>::std)
      .def("__getitem__", [](const TinyVectorX<MyScalar, MyTinyConstants>& a,
          int i) { return a[i]; })
      .def("__setitem__", [](TinyVectorX<MyScalar, MyTinyConstants>& a, int i,
          MyScalar v) { a[i] = v; })
      .def("__repr__",
      [](const TinyVectorX<MyScalar, MyTinyConstants>& a) {
          std::string values;
          for (int i = 0; i < a.size(); i++)
          {
              values += std::to_string(MyTinyConstants::getDouble(a[i])) + ",";
          }
          return "[" + values + "]";
      })
      ;


  py::class_<TinyVector3<MyScalar, MyTinyConstants>>(m, "TinyVector3")
      .def(py::init<MyScalar, MyScalar, MyScalar>())
      .def("set_zero", &TinyVector3<MyScalar, MyTinyConstants>::set_zero)
      .def("sqnorm", &TinyVector3<MyScalar, MyTinyConstants>::sqnorm)
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

*/

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
      .def("get_normal", &Plane<MyAlgebra>::get_normal)
      .def("get_constant", &Plane<MyAlgebra>::get_constant);

  py::class_<Capsule<MyAlgebra>,
             std::unique_ptr<Capsule<MyAlgebra>>>(m, "TinyCapsule", geom)
      .def(py::init<MyScalar, MyScalar>())
      .def("get_radius", &Capsule<MyAlgebra>::get_radius)
      .def("get_length", &Capsule<MyAlgebra>::get_length);


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
      .def_readwrite("total_force",
                     &RigidBody<MyAlgebra>::total_force_)
      .def_readwrite("total_torque",
                     &RigidBody<MyAlgebra>::total_torque_)

      .def("apply_gravity", &RigidBody<MyAlgebra>::apply_gravity)
      .def("apply_force_impulse",
           &RigidBody<MyAlgebra>::apply_force_impulse)

      .def("apply_central_force",
           &RigidBody<MyAlgebra>::apply_central_force)
      .def("apply_impulse", &RigidBody<MyAlgebra>::apply_impulse)
      .def("clear_forces", &RigidBody<MyAlgebra>::clear_forces)
      .def("integrate", &RigidBody<MyAlgebra>::integrate);

/*
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
      .def("transposed", &TinyMatrix3x3<MyScalar, MyTinyConstants>::transpose)
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
*/

  py::class_<Pose<MyAlgebra>>(m, "TinyPose")
      .def(py::init<MyAlgebra::Vector3,
                    MyAlgebra::Quaternion>())
      .def_readwrite("position", &Pose<MyAlgebra>::position_)
      .def_readwrite("orientation",
                     &Pose<MyAlgebra>::orientation_)
      .def("inverse_transform",
           &Pose<MyAlgebra>::inverse_transform);

  py::class_<Transform<MyAlgebra>>(m,"TinySpatialTransform")
      .def(py::init<>())
      .def("set_identity",
           &Transform<MyAlgebra>::set_identity)
      .def_readwrite("translation",
                     &Transform<MyAlgebra>::translation)
      .def_readwrite("rotation",
                     &Transform<MyAlgebra>::rotation)
      .def("apply_inverse",
          &Transform<MyAlgebra>::apply_inverse2)
      .def("print", &Transform<MyAlgebra>::print)
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


  py::class_<ForceVector<MyAlgebra>>(
      m, "TinySpatialForceVector")
      //.def(py::init<int>())
      .def_readwrite("topVec",
                     &ForceVector<MyAlgebra>::top)
      .def_readwrite(
          "bottomVec",
          &ForceVector<MyAlgebra>::bottom);

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
      .value("JOINT_SPHERICAL", JOINT_SPHERICAL, "JOINT_SPHERICAL")
      .export_values();

  py::enum_<GeometryTypes>(m, "TinyGeometryTypes")
      .value("SPHERE_TYPE", TINY_SPHERE_TYPE, "SPHERE_TYPE")
      .value("BOX_TYPE", TINY_BOX_TYPE, "BOX_TYPE")
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
      .def_readwrite("damping", &Link<MyAlgebra>::damping)
      .def_readwrite("link_name", &Link<MyAlgebra>::link_name)
      .def_readwrite("joint_name", &Link<MyAlgebra>::joint_name)
      .def_readwrite("q_index", &Link<MyAlgebra>::q_index)
      .def_readwrite("qd_index", &Link<MyAlgebra>::qd_index)
      .def_readwrite("world_transform", &Link<MyAlgebra>::X_world)
      .def_readwrite("index", &Link<MyAlgebra>::index)
      .def_readwrite("parent_index", &Link<MyAlgebra>::parent_index)
      .def_readwrite("X_T", &Link<MyAlgebra>::X_T)
      .def_readwrite("X_J", &Link<MyAlgebra>::X_J)
      .def_readwrite("X_parent", &Link<MyAlgebra>::X_parent)
      .def_readwrite("X_world", &Link<MyAlgebra>::X_world)
      .def_readwrite("vJ", &Link<MyAlgebra>::vJ)
      .def_readwrite("cJ", &Link<MyAlgebra>::cJ)
      .def_readwrite("v", &Link<MyAlgebra>::v)
      .def_readwrite("a", &Link<MyAlgebra>::a)
      .def_readwrite("c", &Link<MyAlgebra>::c)
      .def_readwrite("rbi", &Link<MyAlgebra>::rbi)
      .def_readwrite("abi", &Link<MyAlgebra>::abi)
      .def_readwrite("pA", &Link<MyAlgebra>::pA)
      .def_readwrite("S", &Link<MyAlgebra>::S)
      .def_readwrite("U", &Link<MyAlgebra>::U)
      .def_readwrite("D", &Link<MyAlgebra>::D)
      .def_readwrite("u", &Link<MyAlgebra>::u)
      .def_readwrite("f", &Link<MyAlgebra>::f)
      .def_readwrite("S_3d", &Link<MyAlgebra>::S_3d)
      .def_readwrite("U_3d", &Link<MyAlgebra>::U_3d)
      .def_readwrite("D_3d", &Link<MyAlgebra>::D_3d)
      .def_readwrite("invD_3d", &Link<MyAlgebra>::invD_3d)
      .def_readwrite("u_3d", &Link<MyAlgebra>::u_3d)
      .def_readwrite("f_ext", &Link<MyAlgebra>::f_ext)
      ;
  
  py::class_<RigidBodyInertia<MyAlgebra>,
      std::unique_ptr<RigidBodyInertia<MyAlgebra>>>(m, "RigidBodyInertia")
      .def(py::init<const MyAlgebra::Scalar&, const MyAlgebra::Vector3&,
          const MyAlgebra::Matrix3&>())
      .def("set_zero", &RigidBodyInertia<MyAlgebra>::set_zero)
      .def_readwrite("mass", &RigidBodyInertia<MyAlgebra>::mass)
      .def_readwrite("com", &RigidBodyInertia<MyAlgebra>::com)
      .def_readwrite("inertia", &RigidBodyInertia<MyAlgebra>::inertia)
      ;
      
  py::class_<ArticulatedBodyInertia<MyAlgebra>,
      std::unique_ptr<ArticulatedBodyInertia<MyAlgebra>>>(m, "ArticulatedBodyInertia")
      .def(py::init<const MyAlgebra::Matrix3&,
                    const MyAlgebra::Matrix3&,
                    const MyAlgebra::Matrix3&>())
      .def_readwrite("I", &ArticulatedBodyInertia<MyAlgebra>::I)
      .def_readwrite("H", &ArticulatedBodyInertia<MyAlgebra>::H)
      .def_readwrite("M", &ArticulatedBodyInertia<MyAlgebra>::M)
      ;

  py::class_<MultiBody<MyAlgebra>,
             std::unique_ptr<MultiBody<MyAlgebra>>>(
      m, "TinyMultiBody")
      .def(py::init<bool>())
      .def("initialize", &MultiBody<MyAlgebra>::initialize)
      .def("set_position",
           &MultiBody<MyAlgebra>::set_position)
      .def("set_orientation",
        py::overload_cast<const Quaternion&>(
          &MultiBody<MyAlgebra>::set_orientation))
      .def("get_position",
          &MultiBody<MyAlgebra>::get_position)
      .def("get_orientation",
          &MultiBody<MyAlgebra>::get_orientation)
      .def("get_world_transform",
           &MultiBody<MyAlgebra>::get_world_transform)
      .def("get_world_com", &MultiBody<MyAlgebra>::get_world_com)
      .def("attach_link", &MultiBody<MyAlgebra>::attach_link)
      .def("set_q", &MultiBody<MyAlgebra>::set_q)
#if 0
      .def("point_jacobian",
           &MultiBody<MyAlgebra>::point_jacobian1)
      .def("bias_forces", &MultiBody<MyAlgebra>::bias_forces)
#endif
      .def("world_to_body", &MultiBody<MyAlgebra>::world_to_body)
      .def("body_to_world", &MultiBody<MyAlgebra>::body_to_world)
      .def("clear_forces", &MultiBody<MyAlgebra>::clear_forces)
      .def("is_floating", &MultiBody<MyAlgebra>::is_floating)
      .def("joint_damping", &MultiBody<MyAlgebra>::joint_damping)
      .def_property_readonly("num_dofs", &MultiBody<MyAlgebra>::dof)
      //.def_property_readonly("num_dofs_qd", &MultiBody<MyAlgebra>::dof_qd)
      .def_readwrite("q", &MultiBody<MyAlgebra>::q_)
      .def_readwrite("links", &MultiBody<MyAlgebra>::links_)
      .def_readwrite("qd", &MultiBody<MyAlgebra>::qd_)
      .def_readwrite("qdd", &MultiBody<MyAlgebra>::qdd_)
      .def_readwrite("tau", &MultiBody<MyAlgebra>::tau_)
      .def("base_velocity", py::overload_cast<>(&MultiBody<MyAlgebra>::base_velocity, py::const_))
      .def("base_acceleration", py::overload_cast<>(&MultiBody<MyAlgebra>::base_acceleration, py::const_))
      .def("base_applied_force", py::overload_cast<>(&MultiBody<MyAlgebra>::base_applied_force, py::const_))
      .def("base_force", py::overload_cast<>(&MultiBody<MyAlgebra>::base_force, py::const_))
      .def("base_bias_force", py::overload_cast<>(&MultiBody<MyAlgebra>::base_bias_force, py::const_))
      .def("base_rbi", py::overload_cast<>(&MultiBody<MyAlgebra>::base_rbi, py::const_))
      .def("base_abi", py::overload_cast<>(&MultiBody<MyAlgebra>::base_abi, py::const_))
      .def("base_X_world", py::overload_cast<>(&MultiBody<MyAlgebra>::base_X_world, py::const_))
      .def("collision_geometries", py::overload_cast<int>(&MultiBody<MyAlgebra>::collision_geometries, py::const_))
      .def("collision_transforms", py::overload_cast<int>(&MultiBody<MyAlgebra>::collision_transforms, py::const_))
      ;

  m.def("fraction", &fraction);
  m.def("copy", &copy);
  m.def("mass_matrix", &MyMassMatrix);
  m.def("forward_kinematics", &MyForwardKinematics);
  m.def("forward_dynamics", &MyForwardDynamics);
  m.def("integrate_euler", &MyIntegrateEuler);
  m.def("integrate_euler_qdd", &MyIntegrateEulerQdd);
  m.def("compute_inertia_dyad", &MyComputeInertia);
  m.def("point_jacobian", &MyPointJacobian);
  m.def("quat_integrate", &MyQuatIntegrate);
  m.def("inverse_kinematics", &MyInverseKinematics);
  m.def("link_transform_base_frame", &MyGetLinkTransformInBase);
  m.def("find_file", &MyFindFile);
  m.def("quat_difference", &QuaternionDifference);
  m.def("quaternion_axis_angle", &Quaternion_Axis_Angle);
  m.def("mb_collision_geometries", &mb_collision_geometries);
  m.def("matrix_to_euler_xyz", &matrix_to_euler_xyz<MyAlgebra>);
  m.def("get_axis_difference_quaternion", &get_axis_difference_quaternion<MyAlgebra>);

  m.def("pi", &MyPi);
  m.def("cos", &MyCos);
  m.def("acos", &MyAcos);
  m.def("sin", &MySin);
  m.def("max", &MyMax);
  m.def("min", &MyMin);
  m.def("where_gt", &MyWhereGT);
  m.def("where_lt", &MyWhereLT);
  m.def("where_eq", &MyWhereEQ);
  m.def("clip", &MyClip);
  m.def("sqrt", &MySqrt);

  
  py::class_<NeuralNetwork<MyAlgebra>>(      m, "NeuralNetwork")
      .def(py::init<int, bool>())
      .def(py::init<int, const std::vector<int>&,NeuralNetworkActivation, bool >())
      //.def("initialize", &NeuralNetwork<MyAlgebra>::initialize)
      //.def("compute", &NeuralNetwork<MyAlgebra>::compute)
      .def("set_parameters", &NeuralNetwork<MyAlgebra>::set_parameters)
      .def("print_params", &NeuralNetwork<MyAlgebra>::print_params)
      //.def("save_graphviz", &NeuralNetwork<MyAlgebra>::save_graphviz)
      ;
      //.def("compute_contacts", &CollisionDispatcher<MyAlgebra>::compute_contacts2);

  py::class_<NeuralNetworkSpecification>(
      m, "NeuralNetworkSpecification")
      .def(py::init<int, bool>())
      .def(py::init<int, const std::vector<int>&, NeuralNetworkActivation, bool>())
      .def("set_input_dim", &NeuralNetworkSpecification::set_input_dim)
      .def("add_linear_layer", &NeuralNetworkSpecification::add_linear_layer)
      .def("empty", &NeuralNetworkSpecification::empty)
      .def("input_dim", &NeuralNetworkSpecification::input_dim)
      .def("output_dim", &NeuralNetworkSpecification::output_dim)
      .def("num_weights", &NeuralNetworkSpecification::num_weights)
      .def("num_units", &NeuralNetworkSpecification::num_units)
      .def("num_biases", &NeuralNetworkSpecification::num_biases)
      .def("num_parameters", &NeuralNetworkSpecification::num_parameters)
      .def("num_layers", &NeuralNetworkSpecification::num_layers)
      .def("print_states", &NeuralNetworkSpecification::print_states<MyAlgebra>)


      ;

      py::enum_<NeuralNetworkActivation>(m, "NeuralNetworkActivation",
          py::arithmetic())
      .value("NN_ACT_IDENTITY", NN_ACT_IDENTITY)
      .value("NN_ACT_TANH", NN_ACT_TANH)
      .value("NN_ACT_SIN", NN_ACT_SIN)
      .value("NN_ACT_RELU", NN_ACT_RELU)
      .value("NN_ACT_ELU", NN_ACT_ELU)
      .value("NN_ACT_SIGMOID", NN_ACT_SIGMOID)
      .value("NN_ACT_SOFTSIGN", NN_ACT_SOFTSIGN)
      .export_values();

      enum NeuralNetworkInitialization {
          NN_INIT_ZERO = -1,
          NN_INIT_XAVIER = 0,  // good for tanh activations
          NN_INIT_HE,          // good for sigmoid activations
      };

      py::enum_<NeuralNetworkInitialization>(m, "NeuralNetworkInitialization",
          py::arithmetic())
          .value("NN_INIT_ZERO", NN_INIT_ZERO)
          .value("NN_INIT_XAVIER", NN_INIT_XAVIER)
          .value("NN_INIT_HE", NN_INIT_HE)
          .export_values();

  

  py::class_<CollisionDispatcher<MyAlgebra>>(
      m, "TinyCollisionDispatcher")
      .def(py::init<>())
      .def("compute_contacts", &CollisionDispatcher<MyAlgebra>::compute_contacts2);


  py::class_<ContactPoint<MyAlgebra>> contact(m, "TinyContactPoint");
  contact.def(py::init<>())
      .def_readwrite(
          "world_normal_on_b",
          &ContactPoint<MyAlgebra>::world_normal_on_b)
      .def_readwrite("world_point_on_a",
                     &ContactPoint<MyAlgebra>::world_point_on_a)
      .def_readwrite("world_point_on_b",
                     &ContactPoint<MyAlgebra>::world_point_on_b)
      .def_readwrite("distance",
                     &ContactPoint<MyAlgebra>::distance)
      .def_readwrite("normal_force",
                     &ContactPoint<MyAlgebra>::normal_force)
      .def_readwrite("lateral_friction_force_1",
                     &ContactPoint<MyAlgebra>::lateral_friction_force_1)
      .def_readwrite("lateral_friction_force_2",
                     &ContactPoint<MyAlgebra>::lateral_friction_force_2)
      .def_readwrite("fr_direction_1",
                     &ContactPoint<MyAlgebra>::fr_direction_1)
      .def_readwrite("fr_direction_2",
                     &ContactPoint<MyAlgebra>::fr_direction_2);

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
           &MultiBodyConstraintSolver<MyAlgebra>::resolve_collision2)
      .def_readwrite("pgs_iterations_", &MultiBodyConstraintSolver<MyAlgebra>::pgs_iterations_)
      .def_readwrite("keep_all_points_", &MultiBodyConstraintSolver<MyAlgebra>::keep_all_points_)
      .def_readwrite("cfm_", &MultiBodyConstraintSolver<MyAlgebra>::cfm_)
      .def_readwrite("erp_", &MultiBodyConstraintSolver<MyAlgebra>::erp_);

#if 0

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
#endif
  py::class_<World<MyAlgebra>>(m, "TinyWorld")
      .def(py::init<>())
      .def("step", &World<MyAlgebra>::step)
      .def_property("gravity", &World<MyAlgebra>::get_gravity,
                    &World<MyAlgebra>::set_gravity)
      .def("compute_contacts_rigid_body",
           &World<MyAlgebra>::compute_contacts_rigid_body)
      
      .def("compute_contacts_multi_body",
           &World<MyAlgebra>::compute_contacts_multi_body)
      .def("get_collision_dispatcher",
           &World<MyAlgebra>::get_collision_dispatcher)
      .def("get_mb_constraint_solver",
           &World<MyAlgebra>::get_mb_constraint_solver)
      .def("set_mb_constraint_solver",
           &World<MyAlgebra>::set_mb_constraint_solver)
      .def_readwrite("friction",
                     &World<MyAlgebra>::default_friction)
      .def_readwrite("restitution",
                     &World<MyAlgebra>::default_restitution)
      .def_readwrite("num_solver_iterations", &World<MyAlgebra>::num_solver_iterations);

  py::class_<TinyRaycastResult<MyScalar, MyTinyConstants>>(m, "TinyRaycastResult")
      .def(py::init<>())
      .def_readwrite("hit_fraction",
                     &TinyRaycastResult<MyScalar, MyTinyConstants>::m_hit_fraction)
      .def_readwrite("collider_index",
                     &TinyRaycastResult<MyScalar, MyTinyConstants>::m_collider_index);

  py::class_<TinyRaycast<MyScalar, MyTinyConstants, MyAlgebra>>(m, "TinyRaycast")
      .def(py::init<>())
      .def("cast_rays", &TinyRaycast<MyScalar, MyTinyConstants, MyAlgebra>::cast_rays)
      .def("volume", &TinyRaycast<MyScalar, MyTinyConstants, MyAlgebra>::volume)
      .def("intersection_volume",
           &TinyRaycast<MyScalar, MyTinyConstants, MyAlgebra>::intersection_volume);

  ///////////////////////////////////////////////////////////////////////////////////////////


  py::class_<UrdfInertial<MyAlgebra>>(m, "TinyUrdfInertial")
      .def(py::init<>())
      .def_readwrite("mass", &UrdfInertial<MyAlgebra>::mass)
      .def_readwrite("inertia_xxyyzz",
                     &UrdfInertial<MyAlgebra>::inertia_xxyyzz)
      .def_readwrite("origin_xyz",
                     &UrdfInertial<MyAlgebra>::origin_xyz)
      .def_readwrite("origin_rpy",
                     &UrdfInertial<MyAlgebra>::origin_rpy);


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
          &UrdfVisual<MyAlgebra>::visual_shape_uid);
      //.def_readwrite(
      //    "sync_visual_body_uid2",
      //    &UrdfVisual<MyAlgebra>::sync_visual_body_uid2);

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

#ifdef ENABLE_TEST_ENVS

    py::class_<ReacherEnvOutput>(m, "ReacherEnvOutput")
      .def(py::init<>())
      .def_readwrite("obs",
                     &ReacherEnvOutput::obs)
      .def_readwrite("reward",
                     &ReacherEnvOutput::reward)
      .def_readwrite("done",
                     &ReacherEnvOutput::done)

    
      .def_readwrite("body0_graphics_pos",
                     &ReacherEnvOutput::body0_graphics_pos)
      .def_readwrite("body0_graphics_orn",
                     &ReacherEnvOutput::body0_graphics_orn)
      .def_readwrite("body1_graphics_pos",
                     &ReacherEnvOutput::body1_graphics_pos)
      .def_readwrite("body1_graphics_orn",
                     &ReacherEnvOutput::body1_graphics_orn)
      .def_readwrite("tip_graphics_pos",
                     &ReacherEnvOutput::tip_graphics_pos)
      .def_readwrite("tip_graphics_orn",
                     &ReacherEnvOutput::tip_graphics_orn)
      .def_readwrite("target_graphics_pos",
                     &ReacherEnvOutput::target_graphics_pos)
      ;

   py::class_<ReacherRolloutOutput>(m, "ReacherRolloutOutput")
      .def(py::init<>())
      .def_readwrite("total_reward",
                     &ReacherRolloutOutput::total_reward)
      .def_readwrite("num_steps",
                     &ReacherRolloutOutput::num_steps)
      ;
   py::class_<ReacherContactSimulation<MyAlgebra> >(m, "ReacherSimulation")
      .def(py::init<>())
      .def_readwrite("m_urdf_filename",
                     &ReacherContactSimulation<MyAlgebra>::m_urdf_filename)

      ;

     py::class_<ReacherEnv<MyAlgebra>>(m, "ReacherEnv")
      .def(py::init<>())
      .def("reset",&ReacherEnv<MyAlgebra>::reset, py::arg("gravity") = MyAlgebra::Vector3(0.,0.,-10.))
      .def("step", &ReacherEnv<MyAlgebra>::step2)
      .def("rollout", &ReacherEnv<MyAlgebra>::rollout)
      .def("update_weights", &ReacherEnv<MyAlgebra>::init_neural_network)
      .def("seed", &ReacherEnv<MyAlgebra>::seed)
      .def("init_neural_network", &ReacherEnv<MyAlgebra>::init_neural_network)
      .def("policy", &ReacherEnv<MyAlgebra>::policy)
      ;


  py::class_<CartpoleEnvOutput>(m, "CartpoleEnvOutput")
      .def(py::init<>())
      .def_readwrite("obs",
                     &CartpoleEnvOutput::obs)
      .def_readwrite("reward",
                     &CartpoleEnvOutput::reward)
      .def_readwrite("done",
                     &CartpoleEnvOutput::done)
      .def_readwrite("cart_graphics_pos",
                     &CartpoleEnvOutput::cart_graphics_pos)
      .def_readwrite("cart_graphics_orn",
                     &CartpoleEnvOutput::cart_graphics_orn)
      .def_readwrite("pole_graphics_pos",
                     &CartpoleEnvOutput::pole_graphics_pos)
      .def_readwrite("pole_graphics_orn",
                     &CartpoleEnvOutput::pole_graphics_orn)
      
      ;

   py::class_<CartpoleRolloutOutput>(m, "CartpoleRolloutOutput")
      .def(py::init<>())
      .def_readwrite("total_reward",
                     &CartpoleRolloutOutput::total_reward)
      .def_readwrite("num_steps",
                     &CartpoleRolloutOutput::num_steps)
      ;

  py::class_<CartpoleContactSimulation<MyAlgebra>>(m, "CartpoleSimulation")
      .def(py::init<>())
      .def_readwrite("m_urdf_filename",
                     &CartpoleContactSimulation<MyAlgebra>::m_urdf_filename)
      ;

  py::class_<CartpoleEnv<MyAlgebra>>(m, "CartpoleEnv")
      .def(py::init<>())
      .def("reset", &CartpoleEnv<MyAlgebra>::reset2)
      .def("step", &CartpoleEnv<MyAlgebra>::step2)
      .def("rollout", &CartpoleEnv<MyAlgebra>::rollout)
      .def("update_weights", &CartpoleEnv<MyAlgebra>::init_neural_network)
      .def("seed", &CartpoleEnv<MyAlgebra>::seed)
      .def("init_neural_network", &CartpoleEnv<MyAlgebra>::init_neural_network)
      .def("policy", &CartpoleEnv<MyAlgebra>::policy)
      ;
  
   py::class_<AntContactSimulation<MyAlgebra>>(m, "AntContactSimulation")
      .def(py::init<>())
      .def_readwrite("m_urdf_filename",
                     &AntContactSimulation<MyAlgebra>::m_urdf_filename)
      ;

    py::class_<AntEnv<MyAlgebra>>(m, "AntEnv")
      .def(py::init<>())
      .def("reset", &AntEnv<MyAlgebra>::reset)
      .def("step", &AntEnv<MyAlgebra>::step2)
      .def("rollout", &AntEnv<MyAlgebra>::rollout)
      .def("update_weights", &AntEnv<MyAlgebra>::init_neural_network)
      .def("seed", &AntEnv<MyAlgebra>::seed)
      .def("init_neural_network", &AntEnv<MyAlgebra>::init_neural_network)
      .def("policy", &AntEnv<MyAlgebra>::policy)
      ;


    py::class_<VectorizedLaikagoEnvOutput>(m, "VectorizedLaikagoEnvOutput")
        .def(py::init<>())

        .def_readwrite("obs", &VectorizedLaikagoEnvOutput::obs)
        .def_readwrite("rewards", &VectorizedLaikagoEnvOutput::rewards)
        .def_readwrite("dones", &VectorizedLaikagoEnvOutput::dones)
        .def_readwrite(
            "visual_world_transforms",
            &VectorizedLaikagoEnvOutput::visual_world_transforms)
            
        ;


     py::class_<VectorizedLaikagoEnv>(m, "VectorizedLaikagoEnv")
        .def(py::init<int, bool>())
        .def("reset", &VectorizedLaikagoEnv::reset)
        .def("step", &VectorizedLaikagoEnv::step)
        .def("action_dim", &VectorizedLaikagoEnv::action_dim) 
        .def("obs_dim", &VectorizedLaikagoEnv::obs_dim) 
        .def("urdf_filename", &VectorizedLaikagoEnv::urdf_filename) 
         ;
    

    py::class_<VectorizedAntEnvOutput>(m, "VectorizedAntEnvOutput")
        .def(py::init<>())
        .def_readwrite("obs", &VectorizedAntEnvOutput::obs)
        .def_readwrite("rewards", &VectorizedAntEnvOutput::rewards)
        .def_readwrite("dones", &VectorizedAntEnvOutput::dones)
         .def_readwrite("visual_world_transforms",
                        &VectorizedAntEnvOutput::visual_world_transforms)
        ;


     py::class_<VectorizedAntEnv>(m, "VectorizedAntEnv")
        .def(py::init<int, bool>())
        .def("reset", &VectorizedAntEnv::reset)
        .def("step", &VectorizedAntEnv::step)
        .def("action_dim", &VectorizedAntEnv::action_dim) 
        .def("obs_dim", &VectorizedAntEnv::obs_dim) 
        .def("urdf_filename", &VectorizedAntEnv::urdf_filename) 
         ;
    

     


#endif//ENABLE_TEST_ENVS

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

