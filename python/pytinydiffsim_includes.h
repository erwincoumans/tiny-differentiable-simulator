
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>


#include "math/tiny/tiny_vector3.h"
#include "geometry.hpp"
#include "rigid_body.hpp"
//#include "examples/motion_import.h"
#include "urdf/urdf_parser.hpp"

#include "math/tiny/tiny_matrix3x3.h"
#include "math/tiny/tiny_matrix_x.h"
#include "mb_constraint_solver_spring.hpp"
#include "multi_body.hpp"
#include "math/pose.hpp"
#include "math/tiny/tiny_quaternion.h"
#include "math/tiny/tiny_raycast.h"
#include "urdf/urdf_structures.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "world.hpp"



template <typename Algebra>
struct UrdfToMultiBody2 {
    typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;

    void convert(TinyUrdfStructures* urdf_structures,
        ::tds::World<Algebra>* world,
        ::tds::MultiBody<Algebra>* mb) {
        ::tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(
            *urdf_structures, *world, *mb);

        mb->initialize();
    }
};

inline MyScalar fraction(int a, int b)
{
    return MyTinyConstants::fraction(a, b);
}

inline void MyMassMatrix(tds::MultiBody<MyAlgebra>& mb, MyAlgebra::VectorX& q,
    MyAlgebra::MatrixX* M)
{
    mass_matrix(mb, q, M);
}

inline void MyForwardKinematics(tds::MultiBody<MyAlgebra>& mb, const MyAlgebra::VectorX& q, const MyAlgebra::VectorX& qd)
{
    forward_kinematics(mb, q, qd);
}

inline void MyForwardDynamics(tds::MultiBody<MyAlgebra>& mb, const MyAlgebra::Vector3& gravity)
{
    forward_dynamics(mb, gravity);
}
inline void MyIntegrateEuler(tds::MultiBody<MyAlgebra>& mb, const MyScalar& dt)
{
    integrate_euler(mb, dt);
}

inline void MyIntegrateEulerQdd(tds::MultiBody<MyAlgebra>& mb, const MyScalar& dt)
{
    integrate_euler_qdd(mb, dt);
}

inline tds::RigidBodyInertia<MyAlgebra> MyComputeInertia(const MyScalar& mass,
    const MyAlgebra::Vector3& com, const MyAlgebra::Matrix3& inertia)
{
    tds::RigidBodyInertia< MyAlgebra> rb_inertia(mass, com, inertia);
    return rb_inertia;
}

inline MyAlgebra::Matrix3X MyPointJacobian(tds::MultiBody<MyAlgebra>& mb, int link_index, const MyAlgebra::Vector3& point, bool is_local)
{
    return tds::point_jacobian2(mb, link_index, point, is_local);
}
