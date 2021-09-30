
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
//#include "mb_constraint_solver_spring.hpp"
#include "multi_body.hpp"
#include "math/pose.hpp"
#include "math/tiny/tiny_quaternion.h"
#include "math/tiny/tiny_raycast.h"
#include "urdf_structures.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "world.hpp"
#include "tiny_inverse_kinematics.h"
#include "dynamics/mass_matrix.hpp"
#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "dynamics/jacobian.hpp"
#include "math/neural_network.hpp"
#include "utils/file_utils.hpp"
#ifdef ENABLE_TEST_ENVS
#include "examples/environments/cartpole_environment.h"
#include "examples/environments/ant_environment.h"
#include "examples/environments/reacher_environment.h"
#endif//ENABLE_CARTPOLE_TEST_ENV

#ifdef _WIN32
#undef min
#undef max
#endif

template <typename Algebra>
struct UrdfToMultiBody2 {
    typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;

    void convert(TinyUrdfStructures* urdf_structures,
        ::tds::World<Algebra>* world,
        ::tds::MultiBody<Algebra>* mb) {
        ::tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(
            *urdf_structures, *world, *mb, 0);

        mb->initialize();
    }
};

inline MyScalar fraction(int a, int b)
{
    return MyTinyConstants::fraction(a, b);
}

inline MyScalar copy(MyScalar v)
{
    return MyTinyConstants::copy(v);
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

inline tds::Transform<MyAlgebra> MyGetLinkTransformInBase(tds::MultiBody<MyAlgebra>& mb, const MyAlgebra::VectorX& q, int link_index)
{
    tds::Transform<MyAlgebra> base_X_world;
    std::vector<tds::Transform<MyAlgebra> > links_X_world;
    std::vector<tds::Transform<MyAlgebra> > links_X_base;

    forward_kinematics_q(mb, q, &base_X_world, &links_X_world, &links_X_base);
    //forward_kinematics_q< TinyAlgebra < Scalar, Utils> >(mb, q, &base_X_world, &links_X_world, &links_X_base);
    return links_X_base[link_index];
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

inline MyAlgebra::Quaternion MyQuatIntegrate(const MyAlgebra::Quaternion& start_orn, const MyAlgebra::Vector3& ang_vel, MyScalar dt)
{
    MyAlgebra::Quaternion orn = start_orn;
    MyAlgebra::Quaternion orn2 = MyAlgebra::quat_velocity(orn, ang_vel, dt);
    MyAlgebra::quat_increment(orn, orn2);
    orn = MyAlgebra::normalize(orn);
    return orn;
}



inline MyAlgebra::Matrix3X MyPointJacobian(tds::MultiBody<MyAlgebra>& mb, int link_index, const MyAlgebra::Vector3& point, bool is_local)
{
    return tds::point_jacobian2(mb, link_index, point, is_local);
}

inline MyAlgebra::VectorX MyInverseKinematics(tds::MultiBody<MyAlgebra>& mb, int target_link_index, const MyAlgebra::Vector3& target_point)
{
    MyAlgebra::VectorX q_target;
#ifdef USE_IK_JAC_TRANSPOSE
    TINY::TinyInverseKinematics<MyAlgebra, TINY::IK_JAC_TRANSPOSE> inverse_kinematics;
#else    
    TINY::TinyInverseKinematics<MyAlgebra, TINY::IK_JAC_PINV> inverse_kinematics;
#endif

    inverse_kinematics.weight_reference = MyTinyConstants::fraction(0,10);
    // step size
    inverse_kinematics.alpha = MyTinyConstants::fraction(3, 10);
    inverse_kinematics.targets.emplace_back(target_link_index, target_point);
    inverse_kinematics.q_reference = mb.q_;
    inverse_kinematics.compute(mb, mb.q_, q_target);
    return q_target;
}

std::string MyFindFile(std::string& org_name)
{
    std::string result_path;
    bool has_result = tds::FileUtils::find_file(org_name, result_path);
    return result_path;
}

MyScalar MyPi()
{
    return MyAlgebra::pi();
}

MyScalar MyCos(MyScalar v)
{
    return MyAlgebra::cos(v);
}

MyScalar MyAcos(MyScalar v)
{
    return MyAlgebra::acos(v);
}

MyScalar MySin(MyScalar v)
{
    return MyAlgebra::sin(v);
}

MyScalar MyMax(MyScalar a, MyScalar b)
{
    return MyAlgebra::max(a, b);
}

MyScalar MyMin(MyScalar a, MyScalar b)
{
    return MyAlgebra::min(a, b);
}

MyScalar MyWhereGT(MyScalar a, MyScalar b, MyScalar x, MyScalar y) {
    return tds::where_gt(a, b, x, y);
}

MyScalar MyWhereLT(MyScalar a, MyScalar b, MyScalar x, MyScalar y) {
    return tds::where_lt(a, b, x, y);
}

MyScalar MyWhereEQ(MyScalar a, MyScalar b, MyScalar x, MyScalar y) {
    return tds::where_eq(a, b, x, y);
}

MyScalar MyClip(MyScalar v, MyScalar low, MyScalar high) {
    return MyAlgebra::max(MyAlgebra::min(v, high), low);
}

MyScalar MySqrt(MyScalar v) {
    return MyAlgebra::sqrt(v);
}

MyAlgebra::Quaternion QuaternionDifference(const MyAlgebra::Quaternion &q_start, const MyAlgebra::Quaternion &q_end) {
    return MyAlgebra::quat_difference(q_start, q_end);
}

