
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
#include "math/matrix_utils.hpp"
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
#include "examples/environments/cartpole_environment2.h"
#include "examples/environments/ant_environment.h"
#include "examples/environments/reacher_environment.h"


#include "examples/environments/ant_environment2.h"
#include "examples/ars/ars_vectorized_environment.h"
#include "examples/environments/laikago_environment2.h"

typedef AntEnv2<MyAlgebra> LocomotionEnvironment2;
typedef VectorizedEnvironment<MyAlgebra, AntContactSimulation2<MyAlgebra>>    VecEnvironment2;

typedef LaikagoEnv<MyAlgebra> LaikagoLocomotionEnvironment;
typedef VectorizedEnvironment<MyAlgebra, LaikagoContactSimulation<MyAlgebra>> LaikagoVecEnvironment;



struct VectorizedAntEnvOutput {
  std::vector < std::vector<float>> obs;
  std::vector<float> rewards;
  std::vector < float> dones;
  std::vector<std::vector<float>> visual_world_transforms;
};

class VectorizedAntEnv {

    LocomotionEnvironment2 locenv;
    VecEnvironment2 venv;
    ARSConfig config;
   public:
    VectorizedAntEnv(int num_envs, bool auto_reset_when_done)
        : locenv(true), venv(locenv.contact_sim, num_envs) { 
        config.batch_size = num_envs;
        config.auto_reset_when_done = auto_reset_when_done;
        printf("VectorizedAntEnv\n");

        //VecEnvironment vec_env(locomotion_simenv.contact_sim, batch_size);
        // vec_env.default_stepper_ = &vec_env.serial_stepper_;

    }

    virtual ~VectorizedAntEnv() {
      printf("~VectorizedAntEnv\n");
    }
  
    std::string urdf_filename() { return venv.contact_sim.urdf_filename_; }

    int action_dim() const { 
        return venv.contact_sim.action_dim();    
    }

    int obs_dim() const { 
        int od = venv.contact_sim.input_dim();//28, x,y,z torso world pos, x,y,z world rotation, q(8) and qd (8)
      return od;
    }
  std::vector<std::vector<double> >reset() 
  { 
    return venv.reset(config);
  }

  VectorizedAntEnvOutput step(std::vector<std::vector<double>> actions) 
  { 
    std::vector<std::vector<double>> obs;
    std::vector<double> rewards;
    std::vector<bool> dones;
    obs.resize(config.batch_size);
    for (int i = 0; i < obs.size(); i++) {
      obs[i].resize(obs_dim());
    }
  
    rewards.resize(config.batch_size);
    dones.resize(config.batch_size);
    venv.step(actions, obs, rewards, dones, config);

    VectorizedAntEnvOutput output;
    output.obs.resize(obs.size());
    for (int i = 0; i < obs.size(); i++) {
      output.obs[i].resize(obs[i].size());
      for (int j = 0; j < obs[i].size(); j++) {
        output.obs[i][j] = float(obs[i][j]);
      }
    }
    output.rewards.resize(rewards.size());
    for (int i = 0; i < rewards.size(); i++) {
      output.rewards[i] = float(rewards[i]);
    }
    output.dones.resize(dones.size());

    for (int i = 0; i < dones.size(); i++) {
      output.dones[i] = dones[i]? 1.f : 0.f;
    }

    output.visual_world_transforms.resize(
        venv.sim_states_with_graphics_.size());

    for (int i = 0; i < venv.sim_states_with_graphics_.size(); i++) {
      output.visual_world_transforms[i].resize(
          venv.sim_states_with_graphics_[i].size());
      for (int j = 0; j < venv.sim_states_with_graphics_[i].size(); j++) {
        output.visual_world_transforms[i][j] =
            float(venv.sim_states_with_graphics_[i][j]);
      }
    }

    return output;
  }

};




struct VectorizedLaikagoEnvOutput {
  std::vector < std::vector<float>> obs;
  std::vector<float> rewards;
  std::vector < float> dones;
  std::vector < std::vector<float>> visual_world_transforms;
};

class VectorizedLaikagoEnv {
  LaikagoLocomotionEnvironment locenv;
  LaikagoVecEnvironment venv;
  ARSConfig config;

 public:
  VectorizedLaikagoEnv(int num_envs, bool auto_reset_when_done)
      : locenv(true), venv(locenv.contact_sim, num_envs) {
    config.batch_size = num_envs;
    config.auto_reset_when_done = auto_reset_when_done;
    printf("VectorizedLaikagoEnv\n");
  }

  virtual ~VectorizedLaikagoEnv() { printf("~VectorizedLaikagoEnv\n"); }

  int action_dim() const { return venv.contact_sim.action_dim(); }

  int obs_dim() const {
    int od = venv.contact_sim.input_dim();  // 28, x,y,z torso world pos, x,y,z
                                            // world rotation, q(8) and qd (8)
    return od;
  }
  std::vector<std::vector<double>> reset() { return venv.reset(config); }

  std::string urdf_filename() { 
      return venv.contact_sim.urdf_filename_;
  }

  VectorizedLaikagoEnvOutput step(std::vector<std::vector<double>> actions) 
  { 
    std::vector<std::vector<double>> obs;
    std::vector<double> rewards;
    std::vector<bool> dones;
    obs.resize(config.batch_size);
    for (int i = 0; i < obs.size(); i++) {
      obs[i].resize(obs_dim());
    }
  
    rewards.resize(config.batch_size);
    dones.resize(config.batch_size);
    venv.step(actions, obs, rewards, dones, config);

    VectorizedLaikagoEnvOutput output;
    output.obs.resize(obs.size());
    for (int i = 0; i < obs.size(); i++) {
      output.obs[i].resize(obs[i].size());
      for (int j = 0; j < obs[i].size(); j++) {
        output.obs[i][j] = float(obs[i][j]);
      }
    }
    output.rewards.resize(rewards.size());
    for (int i = 0; i < rewards.size(); i++) {
      output.rewards[i] = float(rewards[i]);
    }
    output.dones.resize(dones.size());

    for (int i = 0; i < dones.size(); i++) {
      output.dones[i] = dones[i]? 1.f : 0.f;
    }

    output.visual_world_transforms.resize(
        venv.sim_states_with_graphics_.size());

    for (int i=0;i<venv.sim_states_with_graphics_.size();i++) {
      output.visual_world_transforms[i].resize(
          venv.sim_states_with_graphics_[i].size());
      for (int j = 0; j < venv.sim_states_with_graphics_[i].size(); j++) {
        output.visual_world_transforms[i][j] =
            float(venv.sim_states_with_graphics_[i][j]);
      }
    }

    return output;
  }

};




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

MyAlgebra::Vector3 Quaternion_Axis_Angle(const MyAlgebra::Quaternion &q) {
    return MyAlgebra::quaternion_axis_angle(q);
}

const std::vector<const tds::Geometry<MyAlgebra> *>* mb_collision_geometries(
        const tds::MultiBody<MyAlgebra>& mb, int link_id) {
    /* Returns a cloned copies of the collision geometries */
    const std::vector<const tds::Geometry<MyAlgebra> *> colls = mb.collision_geometries(link_id);
    std::vector<const tds::Geometry<MyAlgebra> *> *collision_geometries = new std::vector<const tds::Geometry<MyAlgebra> *>;
    for (const auto *geom : colls) {
      collision_geometries->push_back(tds::clone<MyAlgebra, MyAlgebra>(geom));
    }
    return collision_geometries;
}




