#include <cassert>
#include <chrono>
#include <cstdio>
#include <map>
#include <thread>

// #define DEBUG 1
//#define VERBOSE_PRINT
// #define USE_MATPLOTLIB 1

#include "base.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "dynamics/jacobian.hpp"
#include "dynamics/kinematics.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "multi_body.hpp"
#ifdef USE_BULLET_URDF_PARSER
#include "urdf/urdf_cache.hpp"
#else
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#endif  // USE_BULLET_URDF_PARSER
#include "utils/file_utils.hpp"
#include "world.hpp"

#if USE_RBDL
#include "rbdl/Dynamics.h"
#include "rbdl/Model.h"
#include "rbdl/rbdl.h"
#endif

using namespace tds;

namespace {
    const double kRbdlTestErrorTolerance = 1e-10;
    //const double kRbdlTestErrorTolerance = 1e-1;
}

#if USE_RBDL
template <typename Algebra>
RigidBodyDynamics::Math::Vector3d to_rbdl(const typename Algebra::Vector3 &v) {
  return RigidBodyDynamics::Math::Vector3d(Algebra::to_double(v[0]),
                                           Algebra::to_double(v[1]),
                                           Algebra::to_double(v[2]));
}

template <typename Algebra>
RigidBodyDynamics::Math::Matrix3d to_rbdl(const typename Algebra::Matrix3 &m) {
  return RigidBodyDynamics::Math::Matrix3d(
      Algebra::to_double(m(0, 0)), Algebra::to_double(m(0, 1)),
      Algebra::to_double(m(0, 2)), Algebra::to_double(m(1, 0)),
      Algebra::to_double(m(1, 1)), Algebra::to_double(m(1, 2)),
      Algebra::to_double(m(2, 0)), Algebra::to_double(m(2, 1)),
      Algebra::to_double(m(2, 2)));
}

template <typename Algebra>
RigidBodyDynamics::Math::SpatialVector to_rbdl(
    const SpatialVector<Algebra> &v) {
  return RigidBodyDynamics::Math::SpatialVector(
      Algebra::to_double(v[0]), Algebra::to_double(v[1]),
      Algebra::to_double(v[2]), Algebra::to_double(v[3]),
      Algebra::to_double(v[4]), Algebra::to_double(v[5]));
}

template <typename Algebra>
RigidBodyDynamics::Math::SpatialTransform to_rbdl(
    const Transform<Algebra> &tf) {
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
  return RigidBodyDynamics::Math::SpatialTransform(
      to_rbdl<Algebra>(tf.rotation), to_rbdl<Algebra>(tf.translation));
#else
  // Transform<Algebra> inv_tf = tf.inverse();
  const auto rot = to_rbdl<Algebra>(tf.rotation);
  return RigidBodyDynamics::Math::SpatialTransform(
      rot.transpose(), to_rbdl<Algebra>(tf.translation));
#endif
}

template <typename Algebra>
RigidBodyDynamics::Model to_rbdl(const MultiBody<Algebra> &mb) {
  RigidBodyDynamics::Model model;
  // maps TDS link IDs to RBDL link IDs
  std::map<int, unsigned int> tds_to_rbdl;
  if (mb.is_floating()) {
    RigidBodyDynamics::Body body(Algebra::to_double(mb.base_rbi().mass),
                                 to_rbdl<Algebra>(mb.base_rbi().com),
                                 to_rbdl<Algebra>(mb.base_rbi().inertia));
    RigidBodyDynamics::Joint joint;
    joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFloatingBase);
    unsigned int rbdl_id = model.AddBody(
        0, to_rbdl<Algebra>(tds::Transform<Algebra>()), joint, body);
#ifdef VERBOSE_PRINT
    std::cout << "RBDL floating-base ID: " << rbdl_id << std::endl;
#endif
    tds_to_rbdl[-1] = rbdl_id;
  }
  for (const Link<Algebra> &link : mb) {
    RigidBodyDynamics::Body body(Algebra::to_double(link.rbi.mass),
                                 to_rbdl<Algebra>(link.rbi.com),
                                 to_rbdl<Algebra>(link.rbi.inertia));
    RigidBodyDynamics::Joint joint;
    switch (link.joint_type) {
      case JOINT_REVOLUTE_X:
      case JOINT_REVOLUTE_Y:
      case JOINT_REVOLUTE_Z:
      case JOINT_REVOLUTE_AXIS:
        joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,
                                         to_rbdl<Algebra>(link.S.top));
        break;
      case JOINT_PRISMATIC_X:
      case JOINT_PRISMATIC_Y:
      case JOINT_PRISMATIC_Z:
      case JOINT_PRISMATIC_AXIS:
        joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypePrismatic,
                                         to_rbdl<Algebra>(link.S.bottom));
        break;
      case JOINT_FIXED:
        joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);
        break;
      default:
        joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeUndefined);
        break;
    }
    unsigned int rbdl_id =
        model.AddBody(tds_to_rbdl[link.parent_index],
                      to_rbdl<Algebra>(link.X_T), joint, body);
    tds_to_rbdl[link.index] = rbdl_id;
  }

#ifdef VERBOSE_PRINT
  std::cout << "TDS to RBDL link index mapping:\n";
  for (const auto &[t, r] : tds_to_rbdl) {
    printf("\t%i -> %u (%s)\n", t, r,
           t < 0
               ? "Base"
               : MultiBody<Algebra>::joint_type_name(mb[t].joint_type).c_str());
  }
#endif
  return model;
}

template <typename Algebra>
bool is_equal(const SpatialVector<Algebra> &a,
              const RigidBodyDynamics::Math::SpatialVector &b) {
  for (int i = 0; i < 6; ++i) {
    if (std::abs(Algebra::to_double(a[i]) - b[i]) > kRbdlTestErrorTolerance) {
      std::cout << "a[" << i << "] = " << Algebra::to_double(a[i]);
      std::cout << "\tb[" << i << "] = " << b[i];
      std::cout << "\terror = " << std::abs(Algebra::to_double(a[i]) - b[i])
                << std::endl;
      return false;
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::Scalar &a, double b) {
  if (std::abs(Algebra::to_double(a) - b) > kRbdlTestErrorTolerance) {
    std::cout << "a = " << Algebra::to_double(a);
    std::cout << "\tb = " << b;
    std::cout << "\terror = " << std::abs(Algebra::to_double(a) - b)
              << std::endl;
    return false;
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::Vector3 &a,
              const RigidBodyDynamics::Math::Vector3d &b) {
  for (int i = 0; i < 3; ++i) {
    if (std::abs(Algebra::to_double(a[i]) - b[i]) > kRbdlTestErrorTolerance) {
      std::cout << "a[" << i << "] = " << Algebra::to_double(a[i]);
      std::cout << "\tb[" << i << "] = " << b[i];
      std::cout << "\terror = " << std::abs(Algebra::to_double(a[i]) - b[i])
                << std::endl;
      return false;
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::VectorX &a,
              const RigidBodyDynamics::Math::VectorNd &b) {
  for (typename Algebra::Index i = 0; i < Algebra::size(a); ++i) {
    if (std::abs(Algebra::to_double(a[i]) - b[i]) > kRbdlTestErrorTolerance) {
      std::cout << "a[" << i << "] = " << Algebra::to_double(a[i]);
      std::cout << "\tb[" << i << "] = " << b[i];
      std::cout << "\terror = " << std::abs(Algebra::to_double(a[i]) - b[i])
                << std::endl;
      return false;
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::Matrix3 &a,
              const RigidBodyDynamics::Math::Matrix3d &b) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) >
          kRbdlTestErrorTolerance) {
#ifdef VERBOSE_PRINT
        std::cout << "a[" << i << "," << j
                  << "] = " << Algebra::to_double(a(i, j));
        std::cout << "\tb[" << i << "," << j << "] = " << b(i, j);
        std::cout << "\terror = "
                  << std::abs(Algebra::to_double(a(i, j)) - b(i, j))
                  << std::endl;
        Algebra::print("%%% a:", a);
        std::cout << "%%% b:\n" << b << "\n\n";
#endif//
        return false;
      }
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::Matrix6 &a,
              const RigidBodyDynamics::Math::SpatialMatrix &b) {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) >
          kRbdlTestErrorTolerance) {
        std::cout << "a[" << i << "," << j
                  << "] = " << Algebra::to_double(a(i, j));
        std::cout << "\tb[" << i << "," << j << "] = " << b(i, j);
        std::cout << "\terror = "
                  << std::abs(Algebra::to_double(a(i, j)) - b(i, j))
                  << std::endl;
        return false;
      }
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::MatrixX &a,
              const RigidBodyDynamics::Math::MatrixNd &b) {
  for (int i = 0; i < Algebra::num_rows(a); ++i) {
    for (int j = 0; j < Algebra::num_cols(a); ++j) {
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) >
          kRbdlTestErrorTolerance) {
        std::cout << "a[" << i << "," << j
                  << "] = " << Algebra::to_double(a(i, j));
        std::cout << "\tb[" << i << "," << j << "] = " << b(i, j);
        std::cout << "\terror = "
                  << std::abs(Algebra::to_double(a(i, j)) - b(i, j))
                  << std::endl;
        return false;
      }
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const typename Algebra::Matrix3X &a,
              const RigidBodyDynamics::Math::MatrixNd &b) {
  for (int i = 0; i < Algebra::num_rows(a); ++i) {
    for (int j = 0; j < Algebra::num_cols(a); ++j) {
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) >
          kRbdlTestErrorTolerance) {
        std::cout << "a[" << i << "," << j
                  << "] = " << Algebra::to_double(a(i, j));
        std::cout << "\tb[" << i << "," << j << "] = " << b(i, j);
        std::cout << "\terror = "
                  << std::abs(Algebra::to_double(a(i, j)) - b(i, j))
                  << std::endl;
        return false;
      }
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const Transform<Algebra> &a,
              const RigidBodyDynamics::Math::SpatialTransform &b) {
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
  return is_equal<Algebra>(a.translation, b.r) &&
         is_equal<Algebra>(a.rotation, b.E);
#else
  return is_equal<Algebra>(a.translation, b.r) &&
         is_equal<Algebra>(a.rotation, b.E.transpose().eval());
#endif
}

template <typename Algebra>
bool is_equal(const RigidBodyInertia<Algebra> &a,
              const RigidBodyDynamics::Math::SpatialRigidBodyInertia &b) {
  auto am = ArticulatedBodyInertia<Algebra>(a).matrix();
  auto bm = b.toMatrix();
  return is_equal<Algebra>(am, bm);
}

template <typename Algebra>
bool is_equal(const ArticulatedBodyInertia<Algebra> &a,
              const RigidBodyDynamics::Math::SpatialMatrix &b) {
  auto ma = a.matrix();
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (std::abs(Algebra::to_double(ma(i, j)) - b(i, j)) >
          kRbdlTestErrorTolerance) {
        std::cout << "a[" << i << "," << j
                  << "] = " << Algebra::to_double(a(i, j));
        std::cout << "\tb[" << i << "," << j << "] = " << b(i, j);
        std::cout << "\terror = "
                  << std::abs(Algebra::to_double(a(i, j)) - b(i, j))
                  << std::endl;
        return false;
      }
    }
  }
  return true;
}

template <typename Algebra>
bool is_equal(const MultiBody<Algebra> &tds,
              const RigidBodyDynamics::Model &rbdl,
              bool check_dynamics = true) {
#ifdef VERBOSE_PRINT
  std::cout << "RBDL velocity [0]:\n" << rbdl.v[0].transpose() << std::endl;
  std::cout << "RBDL velocity [1]:\n" << rbdl.v[1].transpose() << std::endl;
  std::cout << "RBDL velocity [2]:\n" << rbdl.v[2].transpose() << std::endl;
#endif
  if (tds.is_floating()) {
    if (!is_equal<Algebra>(tds.base_X_world(), rbdl.X_base[2])) {
      fprintf(stderr, "Mismatch in X_base (X_world) at floating base.\n");
      Algebra::print("TDS:  ", tds.base_X_world());
      std::cout << "RBDL:\n" << rbdl.X_base[2] << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds.base_velocity(), rbdl.v[2])) {
      fprintf(stderr, "Mismatch in v at floating base.\n");
      Algebra::print("TDS:  ", tds.base_velocity());
      std::cout << "RBDL: " << rbdl.v[2].transpose() << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds.base_abi(), rbdl.IA[2])) {
      fprintf(stderr, "Mismatch in ABI at floating base.\n");
      Algebra::print("TDS:  ", tds.base_abi());
      std::cout << "RBDL:\n" << rbdl.IA[2] << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds.base_bias_force(), rbdl.pA[2])) {
      fprintf(stderr, "Mismatch in pA at floating base.\n");
      Algebra::print("TDS:  ", tds.base_bias_force());
      std::cout << "RBDL: " << rbdl.pA[2].transpose() << std::endl;
      return false;
    }
  }

  // std::cout << "RBDL ABI[0]:\n" << rbdl.IA[0] << std::endl;
  // std::cout << "RBDL ABI[1]:\n" << rbdl.IA[1] << std::endl;
  #ifdef VERBOSE_PRINT
  std::cout << "RBDL ABI[2]:\n" << rbdl.IA[2] << std::endl;
  Algebra::print("TDS Base ABI:", tds.base_abi());
  // std::cout << "RBDL pA[0]:\n" << rbdl.pA[0].transpose() << std::endl;
  // std::cout << "RBDL pA[1]:\n" << rbdl.pA[1].transpose() << std::endl;
  std::cout << "RBDL pA[2]:\n" << rbdl.pA[2].transpose() << std::endl;
  Algebra::print("TDS Base pA:", tds.base_bias_force());
  // std::cout << "RBDL acceleration [0]:\n" << rbdl.a[0].transpose() <<
  // std::endl; std::cout << "RBDL acceleration [1]:\n" << rbdl.a[1].transpose()
  // << std::endl; std::cout << "RBDL acceleration [2]:\n" <<
  // rbdl.a[2].transpose() << std::endl;


  std::cout << "RBDL X_base[2]:\n" << rbdl.X_base[2] << std::endl;
  Algebra::print("TDS Base base_X_world:", tds.base_X_world());
#endif
  // floating-base models in RBDL have a TranslationXYZ and a spherical joint
  // first
  const std::size_t id_offset = tds.is_floating() ? 3 : 1;
  for (std::size_t j = 0; j < tds.size(); ++j) {
    std::size_t rbdl_j = j + id_offset;
    // X_T must match literally no matter the transform associativity
    if (!is_equal<Algebra>(tds[j].X_T, rbdl.X_T[rbdl_j])) {
      fprintf(stderr, "Mismatch in X_T at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].X_T);
      std::cout << "RBDL:\n" << rbdl.X_T[rbdl_j] << std::endl;
      return false;
      std::cout << "\n";
    }
    if (!is_equal<Algebra>(tds[j].X_parent, rbdl.X_lambda[rbdl_j])) {
      fprintf(stderr, "Mismatch in X_lambda (X_parent) at link %i.\n",
              static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].X_parent);
      std::cout << "RBDL:\n" << rbdl.X_lambda[rbdl_j] << std::endl;
      return false;
      std::cout << "\n";
    }
    if (!is_equal<Algebra>(tds[j].X_world, rbdl.X_base[rbdl_j])) {
      fprintf(stderr, "Mismatch in X_base (X_world) at link %i.\n",
              static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].X_world);
      std::cout << "RBDL:\n" << rbdl.X_base[rbdl_j] << std::endl;
      std::cout << "\n";
      return false;
    }
    if (!is_equal<Algebra>(tds[j].S, rbdl.S[rbdl_j])) {
      fprintf(stderr, "Mismatch in S at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].S);
      std::cout << "RBDL: " << rbdl.S[rbdl_j].transpose() << std::endl;
      std::cout << "\n";
      return false;
    }
    if (!is_equal<Algebra>(tds[j].v, rbdl.v[rbdl_j])) {
      fprintf(stderr, "Mismatch in v at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].v);
      std::cout << "RBDL: " << rbdl.v[rbdl_j].transpose() << std::endl;
      return false;
    }
    if (check_dynamics) {
      if (!is_equal<Algebra>(tds[j].c, rbdl.c[rbdl_j])) {
        fprintf(stderr, "Mismatch in c at link %i.\n", static_cast<int>(j));
        Algebra::print("TDS:  ", tds[j].c);
        std::cout << "RBDL: " << rbdl.c[rbdl_j].transpose() << std::endl;
        return false;
      }
    }
    if (!is_equal<Algebra>(tds[j].rbi, rbdl.Ic[rbdl_j])) {
      fprintf(stderr, "Mismatch in RBI at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].rbi);
      std::cout << "RBDL: " << rbdl.Ic[rbdl_j] << std::endl;
      return false;
    }
  }

  if (check_dynamics) {
    // check quantities from second (reverse) loop in ABA
    for (int j = static_cast<int>(tds.size()) - 1; j >= 0; --j) {
      const int rbdl_j = j + id_offset;
      if (!is_equal<Algebra>(tds[j].abi, rbdl.IA[rbdl_j])) {
        fprintf(stderr, "Mismatch in ABI at link %i.\n", j);
        Algebra::print("TDS:  ", tds[j].abi);
        std::cout << "RBDL:\n" << rbdl.IA[rbdl_j] << std::endl;
        return false;
      }
      if (!is_equal<Algebra>(tds[j].U, rbdl.U[rbdl_j])) {
        fprintf(stderr, "Mismatch in U at link %i.\n", j);
        Algebra::print("TDS:  ", tds[j].U);
        std::cout << "RBDL: " << rbdl.U[rbdl_j].transpose() << std::endl;
        return false;
      }
      if (!is_equal<Algebra>(tds[j].pA, rbdl.pA[rbdl_j])) {
        fprintf(stderr, "Mismatch in pA at link %i.\n", j);
        Algebra::print("TDS:  ", tds[j].pA);
        std::cout << "RBDL: " << rbdl.pA[rbdl_j].transpose() << std::endl;
        return false;
      }
      if (!is_equal<Algebra>(tds[j].u, rbdl.u[rbdl_j])) {
        fprintf(stderr, "Mismatch in u at link %i.\n", j);
        Algebra::print("TDS:  ", tds[j].u);
        std::cout << "RBDL: " << rbdl.u[rbdl_j] << std::endl;
        return false;
      }
      if (!is_equal<Algebra>(tds[j].D, rbdl.d[rbdl_j])) {
        fprintf(stderr, "Mismatch in D at link %i.\n", j);
        Algebra::print("TDS:  ", tds[j].D);
        std::cout << "RBDL: " << rbdl.d[rbdl_j] << std::endl;
        return false;
      }
    }
  }

  if (tds.is_floating()) {
    if (!is_equal<Algebra>(tds.base_acceleration(), rbdl.a[2])) {
      fprintf(stderr, "Mismatch in a at floating base.\n");
      Algebra::print("TDS:  ", tds.base_acceleration());
      std::cout << "RBDL: " << rbdl.a[2].transpose() << std::endl;
      return false;
    }
  }

  for (std::size_t j = 0; j < tds.size(); ++j) {
    std::size_t rbdl_j = j + id_offset;
    if (!is_equal<Algebra>(tds[j].a, rbdl.a[rbdl_j])) {
      fprintf(stderr, "Mismatch in a at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].a);
      std::cout << "RBDL: " << rbdl.a[rbdl_j].transpose() << std::endl;
      return false;
    }
  }

  return true;
}

template <typename Algebra>
bool is_equal(const MultiBody<Algebra> &tds,
              const RigidBodyDynamics::Math::VectorNd &q,
              const RigidBodyDynamics::Math::VectorNd &qd,
              const RigidBodyDynamics::Math::VectorNd &qdd) {
  RigidBodyDynamics::Math::VectorNd rbdl_q = q;
  RigidBodyDynamics::Math::VectorNd rbdl_qd = qd;
  RigidBodyDynamics::Math::VectorNd rbdl_qdd = qdd;

  if (tds.is_floating()) {
    // change order to have the dimensions line up for floating-base systems
    rbdl_q[4] = q[0];
    rbdl_q[5] = q[1];
    rbdl_q[6] = q[2];

    // w coordinate of quat is stored at the end
    rbdl_q[3] = q[tds.dof() - 1];
    rbdl_q[0] = q[3];
    rbdl_q[1] = q[4];
    rbdl_q[2] = q[5];
    for (int i = 7; i < tds.dof(); ++i) {
      rbdl_q[i] = q[i - 1];
    }
    rbdl_qd[3] = qd[0];
    rbdl_qd[4] = qd[1];
    rbdl_qd[5] = qd[2];
    rbdl_qd[0] = qd[3];
    rbdl_qd[1] = qd[4];
    rbdl_qd[2] = qd[5];
    rbdl_qdd[3] = qdd[0];
    rbdl_qdd[4] = qdd[1];
    rbdl_qdd[5] = qdd[2];
    rbdl_qdd[0] = qdd[3];
    rbdl_qdd[1] = qdd[4];
    rbdl_qdd[2] = qdd[5];
  }

  bool equal = true;
  if (!is_equal<Algebra>(tds.q(), rbdl_q)) {
    fprintf(stderr, "Mismatch in q.\n");
    Algebra::print("TDS:  ", tds.q());
    std::cout << "RBDL: " << rbdl_q.transpose() << std::endl;
    std::cout << "RBDL raw: " << q.transpose() << std::endl;
    equal = false;
  }
  if (!is_equal<Algebra>(tds.qd(), rbdl_qd)) {
    fprintf(stderr, "Mismatch in qd.\n");
    Algebra::print("TDS:  ", tds.qd());
    std::cout << "RBDL: " << rbdl_qd.transpose() << std::endl;
    std::cout << "RBDL raw: " << qd.transpose() << std::endl;
    equal = false;
  }
  if (!is_equal<Algebra>(tds.qdd(), rbdl_qdd)) {
    fprintf(stderr, "Mismatch in qdd.\n");
    Algebra::print("TDS:  ", tds.qdd());
    std::cout << "RBDL: " << rbdl_qdd.transpose() << std::endl;
    std::cout << "RBDL raw: " << qdd.transpose() << std::endl;
    equal = false;
  }
  return equal;
}

template <typename Algebra>
void assign(const MultiBody<Algebra> &mb,
            RigidBodyDynamics::Math::VectorNd *rbdl_q = nullptr,
            RigidBodyDynamics::Math::VectorNd *rbdl_qd = nullptr,
            RigidBodyDynamics::Math::VectorNd *rbdl_qdd = nullptr,
            RigidBodyDynamics::Math::VectorNd *rbdl_tau = nullptr) {
  int q_offset = 0, qd_offset = 0;
  if (mb.is_floating()) {
    if (rbdl_q) {
      (*rbdl_q)[0] = Algebra::to_double(mb.q(4));
      (*rbdl_q)[1] = Algebra::to_double(mb.q(5));
      (*rbdl_q)[2] = Algebra::to_double(mb.q(6));
      // w coordinate of quat is stored at the end
      (*rbdl_q)[mb.dof() - 1] = Algebra::to_double(mb.q(3));
      (*rbdl_q)[3] = Algebra::to_double(mb.q(0));
      (*rbdl_q)[4] = Algebra::to_double(mb.q(1));
      (*rbdl_q)[5] = Algebra::to_double(mb.q(2));
    }
    q_offset = 7;
    if (rbdl_qd) {
      (*rbdl_qd)[0] = Algebra::to_double(mb.qd(3));
      (*rbdl_qd)[1] = Algebra::to_double(mb.qd(4));
      (*rbdl_qd)[2] = Algebra::to_double(mb.qd(5));
      (*rbdl_qd)[3] = Algebra::to_double(mb.qd(0));
      (*rbdl_qd)[4] = Algebra::to_double(mb.qd(1));
      (*rbdl_qd)[5] = Algebra::to_double(mb.qd(2));
    }
    qd_offset = 6;
  }

  if (rbdl_q) {
    for (int i = q_offset; i < mb.dof(); ++i) {
      (*rbdl_q)[i - int(mb.is_floating())] = Algebra::to_double(mb.q(i));
    }
  }
  if (rbdl_qd) {
    for (int i = qd_offset; i < mb.dof_qd(); ++i) {
      (*rbdl_qd)[i] = Algebra::to_double(mb.qd(i));
    }
  }
  if (rbdl_qdd) {
    for (int i = qd_offset; i < mb.dof_qd(); ++i) {
      (*rbdl_qdd)[i] = Algebra::to_double(mb.qdd(i));
    }
  }
  if (rbdl_tau) {
    for (int i = 0; i < mb.dof_actuated(); ++i) {
      (*rbdl_tau)[i + qd_offset] = Algebra::to_double(mb.tau(i));
    }
  }
}
#endif
