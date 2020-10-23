#include <cassert>
#include <chrono>
#include <cstdio>
#include <thread>

// #define DEBUG 1

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
#endif //USE_BULLET_URDF_PARSER
#include "utils/file_utils.hpp"
#include "world.hpp"

#if USE_RBDL
#include "rbdl/Dynamics.h"
#include "rbdl/Model.h"
#include "rbdl/rbdl.h"
#endif

using namespace tds;

const double ERROR_TOLERANCE = 1e-2;


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
RigidBodyDynamics::Math::SpatialTransform to_rbdl(
    const Transform<Algebra> &tf) {
  return RigidBodyDynamics::Math::SpatialTransform(
      to_rbdl<Algebra>(tf.rotation), to_rbdl<Algebra>(tf.translation));
}

template <typename Algebra>
RigidBodyDynamics::Model to_rbdl(const MultiBody<Algebra> &mb) {
  RigidBodyDynamics::Model model;
  unsigned int parent_id_offset = 0;
  if (mb.is_floating()) {
    RigidBodyDynamics::Body body(Algebra::to_double(mb.base_rbi().mass),
                                 to_rbdl<Algebra>(mb.base_rbi().com),
                                 to_rbdl<Algebra>(mb.base_rbi().inertia));
    RigidBodyDynamics::Joint joint;
    joint = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFloatingBase);
    parent_id_offset = model.AddBody(
        0, to_rbdl<Algebra>(tds::Transform<Algebra>()), joint, body);
    std::cout << "RBDL floating-base ID: " << parent_id_offset << std::endl;
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
    unsigned int parent_id = link.parent_index < 0 ? 0u : link.parent_index + 1;
    parent_id += parent_id_offset;
    model.AddBody(parent_id, to_rbdl<Algebra>(link.X_T), joint, body);
  }
  return model;
}

template <typename Algebra>
bool is_equal(const SpatialVector<Algebra> &a,
              const RigidBodyDynamics::Math::SpatialVector &b) {
  for (int i = 0; i < 6; ++i) {
    if (std::abs(Algebra::to_double(a[i]) - b[i]) > ERROR_TOLERANCE) {
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
  if (std::abs(Algebra::to_double(a) - b) > ERROR_TOLERANCE) {
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
    if (std::abs(Algebra::to_double(a[i]) - b[i]) > ERROR_TOLERANCE) {
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
    if (std::abs(Algebra::to_double(a[i]) - b[i]) > ERROR_TOLERANCE) {
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
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) > ERROR_TOLERANCE) {
#else
      if (std::abs(Algebra::to_double(a(i, j)) - b(j, i)) > ERROR_TOLERANCE) {
#endif
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
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) > ERROR_TOLERANCE) {
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
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) > ERROR_TOLERANCE) {
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
  return is_equal<Algebra>(a.translation, b.r) &&
         is_equal<Algebra>(a.rotation, b.E);
}

template <typename Algebra>
bool is_equal(const ArticulatedBodyInertia<Algebra> &a,
              const RigidBodyDynamics::Math::SpatialMatrix &b) {
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (std::abs(Algebra::to_double(a(i, j)) - b(i, j)) > ERROR_TOLERANCE) {
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
              const RigidBodyDynamics::Model &rbdl) {
  // floating-base models in RBDL have a TranslationXYZ and a spherical joint
  // first
  std::size_t id_offset = tds.is_floating() ? 3 : 1;
  for (std::size_t j = 0; j < tds.size(); ++j) {
    std::size_t rbdl_j = j + id_offset;
    if (!is_equal<Algebra>(tds[j].X_world, rbdl.X_base[rbdl_j])) {
      fprintf(stderr, "Mismatch in X_base (X_world) at link %i.\n",
              static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].X_world);
      std::cout << "RBDL:\n" << rbdl.X_base[rbdl_j] << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].X_parent, rbdl.X_lambda[rbdl_j])) {
      fprintf(stderr, "Mismatch in X_lambda (X_parent) at link %i.\n",
              static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].X_parent);
      std::cout << "RBDL:\n" << rbdl.X_lambda[rbdl_j] << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].abi, rbdl.IA[rbdl_j])) {
      fprintf(stderr, "Mismatch in ABI at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].abi);
      std::cout << "RBDL:\n" << rbdl.IA[rbdl_j] << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].S, rbdl.S[rbdl_j])) {
      fprintf(stderr, "Mismatch in S at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].S);
      std::cout << "RBDL: " << rbdl.S[rbdl_j].transpose() << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].v, rbdl.v[rbdl_j])) {
      fprintf(stderr, "Mismatch in v at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].v);
      std::cout << "RBDL: " << rbdl.v[rbdl_j].transpose() << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].c, rbdl.c[rbdl_j])) {
      fprintf(stderr, "Mismatch in c at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].c);
      std::cout << "RBDL: " << rbdl.c[rbdl_j].transpose() << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].U, rbdl.U[rbdl_j])) {
      fprintf(stderr, "Mismatch in U at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].U);
      std::cout << "RBDL: " << rbdl.U[rbdl_j].transpose() << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].pA, rbdl.pA[rbdl_j])) {
      fprintf(stderr, "Mismatch in pA at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].pA);
      std::cout << "RBDL: " << rbdl.pA[rbdl_j].transpose() << std::endl;
      return false;
    }
    if (!is_equal<Algebra>(tds[j].u, rbdl.u[rbdl_j])) {
      fprintf(stderr, "Mismatch in u at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].u);
      std::cout << "RBDL: " << rbdl.u[rbdl_j] << std::endl;
      // return false;
    }
    if (!is_equal<Algebra>(tds[j].D, rbdl.d[rbdl_j])) {
      fprintf(stderr, "Mismatch in D at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].D);
      std::cout << "RBDL: " << rbdl.d[rbdl_j] << std::endl;
      // return false;
    }
    if (!is_equal<Algebra>(tds[j].a, rbdl.a[rbdl_j])) {
      fprintf(stderr, "Mismatch in a at link %i.\n", static_cast<int>(j));
      Algebra::print("TDS:  ", tds[j].a);
      std::cout << "RBDL: " << rbdl.a[rbdl_j].transpose() << std::endl;
      // return false;
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

  if (!is_equal<Algebra>(tds.q(), rbdl_q)) {
    fprintf(stderr, "Mismatch in q.\n");
    Algebra::print("TDS:  ", tds.q());
    std::cout << "RBDL: " << rbdl_q.transpose() << std::endl;
    return false;
  }
  if (!is_equal<Algebra>(tds.qd(), rbdl_qd)) {
    fprintf(stderr, "Mismatch in qd.\n");
    Algebra::print("TDS:  ", tds.qd());
    std::cout << "RBDL: " << rbdl_qd.transpose() << std::endl;
    return false;
  }
  if (!is_equal<Algebra>(tds.qdd(), rbdl_qdd)) {
    fprintf(stderr, "Mismatch in qdd.\n");
    Algebra::print("TDS:  ", tds.qdd());
    std::cout << "RBDL: " << rbdl_qdd.transpose() << std::endl;
    return false;
  }
  return true;
}
#endif

