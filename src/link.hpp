#pragma once

#include <vector>

#include "geometry.hpp"
#include "math/transform.hpp"

namespace tds {
enum JointType {
  JOINT_FIXED = -1,
  JOINT_PRISMATIC_X = 0,
  JOINT_PRISMATIC_Y,
  JOINT_PRISMATIC_Z,
  JOINT_PRISMATIC_AXIS,
  JOINT_REVOLUTE_X,
  JOINT_REVOLUTE_Y,
  JOINT_REVOLUTE_Z,
  JOINT_REVOLUTE_AXIS,
  JOINT_INVALID,
};

template <typename Algebra>
struct Link {
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::RigidBodyInertia<Algebra> RigidBodyInertia;
  typedef tds::ArticulatedBodyInertia<Algebra> ArticulatedBodyInertia;
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;

  JointType joint_type{JOINT_REVOLUTE_Z};

  Transform X_T;               // parent_link_to_joint
  mutable Transform X_J;       // joint_to_child_link    //depends on q
  mutable Transform X_parent;  // parent_link_to_child_link

  mutable Transform X_world;  // world_to_link
  mutable MotionVector vJ;    // local joint velocity (relative to parent link)
  mutable MotionVector v;     // global joint velocity (relative to world)
  mutable MotionVector a;     // acceleration (relative to world)
  mutable MotionVector c;     // velocity product acceleration

  RigidBodyInertia rbi;  // local rigid-body spatial inertia (constant)
  mutable ArticulatedBodyInertia abi;  // spatial articulated inertia

  mutable ForceVector pA;  // bias forces or zero-acceleration forces
  MotionVector S;          // motion subspace (spatial joint axis/matrix)

  mutable ForceVector U;  // temp var in ABA, page 130
  mutable Scalar D;       // temp var in ABA, page 130
  mutable Scalar u;       // temp var in ABA, page 130
  mutable ForceVector f;  // temp var in RNEA, page 183

  ForceVector f_ext;  // user-defined external force in world frame

  // These two variables are managed by MultiBody and should not be changed.
  int parent_index{-1};  // index of parent link in MultiBody
  int index{-1};         // index of this link in MultiBody

  std::vector<const Geometry<Algebra> *> collision_geometries;
  std::vector<Transform> X_collisions;  // offset of collision geometries
  // (relative to this link frame)
  std::vector<int> visual_ids;
  std::vector<Transform>
      X_visuals;  // offset of geometry (relative to this link frame)

  std::string link_name;
  std::string joint_name;

  // index in MultiBody q / qd arrays
  int q_index{-2};
  int qd_index{-2};

  Scalar stiffness{Algebra::zero()};
  Scalar damping{Algebra::zero()};

  Link() = default;
  Link(JointType joint_type, const Transform &parent_link_to_joint,
       const RigidBodyInertia &rbi)
      : X_T(parent_link_to_joint), rbi(rbi) {
    set_joint_type(joint_type);
  }

  void set_joint_type(JointType type,
                      const Vector3 &axis = Algebra::unit3_x()) {
    joint_type = type;
    S.set_zero();
    switch (joint_type) {
      case JOINT_PRISMATIC_X:
        S.bottom[0] = Algebra::one();
        break;
      case JOINT_PRISMATIC_Y:
        S.bottom[1] = Algebra::one();
        break;
      case JOINT_PRISMATIC_Z:
        S.bottom[2] = Algebra::one();
        break;
      case JOINT_PRISMATIC_AXIS:
        if (Algebra::norm(axis) == Algebra::zero()) {
          fprintf(stderr,
                  "Error: tried to set zero vector as prismatic joint axis.\n");
          assert(0);
          exit(1);
        }
        S.bottom = axis;
        break;
      case JOINT_REVOLUTE_X:
        S.top[0] = Algebra::one();
        break;
      case JOINT_REVOLUTE_Y:
        S.top[1] = Algebra::one();
        break;
      case JOINT_REVOLUTE_Z:
        S.top[2] = Algebra::one();
        break;
      case JOINT_REVOLUTE_AXIS:
        if (Algebra::norm(axis) == Algebra::zero()) {
          fprintf(stderr,
                  "Error: tried to set zero vector as revolute joint axis.\n");
          assert(0);
          exit(1);
        }
        S.top = axis;
        break;
      case JOINT_FIXED:
        break;
      default:
        fprintf(stderr,
                "Error: unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
    if (Algebra::norm(S) == Algebra::zero()) {
      fprintf(stderr,
              "Error: subspace matrix S is zero after setting joint type on "
              "link.\n");
      assert(0);
      exit(1);
    }
  }

  void jcalc(const Scalar &q, Transform *X_J, Transform *X_parent) const {
    X_J->set_identity();
    X_parent->set_identity();
    switch (joint_type) {
      case JOINT_PRISMATIC_X:
        X_J->translation[0] = q;
        break;
      case JOINT_PRISMATIC_Y:
        X_J->translation[1] = q;
        break;
      case JOINT_PRISMATIC_Z:
        X_J->translation[2] = q;
        break;
      case JOINT_PRISMATIC_AXIS: {
        const Vector3 &axis = S.bottom;
        X_J->translation = axis * q;
        break;
      }
      case JOINT_REVOLUTE_X:
        X_J->rotation = Algebra::rotation_x_matrix(q);
        break;
      case JOINT_REVOLUTE_Y:
        X_J->rotation = Algebra::rotation_y_matrix(q);
        break;
      case JOINT_REVOLUTE_Z:
        X_J->rotation = Algebra::rotation_z_matrix(q);
        break;
      case JOINT_REVOLUTE_AXIS: {
        const Vector3 &axis = S.top;
        const auto quat = Algebra::axis_angle_quaternion(axis, q);
        X_J->rotation = Algebra::quat_to_matrix(quat);
        break;
      }
      case JOINT_FIXED:
        // Transform is set to identity in its constructor already
        // and never changes.
        break;
      default:
        fprintf(stderr,
                "Error: Unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
#if SWAP_TRANSFORM_ASSOCIATIVITY
    *X_parent = (*X_J) * X_T;
#else
    *X_parent = X_T * (*X_J);
#endif
  }

  inline void jcalc(const Scalar &qd, MotionVector *v_J) const {
    switch (joint_type) {
      case JOINT_PRISMATIC_X:
        v_J->bottom[0] = qd;
        break;
      case JOINT_PRISMATIC_Y:
        v_J->bottom[1] = qd;
        break;
      case JOINT_PRISMATIC_Z:
        v_J->bottom[2] = qd;
        break;
      case JOINT_PRISMATIC_AXIS: {
        const Vector3 &axis = S.bottom;
        v_J->bottom = axis * qd;
        break;
      }
      case JOINT_REVOLUTE_X:
        v_J->top[0] = qd;
        break;
      case JOINT_REVOLUTE_Y:
        v_J->top[1] = qd;
        break;
      case JOINT_REVOLUTE_Z:
        v_J->top[2] = qd;
        break;
      case JOINT_REVOLUTE_AXIS: {
        const Vector3 &axis = S.top;
        v_J->top = axis * qd;
        break;
      }
      case JOINT_FIXED:
        break;
      default:
        fprintf(stderr,
                "Error: Unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
  }

  inline void jcalc(const Scalar &q) const { jcalc(q, &X_J, &X_parent); }

  inline void jcalc(const Scalar &q, const Scalar &qd) const {
    jcalc(q);
    jcalc(qd, &vJ);
  }
};
}  // namespace tds
