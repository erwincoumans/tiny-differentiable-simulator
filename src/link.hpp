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
  JOINT_SPHERICAL,
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
  using VectorX = typename Algebra::VectorX;
  using Quaternion = typename Algebra::Quaternion;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6x3 = typename Algebra::Matrix6x3;

  JointType joint_type{JOINT_REVOLUTE_Z};

  Transform X_T;               // parent_link_to_joint
  mutable Transform X_J;       // joint_to_child_link    //depends on q
  mutable Transform X_parent;  // parent_link_to_child_link

  mutable Transform X_world;  // world_to_link
  mutable MotionVector vJ;    // local joint velocity (relative to parent link)
  mutable MotionVector cJ;    // bias velocity for link
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

  /// Multi DoF variables
  Matrix6x3 S_3d;          // 3 DoF joint motion subspace (spatial joint axis/matrix)
  mutable Matrix6x3 U_3d;  // temp var in ABA, page 130, for 3 DoF joints
  mutable Matrix3 D_3d;       // temp var in ABA, page 130, for 3 DoF joints
  mutable Matrix3 invD_3d;    // Create a container for the inverse of D to avoid recalculating the inverse
  mutable Vector3 u_3d;       // temp var in ABA, page 130, for 3 DoF joints

  ForceVector f_ext;  // user-defined external force in world frame

  // These two variables are managed by MultiBody and should not be changed.
  int parent_index{-1};  // index of parent link in MultiBody
  int index{-1};         // index of this link in MultiBody

  std::vector<const Geometry<Algebra> *> collision_geometries;
  std::vector<Transform> X_collisions;  // offset of collision geometries
  // (relative to this link frame)
  std::vector<int> visual_instance_uids;
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

  template <typename AlgebraTo = Algebra>
  Link<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    Link<AlgebraTo> conv(joint_type, tds::clone<Algebra, AlgebraTo>(X_T),
                         rbi.template clone<AlgebraTo>());
    conv.index = index;
    conv.parent_index = parent_index;
    conv.link_name = link_name;
    conv.joint_name = joint_name;
    conv.f_ext = tds::clone<Algebra, AlgebraTo>(f_ext);
    conv.visual_instance_uids = visual_instance_uids;
    for (const auto &x : X_visuals) {
      conv.X_visuals.push_back(x.template clone<AlgebraTo>());
    }
    for (const auto &x : X_collisions) {
      conv.X_collisions.push_back(x.template clone<AlgebraTo>());
    }
    for (auto *geom : collision_geometries) {
      conv.collision_geometries.push_back(tds::clone<Algebra, AlgebraTo>(geom));
    }
    conv.q_index = q_index;
    conv.qd_index = qd_index;
    conv.stiffness = C::convert(stiffness);
    conv.damping = C::convert(damping);
    return conv;
  }

  void set_joint_type(JointType type,
                      const Vector3 &axis = Algebra::unit3_x()) {
    joint_type = type;
    if (joint_type != JOINT_SPHERICAL) {
        S.set_zero();
    }
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
      case JOINT_SPHERICAL:
          Algebra::set_zero(S_3d);
//          Matrix3 S_top = Algebra::zero3();
//          Algebra::assign_row(S_3d, 0, Vector3(0., 0., 1.));
//        Algebra::assign_row(S_3d, 1, Vector3(0., 1., 0. ));
//        Algebra::assign_row(S_3d, 2, Vector3(1., 0., 0.));
          Algebra::assign_block(S_3d, Algebra::eye3(), 0, 0);
          break;
      case JOINT_FIXED:
        break;
      default:
        fprintf(stderr,
                "Error: unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
    if (joint_type != JOINT_FIXED && joint_type != JOINT_SPHERICAL)
    {
        if (Algebra::norm(S) == Algebra::zero()) {
            fprintf(stderr,
                "Error: subspace matrix S is zero after setting joint type on "
                "link.\n");
            assert(0);
            exit(1);
        }
    }
  }

  // Updated jcalc functions for multi DoF joints
  inline void jcalc(const Quaternion &q, Transform *X_J, Transform *X_parent) const {
      X_J->set_identity();
      X_parent->set_identity();

      switch (joint_type) {
          case JOINT_SPHERICAL:
              X_J->rotation = Algebra::quat_to_matrix(q);
              break;
          default:
              fprintf(stderr,
                      "Error: unknown joint type encountered in " __FILE__ ":%i\n",
                      __LINE__);
      }

#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
      *X_parent = (*X_J) * X_T;
#else
      *X_parent = X_T * (*X_J);
#endif
  }

  inline void jcalc(const Vector3 &qd, MotionVector *v_J) const {
        switch (joint_type) {
            case JOINT_SPHERICAL:
                v_J->top = qd;
                break;
            default:
                fprintf(stderr,
                        "Error: unknown joint type encountered in " __FILE__ ":%i\n",
                        __LINE__);
        }
    }

  void jcalc(const VectorX &q, Transform *X_J, Transform *X_parent) const {
        X_J->set_identity();
        X_parent->set_identity();
        switch (joint_type) {
            case JOINT_PRISMATIC_X:
                X_J->translation[0] = q[0];
                break;
            case JOINT_PRISMATIC_Y:
                X_J->translation[1] = q[0];
                break;
            case JOINT_PRISMATIC_Z:
                X_J->translation[2] = q[0];
                break;
            case JOINT_PRISMATIC_AXIS: {
                const Vector3 &axis = S.bottom;
                X_J->translation = axis * q[0];
                break;
            }
            case JOINT_REVOLUTE_X:
                X_J->rotation = Algebra::rotation_x_matrix(q[0]);
                break;
            case JOINT_REVOLUTE_Y:
                X_J->rotation = Algebra::rotation_y_matrix(q[0]);
                break;
            case JOINT_REVOLUTE_Z:
                X_J->rotation = Algebra::rotation_z_matrix(q[0]);
                break;
            case JOINT_REVOLUTE_AXIS: {
                const Vector3 &axis = S.top;
                const auto quat = Algebra::axis_angle_quaternion(axis, q[0]);
                X_J->rotation = Algebra::quat_to_matrix(quat);
                break;
            }
            case JOINT_SPHERICAL: {
                const auto quat = Algebra::quat_from_xyzw(q[0], q[1], q[2], q[3]);
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
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
        *X_parent = (*X_J) * X_T;
#else


        //X_T.print("X_T");
        //X_J->print("X_J");
        *X_parent = X_T * (*X_J);
        //X_parent->print("X_parent");

#endif
    }

  inline void jcalc(const VectorX &qd, MotionVector *v_J) const {
        switch (joint_type) {
            case JOINT_PRISMATIC_X:
                v_J->bottom[0] = qd[0];
                break;
            case JOINT_PRISMATIC_Y:
                v_J->bottom[1] = qd[0];
                break;
            case JOINT_PRISMATIC_Z:
                v_J->bottom[2] = qd[0];
                break;
            case JOINT_PRISMATIC_AXIS: {
                const Vector3 &axis = S.bottom;
                v_J->bottom = axis * qd[0];
                break;
            }
            case JOINT_REVOLUTE_X:
                v_J->top[0] = qd[0];
                break;
            case JOINT_REVOLUTE_Y:
                v_J->top[1] = qd[0];
                break;
            case JOINT_REVOLUTE_Z:
                v_J->top[2] = qd[0];
                break;
            case JOINT_REVOLUTE_AXIS: {
                const Vector3 &axis = S.top;
                v_J->top = axis * qd[0];
                break;
            }
            case JOINT_SPHERICAL:
                v_J->top = Vector3(qd[0], qd[1], qd[2]);
                break;
            case JOINT_FIXED:
                break;
            default:
                fprintf(stderr,
                        "Error: Unknown joint type encountered in " __FILE__ ":%i\n",
                        __LINE__);
        }
    }
    inline void jcalc(const VectorX &q) const { 
        jcalc(q, &X_J, &X_parent); 
    }
    inline void jcalc(const VectorX &q, const VectorX &qd) const {
        jcalc(q);
        jcalc(qd, &vJ);
    }

  // Old jcalc functions
  inline void jcalc1(Scalar q) { jcalc(q, &X_J, &X_parent); }

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
        //*X_J = X_J_fixed;
        break;
      default:
        fprintf(stderr,
                "Error: Unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
#ifdef TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS
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
        //*v_J = v_J_fixed;
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

template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static inline Link<AlgebraTo> clone(const Link<AlgebraFrom> &link) {
  return link.template clone<AlgebraTo>();
}
}  // namespace tds
