/*
 * Copyright 2020 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TINY_MULTIBODY_H
#define TINY_MULTIBODY_H

/// TinyMultiBody follows the notation from the Featherstone Book,
/// Rigid Body Dynamics Algorithms, 2008. This Forward Dynamics implementation
/// of Articulated Body Algorithm is strongly influenced by the RBDL library.
/// By Martin L. Felis. https://rbdl.bitbucket.io

#include <string>
#include <vector>

#include "tiny_actuator.h"
#include "tiny_geometry.h"
#include "tiny_matrix_x.h"
#include "tiny_spatial_motion_vector.h"
#include "tiny_spatial_transform.h"
#include "tiny_symmetric_spatial_dyad.h"

enum TinyJointType {
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

enum TinyIntegrationType { INT_EULER, INT_EULER_SYMPLECTIC };

template <typename TinyScalar, typename TinyConstants>
class TinyLink {
  typedef ::TinySpatialTransform<TinyScalar, TinyConstants>
      TinySpatialTransform;
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef ::TinySpatialMotionVector<TinyScalar, TinyConstants>
      TinySpatialMotionVector;
  typedef ::TinySymmetricSpatialDyad<TinyScalar, TinyConstants>
      TinySymmetricSpatialDyad;

 public:
  TinyLink() = default;
  TinyLink(TinyJointType joint_type, TinySpatialTransform& parent_link_to_joint,
           const TinySymmetricSpatialDyad& inertia)
      : m_joint_type(joint_type), m_X_T(parent_link_to_joint), m_I(inertia) {}

  TinySpatialTransform m_X_T;        // parent_link_to_joint
  TinySpatialTransform m_X_J;        // joint_to_child_link    //depends on q
  TinySpatialTransform m_X_parent2;  // parent_link_to_child_link

  TinyJointType m_joint_type{JOINT_REVOLUTE_Z};

  TinySpatialTransform m_X_world;  // world_to_link
  TinySpatialMotionVector
      m_vJ;  // local joint velocity (relative to parent link)
  TinySpatialMotionVector m_v;  // global joint velocity (relative to world)
  TinySpatialMotionVector m_a;  // acceleration (relative to world)
  TinySpatialMotionVector m_c;  // velocity product acceleration

  TinySymmetricSpatialDyad
      m_I;  // local spatial inertia (constant) // TODO replace by its original
            // terms (COM, gyration etc.)
  TinySymmetricSpatialDyad m_IA;  // spatial articulated inertia, IC in CRBA

  TinySpatialMotionVector m_pA;  // bias forces or zero-acceleration forces
  TinySpatialMotionVector m_S;   // motion subspace (spatial joint axis/matrix)

  TinySpatialMotionVector m_U;  // temp var in ABA, page 130
  TinyScalar m_d;               // temp var in ABA, page 130
  TinyScalar m_u;               // temp var in ABA, page 130
  TinySpatialMotionVector m_f;  // temp var in RNEA, page 183

  TinySpatialMotionVector
      m_f_ext;  // user-defined external force in world frame

  // These two variables are managed by TinyMultiBody and should not be changed.
  int m_parent_index{-1};  // index of parent link in TinyMultiBody
  int m_index{-1};         // index of this link in TinyMultiBody

  std::vector<const TinyGeometry<TinyScalar, TinyConstants>*>
      m_collision_geometries;
  std::vector<TinySpatialTransform>
      m_X_collisions;  // offset of collision geometries (relative to this link
                       // frame)
  std::vector<int> m_visual_uids1;
  std::vector<int> m_visual_uids2;
  std::vector<TinySpatialTransform>
      m_X_visuals;  // offset of geometry (relative to this link frame)
  std::string m_link_name;
  std::string m_joint_name;
  // index in MultiBody q / qd arrays
  int m_q_index{-2};
  int m_qd_index{-2};

  TinyScalar m_stiffness{TinyConstants::zero()};
  TinyScalar m_damping{TinyConstants::zero()};

  void set_joint_type(TinyJointType type,
                      const TinyVector3& axis = TinyVector3::makeUnitX()) {
    m_joint_type = type;
    m_S.set_zero();
    switch (m_joint_type) {
      case JOINT_PRISMATIC_X:
        m_S.m_bottomVec.setX(TinyConstants::one());
        break;
      case JOINT_PRISMATIC_Y:
        m_S.m_bottomVec.setY(TinyConstants::one());
        break;
      case JOINT_PRISMATIC_Z:
        m_S.m_bottomVec.setZ(TinyConstants::one());
        break;
      case JOINT_PRISMATIC_AXIS:
        m_S.m_bottomVec = axis;
        break;
      case JOINT_REVOLUTE_X:
        m_S.m_topVec.setX(TinyConstants::one());
        break;
      case JOINT_REVOLUTE_Y:
        m_S.m_topVec.setY(TinyConstants::one());
        break;
      case JOINT_REVOLUTE_Z:
        m_S.m_topVec.setZ(TinyConstants::one());
        break;
      case JOINT_REVOLUTE_AXIS:
        m_S.m_topVec = axis;
        break;
      case JOINT_FIXED:
        break;
      default:
        fprintf(stderr,
                "Error: Unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
  }

  void jcalc(TinyScalar q, TinySpatialTransform* X_J,
             TinySpatialTransform* X_parent) const {
    X_J->set_identity();
    X_parent->set_identity();
    switch (m_joint_type) {
      case JOINT_PRISMATIC_X:
        X_J->m_translation.setX(q);
        break;
      case JOINT_PRISMATIC_Y:
        X_J->m_translation.setY(q);
        break;
      case JOINT_PRISMATIC_Z:
        X_J->m_translation.setZ(q);
        break;
      case JOINT_PRISMATIC_AXIS: {
        const TinyVector3& axis = m_S.m_bottomVec;
        X_J->m_translation = axis * q;
        break;
      }
      case JOINT_REVOLUTE_X:
        X_J->m_rotation.set_rotation_x(q);
        break;
      case JOINT_REVOLUTE_Y:
        X_J->m_rotation.set_rotation_y(q);
        break;
      case JOINT_REVOLUTE_Z:
        X_J->m_rotation.set_rotation_z(q);
        break;
      case JOINT_REVOLUTE_AXIS: {
        const TinyVector3& axis = m_S.m_topVec;
        TinyQuaternion<TinyScalar, TinyConstants> orn;
        orn.setRotation(axis, q);
        X_J->m_rotation.setRotation(orn);
        break;
      }
      case JOINT_FIXED:
        // TinySpatialTransform is set to identity in its constructor already
        // and never changes.
        break;
      default:
        fprintf(stderr,
                "Error: Unknown joint type encountered in " __FILE__ ":%i\n",
                __LINE__);
    }
    *X_parent = m_X_T * (*X_J);
  }

  inline void jcalc(TinyScalar qd, TinySpatialMotionVector* v_J) const {
    switch (m_joint_type) {
      case JOINT_PRISMATIC_X:
        v_J->m_bottomVec.setX(qd);
        break;
      case JOINT_PRISMATIC_Y:
        v_J->m_bottomVec.setY(qd);
        break;
      case JOINT_PRISMATIC_Z:
        v_J->m_bottomVec.setZ(qd);
        break;
      case JOINT_PRISMATIC_AXIS: {
        const TinyVector3& axis = m_S.m_bottomVec;
        v_J->m_bottomVec = axis * qd;
        break;
      }
      case JOINT_REVOLUTE_X:
        v_J->m_topVec.setX(qd);
        break;
      case JOINT_REVOLUTE_Y:
        v_J->m_topVec.setY(qd);
        break;
      case JOINT_REVOLUTE_Z:
        v_J->m_topVec.setZ(qd);
        break;
      case JOINT_REVOLUTE_AXIS: {
        const TinyVector3& axis = m_S.m_topVec;
        v_J->m_topVec = axis * qd;
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

  inline void jcalc(TinyScalar q) { jcalc(q, &m_X_J, &m_X_parent2); }
  inline void jcalc1(TinyScalar q) { jcalc(q, &m_X_J, &m_X_parent2); }

  inline void jcalc(TinyScalar q, TinyScalar qd) {
    jcalc(q);
    jcalc(qd, &m_vJ);
  }
};

template <typename TinyScalar, typename TinyConstants>
class TinyMultiBody {
  typedef ::TinyLink<TinyScalar, TinyConstants> TinyLink;
  typedef ::TinySpatialMotionVector<TinyScalar, TinyConstants>
      TinySpatialMotionVector;
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyVectorX<TinyScalar, TinyConstants> TinyVectorX;
  typedef ::TinyQuaternion<TinyScalar, TinyConstants> TinyQuaternion;
  typedef ::TinySymmetricSpatialDyad<TinyScalar, TinyConstants>
      TinySymmetricSpatialDyad;
  typedef ::TinySpatialTransform<TinyScalar, TinyConstants>
      TinySpatialTransform;
  typedef ::TinyMatrix3x3<TinyScalar, TinyConstants> TinyMatrix3x3;
  typedef ::TinyMatrix3xX<TinyScalar, TinyConstants> TinyMatrix3xX;
  typedef ::TinyMatrix6xX<TinyScalar, TinyConstants> TinyMatrix6xX;
  typedef ::TinyMatrixXxX<TinyScalar, TinyConstants> TinyMatrixXxX;
  typedef ::TinyActuator<TinyScalar, TinyConstants> TinyActuator;

 public:
  std::vector<TinyLink> m_links;

  TinyIntegrationType m_integration_type{INT_EULER_SYMPLECTIC};

  /**
   * Number of degrees of freedom, excluding floating-base coordinates.
   */
  int m_dof{0};

  /**
   * Optionally defined actuator for this link that computes the dynamics of
   * the control input and computes the joint torque.
   * Memory is managed by TinyLink.
   */
  TinyActuator* m_actuator{nullptr};

  /**
   * Dimensionality of joint positions q (including 7-DoF floating-base
   * coordinates if this system is floating-base).
   */
  int dof() const { return m_isFloating ? m_dof + 7 : m_dof; }
  /**
   * Dimensionality of joint velocities qd and accelerations qdd (including
   * 6-DoF base velocity and acceleration, if this system is floating-base).
   */
  int dof_qd() const { return m_isFloating ? m_dof + 6 : m_dof; }

  /**
   * Indices in `m_tau` that are controllable, i.e. actuated.
   * For floating-base system, the index 0 corresponds to the first degree of
   * freedom not part of the 6D floating-base coordinates.
   */
  std::vector<int> m_control_indices;
  /**
   * Dimensionality of control input, i.e. number of actuated DOFs.
   */
  int dof_actuated() const {
    return static_cast<int>(m_control_indices.size());
  }

  /**
   * Whether this system is floating or fixed to the world frame.
   */
  bool m_isFloating{false};

  // quantities related to floating base
  TinySpatialMotionVector m_baseVelocity;             // v_0
  TinySpatialMotionVector m_baseAcceleration;         // a_0
  TinySpatialMotionVector m_baseAppliedForce;         // f_ext_0 in world frame
  TinySpatialMotionVector m_baseForce;                // f_0 (used by RNEA)
  TinySpatialMotionVector m_baseBiasForce;            // pA_0
  TinySymmetricSpatialDyad m_baseInertia;             // I_0
  TinySymmetricSpatialDyad m_baseArticulatedInertia;  // IA_0
  TinySpatialTransform m_base_X_world;
  std::vector<int> m_visual_uids1;
  std::vector<int> m_visual_uids2;
  std::vector<TinySpatialTransform>
      m_X_visuals;  // offset of geometry (relative to the base frame)

  std::vector<const TinyGeometry<TinyScalar, TinyConstants>*>
      m_collision_geometries;
  std::vector<TinySpatialTransform>
      m_X_collisions;  // offset of collision geometries (relative to this link
                       // frame)

  std::vector<TinyScalar> m_q, m_qd, m_qdd, m_tau;

  TinySubmitProfileTiming m_profileTimingFunc;

  explicit TinyMultiBody(bool isFloating = false)
      : m_isFloating(isFloating), m_profileTimingFunc(nullptr) {}

  inline void submitProfileTiming(const std::string& name) {
    if (m_profileTimingFunc) {
      m_profileTimingFunc(name);
    }
  }

  /**
   * Set 3D base position in world coordinates.
   */
  void set_position(const TinyVector3& initial_position) {
    m_base_X_world.m_translation.setValue(
        initial_position[0], initial_position[1], initial_position[2]);
    if (m_isFloating) {
      m_q[4] = initial_position[0];
      m_q[5] = initial_position[1];
      m_q[6] = initial_position[2];
    }
  }

  /**
   * Ensures that the joint coordinates q, qd, qdd, tau are initialized
   * properly in the MultiBody member variables.
   */
  void initialize() {
    // make sure m_dof and the q / qd indices in the links are accurate
    int q_index = m_isFloating ? 7 : 0;
    int qd_index = m_isFloating ? 6 : 0;
    m_dof = 0;  // excludes floating-base DOF
    for (TinyLink& link : m_links) {
      assert(link.m_index >= 0);
      link.m_q_index = q_index;
      link.m_qd_index = qd_index;
      if (link.m_joint_type != JOINT_FIXED) {
        ++q_index;
        ++qd_index;
        ++m_dof;
      } else {
        link.m_q_index = -2;
        link.m_qd_index = -2;
      }
    }

    if (static_cast<int>(m_q.size()) != dof()) {
      m_q.resize(dof(), TinyConstants::zero());
    }
    for (TinyScalar& v : m_q) {
      v = TinyConstants::zero();
    }
    if (m_isFloating) {
      m_q[3] = TinyConstants::one();  // make sure orientation is valid
    }
    if (static_cast<int>(m_qd.size()) != dof_qd()) {
      m_qd.resize(dof_qd(), TinyConstants::zero());
    }
    for (TinyScalar& v : m_qd) {
      v = TinyConstants::zero();
    }
    if (static_cast<int>(m_qdd.size()) != dof_qd()) {
      m_qdd.resize(dof_qd(), TinyConstants::zero());
    }
    for (TinyScalar& v : m_qdd) {
      v = TinyConstants::zero();
    }
    if (static_cast<int>(m_tau.size()) != m_dof) {
      m_tau.resize(m_dof, TinyConstants::zero());
    }
    for (TinyScalar& v : m_tau) {
      v = TinyConstants::zero();
    }

    // (Re-)create actuator to make sure it has the right degrees of freedom.
    if (m_actuator) {
      delete m_actuator;
      m_actuator = new TinyActuator(dof_actuated());
    }
  }

  /**
   * Copy constructor. Skips visualization members, temporary variables.
   * The actuator is not copied, but the original pointer `m_actuator` is
   * carried over.
   */
  template <typename Scalar, typename Utils>
  TinyMultiBody(const TinyMultiBody<Scalar, Utils>& mb)
      : m_links(mb.m_links),
        m_integration_type(mb.m_integration_type),
        m_dof(mb.m_dof),
        m_actuator(mb.m_actuator),
        m_control_indices(mb.m_control_indices),
        m_isFloating(mb.m_isFloating),
        m_baseVelocity(mb.m_baseVelocity),
        m_baseAcceleration(mb.m_baseAcceleration),
        m_baseAppliedForce(mb.m_baseAppliedForce),
        m_baseForce(mb.m_baseForce),
        m_baseBiasForce(mb.m_baseBiasForce),
        m_baseInertia(mb.m_baseInertia),
        m_base_X_world(mb.m_base_X_world),
        m_collision_geometries(mb.m_collision_geometries),
        m_X_collisions(mb.m_X_collisions),
        m_q(mb.m_q),
        m_qd(mb.m_qd),
        m_qdd(mb.m_qdd),
        m_tau(mb.m_tau) {
    // TODO implement type conversion
    static_assert(std::is_same<Scalar, TinyScalar>::value,
                  "Copy constructor for TinyMultiBody of different scalar type "
                  "not yet implemented.");
  }

  virtual ~TinyMultiBody() {
    if (m_actuator) {
      delete m_actuator;
    }
  }

  void print_state() const {
    printf("q: [");
    for (int i = 0; i < dof(); ++i) {
      if (i > 0) printf(" ");
      printf("%.2f", TinyConstants::getDouble(m_q[i]));
    }
    printf("] \tqd: [");
    for (int i = 0; i < dof_qd(); ++i) {
      if (i > 0) printf(" ");
      printf("%.2f", TinyConstants::getDouble(m_qd[i]));
    }
    printf("] \tqdd: [");
    for (int i = 0; i < dof_qd(); ++i) {
      if (i > 0) printf(" ");
      printf("%.2f", TinyConstants::getDouble(m_qdd[i]));
    }
    printf("] \ttau: [");
    for (int i = 0; i < m_dof; ++i) {
      if (i > 0) printf(" ");
      printf("%.2f", TinyConstants::getDouble(m_tau[i]));
    }
    printf("]\n");
  }

  const TinySpatialTransform& get_world_transform(int link) const {
    if (link == -1) {
      return m_base_X_world;
    } else {
      return m_links[link].m_X_world;
    }
  }

  /**
   * Compute center of mass of link in world coordinates.
   * @param link Index of link in `m_links`.
   * @return 3D coordinates of center of mass in world coordinates.
   */
  const TinyVector3 get_world_com(int link) const {
    const TinySpatialTransform& tf = get_world_transform(link);
    if (link == -1) {
      return tf.apply(m_baseInertia.m_center_of_mass);
    } else {
      return tf.apply(m_links[link].m_I.m_center_of_mass);
    }
  }

  std::vector<const TinyGeometry<TinyScalar, TinyConstants>*>&
  get_collision_geometries(int i) {
    if (i == -1) {
      return m_collision_geometries;
    } else {
      return m_links[i].m_collision_geometries;
    }
  }

  std::vector<TinySpatialTransform>& get_collision_transforms(int i) {
    if (i == -1) {
      return m_X_collisions;
    } else {
      return m_links[i].m_X_collisions;
    }
  }

  static std::string joint_type_name(TinyJointType t) {
    static std::string names[] = {
        "JOINT_FIXED",       "JOINT_PRISMATIC_X",    "JOINT_PRISMATIC_Y",
        "JOINT_PRISMATIC_Z", "JOINT_PRISMATIC_AXIS", "JOINT_REVOLUTE_X",
        "JOINT_REVOLUTE_Y",  "JOINT_REVOLUTE_Z",     "JOINT_REVOLUTE_AXIS",
    };
    return names[int(t) + 1];
  }

  // attaches a new link, setting parent to the last link
  void attach(TinyLink& link, bool is_controllable = true) {
    if (m_links.empty())
      link.m_parent_index = -1;
    else
      link.m_parent_index = m_links.size() - 1;
    link.m_index = m_links.size();
    if (link.m_joint_type != JOINT_FIXED) {
      link.m_q_index = dof();
      link.m_qd_index = dof_qd();
      m_dof++;
      if (is_controllable) {
        if (m_control_indices.empty()) {
          m_control_indices.push_back(0);
        } else {
          m_control_indices.push_back(m_control_indices.back() + 1);
        }
      }
    } else {
      link.m_q_index = -2;
      link.m_qd_index = -2;
    }
#ifdef DEBUG
    printf(
        "Attached link %i of type %s (parent: %i, index q: %i, index qd: "
        "%i).\n",
        link.m_index, joint_type_name(link.m_joint_type).c_str(),
        link.m_parent_index, link.m_q_index, link.m_qd_index);
//    link.m_S.print("joint.S");
#endif
    m_links.push_back(link);
  }

  void attach_link(TinyLink& link, int parent_index,
                   bool is_controllable = true) {
    attach(link, parent_index, is_controllable);
  }
  void attach(TinyLink& link, int parent_index, bool is_controllable = true) {
    int sz = m_links.size();
    assert(parent_index < sz);
    link.m_index = sz;
    link.m_parent_index = parent_index;
    if (link.m_joint_type != JOINT_FIXED) {
      link.m_q_index = dof();
      link.m_qd_index = dof_qd();
      m_dof++;
      if (is_controllable) {
        if (m_control_indices.empty()) {
          m_control_indices.push_back(0);
        } else {
          m_control_indices.push_back(m_control_indices.back() + 1);
        }
      }
    } else {
      link.m_q_index = -2;
      link.m_qd_index = -2;
    }
#ifdef DEBUG
    printf(
        "Attached link %i of type %s (parent: %i, index q: %i, index qd: "
        "%i).\n",
        link.m_index, joint_type_name(link.m_joint_type).c_str(),
        link.m_parent_index, link.m_q_index, link.m_qd_index);
//    link.m_S.print("joint.S");
#endif
    m_links.push_back(link);
  }

  inline TinyScalar get_q_for_link(const std::vector<TinyScalar>& q,
                                   int link_index) const {
    if (q.empty()) return TinyConstants::zero();
    const TinyLink& link = m_links[link_index];
    return link.m_joint_type == JOINT_FIXED ? TinyConstants::zero()
                                            : q[link.m_q_index];
  }
  inline TinyScalar get_q_for_link(int link_index) const {
    get_q_for_link(m_q, link_index);
  }

  inline TinyScalar get_qd_for_link(const std::vector<TinyScalar>& qd,
                                    int link_index) const {
    if (qd.empty()) return TinyConstants::zero();
    const TinyLink& link = m_links[link_index];
    return link.m_joint_type == JOINT_FIXED ? TinyConstants::zero()
                                            : qd[link.m_qd_index];
  }
  inline TinyScalar get_qd_for_link(int link_index) const {
    return get_qd_for_link(m_qd, link_index);
  }

  inline TinyScalar get_qdd_for_link(const std::vector<TinyScalar>& qdd,
                                     int link_index) const {
    return get_qd_for_link(qdd, link_index);
  }
  inline TinyScalar get_qdd_for_link(int link_index) const {
    return get_qdd_for_link(m_qdd, link_index);
  }

  inline TinyScalar get_tau_for_link(const std::vector<TinyScalar>& tau,
                                     int link_index) const {
    if (tau.empty()) return TinyConstants::zero();
    const TinyLink& link = m_links[link_index];
    int offset = m_isFloating ? -6 : 0;
    return link.m_joint_type == JOINT_FIXED ? TinyConstants::zero()
                                            : tau[link.m_qd_index + offset];
  }
  inline TinyScalar get_tau_for_link(int link_index) const {
    return get_tau_for_link(m_tau, link_index);
  }

  /**
   * Set joint torques and external forces in all links and the base to zero.
   */
  void clear_forces() {
    m_baseAppliedForce.set_zero();
    for (TinyLink& link : m_links) {
      link.m_f_ext.set_zero();
    }
    for (int i = 0; i < m_dof; ++i) {
      m_tau[i] = TinyConstants::zero();
    }
  }

  /**
   * Computes the forward kinematics given joint positions q and assigns
   * base_X_world the base transform in world frame, and optionally the link
   * transforms in world and in base frame.
   * Input q must have dimensions of dof().
   */
  void forward_kinematics_q(
      const std::vector<TinyScalar>& q, TinySpatialTransform* base_X_world,
      std::vector<TinySpatialTransform>* links_X_world = nullptr,
      std::vector<TinySpatialTransform>* links_X_base = nullptr) const {
    assert(q.size() == dof());
    assert(base_X_world != nullptr);

    if (m_isFloating) {
      base_X_world->m_rotation.setRotation(
          TinyQuaternion(q[0], q[1], q[2], q[3]));
      base_X_world->m_translation.setValue(q[4], q[5], q[6]);
    } else {
      *base_X_world = m_base_X_world;
    }
    if (links_X_world) links_X_world->resize(m_links.size());
    if (links_X_base) links_X_base->resize(m_links.size());
    TinySpatialTransform x_j;
    TinySpatialTransform x_parent;
    TinySpatialTransform ident;
    ident.set_identity();
    for (int i = 0; i < m_links.size(); i++) {
      const TinyLink& link = m_links[i];
      int parent = link.m_parent_index;

      TinyScalar q_val = get_q_for_link(q, i);
      link.jcalc(q_val, &x_j, &x_parent);

      if (parent >= 0 || m_isFloating) {
        if (links_X_world) {
          const TinySpatialTransform& parent_X_world =
              parent >= 0 ? (*links_X_world)[parent] : *base_X_world;
          (*links_X_world)[i] = parent_X_world * x_parent;
        }
        if (links_X_base) {
          const TinySpatialTransform& parent_X_base =
              parent >= 0 ? (*links_X_base)[parent] : ident;
          (*links_X_base)[i] = parent_X_base * x_parent;
        }
      } else {
        // first link in fixed-base system
        if (links_X_world) (*links_X_world)[i] = *base_X_world * x_parent;
        if (links_X_base) (*links_X_base)[i] = x_parent;
      }
    }
  }

  /**
   * Computes generalized bias force C (also called "nonlinear effects") equal
   * to the amount of generalized force that has to be applied to the system
   * such that the resultant generalized acceleration qdd is zero.
   * This is useful for gravity compensation.
   * Also computes the base bias force pC_0 (i.e. m_baseBiasForce).
   *
   * C, pC_0 = ID(q, qd, 0)
   */
  void bias_forces(const std::vector<TinyScalar>& q,
                   const std::vector<TinyScalar>& qd, TinyVectorX* C,
                   const TinySpatialMotionVector& gravity,
                   TinySpatialMotionVector* pC_0 = nullptr) {
    assert(q.empty() || q.size() == dof());
    assert(qd.empty() || qd.size() == dof_qd());
    assert(C != nullptr);
    assert(C->m_size == m_links.size());

    m_baseAcceleration = gravity;

    forward_kinematics(q, qd);

    m_baseForce = m_baseInertia.mul_inv(m_baseAcceleration) + m_baseBiasForce;

    for (int i = m_links.size() - 1; i >= 0; --i) {
      (*C)[i] = m_links[i].m_S.dot(m_links[i].m_f);
      int parent = m_links[i].m_parent_index;
      TinySpatialMotionVector& parent_f =
          parent >= 0 ? m_links[parent].m_f : m_baseForce;
      parent_f +=
          m_links[i].m_X_parent2.apply_inverse_transpose(m_links[i].m_f);
    }
    m_baseBiasForce = m_baseForce;
    if (pC_0) *pC_0 = m_baseBiasForce;
  }

  /**
   * Composite Rigid Body Algorithm (CRBA) to compute the joint space inertia
   * matrix. M must be a properly initialized square matrix of size dof_qd().
   * The inertia matrix is computed in the base frame.
   */
  void mass_matrix(const std::vector<TinyScalar>& q, TinyMatrixXxX* M) {
    assert(q.size() == dof());
    assert(M != nullptr);
    int n = m_links.size();
    // printf("n is %i\n", n);
    assert(M->m_rows == dof_qd());
    assert(M->m_cols == dof_qd());

    forward_kinematics(q);

    M->set_zero();
    for (int i = n - 1; i >= 0; --i) {
      int parent = m_links[i].m_parent_index;
      const TinySymmetricSpatialDyad& Ic = m_links[i].m_IA;
      const TinySpatialTransform& Xp = m_links[i].m_X_parent2;
      if (parent >= 0) {
        m_links[parent].m_IA += TinySymmetricSpatialDyad::shift(Ic, Xp);
      } else if (m_isFloating) {
        m_baseArticulatedInertia += TinySymmetricSpatialDyad::shift(Ic, Xp);
      }
      TinySpatialMotionVector Fi = Ic.mul_inv(m_links[i].m_S);
      int qd_i = m_links[i].m_qd_index;
      if (m_links[i].m_joint_type == JOINT_FIXED) continue;

      (*M)(qd_i, qd_i) = m_links[i].m_S.dot(Fi);

      int j = i;
      while (m_links[j].m_parent_index != -1) {
        Fi = m_links[j].m_X_parent2.apply_transpose(Fi);
        j = m_links[j].m_parent_index;
        if (m_links[j].m_joint_type == JOINT_FIXED) continue;
        int qd_j = m_links[j].m_qd_index;
        (*M)(qd_i, qd_j) = Fi.dot(m_links[j].m_S);
        (*M)(qd_j, qd_i) = (*M)(qd_i, qd_j);
      }

      if (m_isFloating) {
        Fi = m_links[j].m_X_parent2.apply_transpose(Fi);
        M->assign_vector_vertical(0, qd_i, Fi);
        M->assign_vector_horizontal(qd_i, 0, Fi);
      }
    }
    if (m_isFloating) {
      // assign Ic_0 to M(0:6, 0:6)
      M->assign_matrix(0, 0, m_baseArticulatedInertia.m_topLeftMat);
      M->assign_matrix(0, 3, m_baseArticulatedInertia.m_topRightMat);
      M->assign_matrix(3, 0, m_baseArticulatedInertia.m_bottomLeftMat);
      M->assign_matrix(3, 3, m_baseArticulatedInertia.m_bottomRightMat);
    }
  }

  /**
   * Composite Rigid Body Algorithm (CRBA) to compute the joint space inertia
   * matrix. M must be a properly initialized square matrix of size dof_qd().
   */
  void mass_matrix(TinyMatrixXxX* M) { mass_matrix(m_q, M); }

  /**
   * LTDL factorization exploiting branch-induced sparsity pattern in
   * inertia matrix M. See Table 6.3 in Featherstone 2008.
   */
  void factorize_LTDL(TinyMatrixXxX* M) {
    assert(M->m_rows == dof_qd());
    assert(M->m_cols == dof_qd());
    assert(!m_isFloating);
    // TODO support floating base
    for (int k = m_links.size() - 1; k >= 0; --k) {
      int i = m_links[k].m_parent_index;
      while (i != 0) {
        const TinyScalar a = (*M)(k, i) / (*M)(k, k);
        int j = i;
        while (j != 0) {
          (*M)(i, j) -= a * (*M)(k, j);
          j = m_links[j].m_parent_index;
        }
        (*M)(k, i) = a;
        i = m_links[i].m_parent_index;
      }
    }
  }

  /**
   * Computes x = inv(L)*x for lower-triangular matrix L exploiting
   * multi-body tree structure. Updates x in place.
   */
  void multiply_inv_L(const TinyMatrixXxX& L, TinyVectorX* x) {
    assert(L.m_rows == dof_qd());
    assert(L.m_cols == dof_qd());
    assert(x->m_rows == dof_qd());
    assert(!m_isFloating);
    // TODO support floating base
    for (int i = 0; i < m_links.size(); ++i) {
      int j = m_links[i].m_parent_index;
      while (j != 0) {
        (*x)[i] -= L(i, j) * (*x)[j];
        j = m_links[j].m_parent_index;
      }
      x[i] = x[i] / L(i, i);
    }
  }

  /**
   * Implements the first phase in ABA, CRBA and RNEA, that computes the
   * joint and body transforms, velocities and bias forces.
   * Initializes articulated inertia with the local body inertia.
   *
   * Joint positions q must have dimension of dof().
   * Joint velocities qd must have dimension of dof_qd().
   * If no joint velocities qd are given, qd is assumed to be zero.
   * If no joint accelerations qdd are given, qdd is assumed to be zero.
   */
  void forward_kinematics(
      const std::vector<TinyScalar>& q,
      const std::vector<TinyScalar>& qd = std::vector<TinyScalar>(),
      const std::vector<TinyScalar>& qdd = std::vector<TinyScalar>()) {
    assert(q.size() == dof());
    assert(qd.empty() || qd.size() == dof_qd());
    assert(qdd.empty() || qdd.size() == dof_qd());

    if (m_isFloating) {
      // update base-world transform from q, and update base velocity from qd
      m_base_X_world.m_rotation.setRotation(
          TinyQuaternion(q[0], q[1], q[2], q[3]));
      m_base_X_world.m_translation.setValue(q[4], q[5], q[6]);
      if (!qd.empty()) {
        m_baseVelocity.m_topVec = TinyVector3(qd[0], qd[1], qd[2]);
        m_baseVelocity.m_bottomVec = TinyVector3(qd[3], qd[4], qd[5]);
      } else {
        m_baseVelocity.set_zero();
      }

      TinySpatialMotionVector I0_mul_v0 = m_baseInertia.mul_inv(m_baseVelocity);
      m_baseBiasForce = m_baseVelocity.crossf(I0_mul_v0) - m_baseAppliedForce;

      m_baseArticulatedInertia = m_baseInertia;
    }

    for (int i = 0; i < m_links.size(); i++) {
      TinyLink& link = m_links[i];
      int parent = link.m_parent_index;

      // update joint transforms, joint velocity (if available)
      TinyScalar q_val = get_q_for_link(q, i);
      TinyScalar qd_val = get_qd_for_link(qd, i);
      link.jcalc(q_val, qd_val);

      if (parent >= 0 || m_isFloating) {
        const TinySpatialTransform& parent_X_world =
            parent >= 0 ? m_links[parent].m_X_world : m_base_X_world;
        link.m_X_world = parent_X_world * link.m_X_parent2;
        const TinySpatialMotionVector& parentVelocity =
            parent >= 0 ? m_links[parent].m_v : m_baseVelocity;
        TinySpatialMotionVector xv = link.m_X_parent2.apply(parentVelocity);
        link.m_v = xv + link.m_vJ;
      } else {
        link.m_X_world = m_base_X_world * link.m_X_parent2;
        link.m_v = link.m_vJ;
      }
      TinySpatialMotionVector v_x_vJ = link.m_v.crossm(link.m_vJ);
      link.m_c = v_x_vJ /*+link.c_J[i]*/;

      link.m_IA = link.m_I;
      TinySpatialMotionVector I_mul_v = link.m_I.mul_inv(link.m_v);
      TinySpatialMotionVector f_ext =
          link.m_X_world.apply_inverse_transpose(link.m_f_ext);
      link.m_pA = link.m_v.crossf(I_mul_v) - f_ext;
#ifdef DEBUG
      link.m_IA.print("link.m_IA");
      I_mul_v.print("I_mul_v");
      link.m_pA.print("link.m_pA");
#endif
      // compute helper temporary variables for floating-base RNEA
      const TinySpatialMotionVector& parent_a =
          parent >= 0 ? m_links[parent].m_a : m_baseAcceleration;
      link.m_a = link.m_X_parent2.apply(parent_a) + v_x_vJ;
      if (!qdd.empty()) {
        link.m_a += link.m_S * get_qdd_for_link(qdd, i);
      }
      link.m_f = link.m_I.mul_inv(link.m_a) + link.m_pA;
    }
  }

  /**
   * Updates the forward kinematics given the q, qd coordinates stored in this
   * model.
   */
  void forward_kinematics() { forward_kinematics(m_q, m_qd); }
  void forward_kinematics1() { forward_kinematics(m_q, m_qd); }

  void forward_dynamics(const TinyVector3& gravity) {
    forward_dynamics(m_q, m_qd, m_tau, gravity, m_qdd);
  }

  void forward_dynamics1(const std::vector<TinyScalar>& q,
                         const std::vector<TinyScalar>& qd,
                         const std::vector<TinyScalar>& tau,
                         const TinyVector3& gravity,
                         std::vector<TinyScalar>& qdd) {
    forward_dynamics(q, qd, tau, gravity, qdd);
  }

  void forward_dynamics(const std::vector<TinyScalar>& q,
                        const std::vector<TinyScalar>& qd,
                        const std::vector<TinyScalar>& tau,
                        const TinyVector3& gravity,
                        std::vector<TinyScalar>& qdd) {
    assert(q.size() == dof());
    assert(qd.size() == dof_qd());
    assert(qdd.size() == dof_qd());
    assert(static_cast<int>(tau.size()) == m_dof);

    TinySpatialMotionVector spatial_gravity(
        TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                    TinyConstants::zero()),
        gravity);

    forward_kinematics(q, qd);

    for (int i = m_links.size() - 1; i >= 0; i--) {
      TinyLink& link = m_links[i];
      int parent = link.m_parent_index;
      link.m_U = link.m_IA.mul_inv(link.m_S);
      link.m_d = link.m_S.dot(link.m_U);
      TinyScalar tau_val = get_tau_for_link(tau, i);
      // apply linear joint stiffness and damping
      // see Eqns. (2.76), (2.78) in Rigid Body Dynamics Notes
      // by Shinjiro Sueda
      // https://github.com/sueda/redmax/blob/master/notes.pdf
      // TODO consider nonzero resting position of joint for stiffness?
      tau_val -= link.m_stiffness * get_q_for_link(q, i);
      tau_val -= link.m_damping * get_qd_for_link(qd, i);

      link.m_u = tau_val - link.m_S.dot(link.m_pA);

#ifdef DEBUG
      link.m_U.print("m_U");
      printf("m_links[%d].m_d=", i);
      double d1 = TinyConstants::getDouble(link.m_d);
      printf("%f\n", d1);
      printf("m_links[%d].m_u=", i);
      double u = TinyConstants::getDouble(link.m_u);
      printf("%f\n", u);
#endif

      TinyScalar invd = link.m_joint_type == JOINT_FIXED
                            ? TinyConstants::zero()
                            : TinyConstants::one() / link.m_d;
#ifdef DEBUG
      printf("invd[%d]=%f\n", i, TinyConstants::getDouble(invd));
#endif
      TinySymmetricSpatialDyad tmp =
          TinySymmetricSpatialDyad::vTimesvTranspose(link.m_U * invd, link.m_U);

      TinySymmetricSpatialDyad Ia = link.m_IA;
      Ia -= tmp;
      TinySpatialMotionVector tmp2 = Ia.mul_inv(link.m_c);
      TinySpatialMotionVector pa =
          link.m_pA + tmp2 + link.m_U * (link.m_u * invd);
#ifdef DEBUG
      Ia.print("Ia-tmp");
      tmp2.print("tmp2");
      pa.print("pa");
#endif

      TinySpatialMotionVector dpA = link.m_X_parent2.apply_transpose(pa);
#ifdef DEBUG
      dpA.print("dpA");
#endif
      TinySymmetricSpatialDyad dI =
          TinySymmetricSpatialDyad::shift(Ia, link.m_X_parent2);
      if (parent >= 0) {
        m_links[parent].m_pA += dpA;
        m_links[parent].m_IA += dI;
#ifdef DEBUG
        m_links[parent].m_pA.print("pa update");
        m_links[parent].m_I.print("mIA");
#endif
      } else if (m_isFloating) {
        m_baseBiasForce += dpA;
        m_baseArticulatedInertia += dI;
#ifdef DEBUG
        m_baseArticulatedInertia.print("m_baseArticulatedInertia");
        m_baseBiasForce.print("m_baseBiasForce");
        dI.print("dI");
        dpA.print("dpA");
#endif
      }
    }

    if (m_isFloating) {
      // m_baseArticulatedInertia.print("FINAL BASE ARTICULATED INERTIA");
      // {
      //   auto temp = m_baseArticulatedInertia.inverse();
      //   temp.print("Inverted base articulated inertia");
      // }
      // {
      //   auto temp = m_baseArticulatedInertia.inverseOld();
      //   temp.print("OLD Inverted base articulated inertia");
      // }

      m_baseAcceleration =
          -m_baseArticulatedInertia.inverse().mul_inv(m_baseBiasForce);

    } else {
      m_baseAcceleration = -spatial_gravity;
    }

    for (int i = 0; i < m_links.size(); i++) {
      TinyLink& link = m_links[i];
      int parent = link.m_parent_index;
      const TinySpatialTransform& X_parent = link.m_X_parent2;
      const TinySpatialMotionVector& parentAccel =
          (parent >= 0) ? m_links[parent].m_a : m_baseAcceleration;
      // if (parent < 0) {
      //   printf("final loop for parent %i\n", parent);
      //   m_baseArticulatedInertia.print("m_baseArticulatedInertia");
      //   m_baseBiasForce.print("m_baseBiasForce");
      //   parentAccel.print("parentAccel");
      // }

      TinySpatialMotionVector xpa = X_parent.apply(parentAccel);
      link.m_a = xpa + link.m_c;
      // xpa.print("xpa");
      // link.m_a.print("a'");
      // model.a[i] = X_parent.apply(model.a[parent]) + model.c[i];
      // LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

      // if (model.mJoints[i].mDoFCount == 1
      {
        TinyScalar invd = link.m_joint_type == JOINT_FIXED
                              ? TinyConstants::zero()
                              : TinyConstants::one() / link.m_d;
        TinyScalar t1 = link.m_U.dot(link.m_a);
        TinyScalar t2 = link.m_u - t1;
        TinyScalar qdd_val = TinyConstants::zero();
        if (link.m_qd_index >= 0) {
          qdd_val = invd * t2;
          qdd[link.m_qd_index] = qdd_val;
        }
        link.m_a = link.m_a + link.m_S * qdd_val;
        // link.m_a.print("a");
      }
    }
    if (m_isFloating) {
      m_baseAcceleration += spatial_gravity;
      for (int i = 0; i < 6; i++) {
        qdd[i] = m_baseAcceleration[i];
      }
    } else {
      m_baseAcceleration.set_zero();
    }
  }

  /**
   * Computes the joint torques to achieve the given joint velocities and
   * accelerations. To compute gravity compensation terms, set qd, qdd to zero
   * and pass in negative gravity. To compute only the Coriolis and centrifugal
   * terms, set gravity, qdd and external forces to zero while keeping the joint
   * velocities qd unchanged.
   * @param q Joint positions.
   * @param qd Joint velocities.
   * @param qdd Joint accelerations.
   * @param gravity Gravity.
   * @param tau Joint forces (output).
   */
  void inverse_dynamics(const std::vector<TinyScalar>& q,
                        const std::vector<TinyScalar>& qd,
                        const std::vector<TinyScalar>& qdd,
                        const TinyVector3& gravity,
                        std::vector<TinyScalar>& tau) {
    assert(q.size() == dof());
    assert(qd.size() == dof_qd());
    assert(qdd.size() == dof_qd());
    assert(static_cast<int>(tau.size()) == m_dof);

    // in the following, the variable names for articulated terms I^A, p^A are
    // used for composite rigid body terms I^c, p^c to avoid introducing more
    // variables

    TinySpatialMotionVector spatial_gravity(
        TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                    TinyConstants::zero()),
        gravity);

    m_baseAcceleration = spatial_gravity;
    forward_kinematics(q, qd, qdd);

    if (!m_isFloating) {
      int tau_index = m_dof - 1;
      for (int i = static_cast<int>(m_links.size() - 1); i >= 0; i--) {
        TinyLink& link = m_links[i];
        int parent = link.m_parent_index;
        if (link.m_joint_type != JOINT_FIXED) {
          tau[tau_index] = link.m_S.dot(link.m_f);
          --tau_index;
        }
        if (parent >= 0) {
          m_links[parent].m_f += link.m_X_parent2.apply_transpose(link.m_f);
        }
      }
      return;
    }

    // I_0^c, p_0^c are (partially) computed by forward_kinematics
    m_baseBiasForce += m_baseInertia.mul_inv(m_baseAcceleration);

    for (int i = static_cast<int>(m_links.size() - 1); i >= 0; i--) {
      TinyLink& link = m_links[i];
      int parent = link.m_parent_index;
      TinySymmetricSpatialDyad& parent_Ic =
          parent >= 0 ? m_links[parent].m_IA : m_baseArticulatedInertia;
      // forward kinematics computes composite rigid-body bias force p^c as f
      TinySpatialMotionVector& parent_pc =
          parent >= 0 ? m_links[parent].m_f : m_baseBiasForce;
      parent_Ic += TinySymmetricSpatialDyad::shift(link.m_IA, link.m_X_parent2);
      parent_pc += link.m_X_parent2.apply_transpose(link.m_f);
    }

    m_baseAcceleration =
        -m_baseArticulatedInertia.inverse().mul_inv(m_baseBiasForce);

    int tau_index = 0;
    for (int i = 0; i < static_cast<int>(m_links.size()); i++) {
      TinyLink& link = m_links[i];
      //!!! The implementation is different from Featherstone Table 9.6, the
      //!!! commented-out lines correspond to the book implementation that
      //!!! leads to forces too low to compensate gravity in the joints (see
      //!!! gravity_compensation target).
      //      int parent = link.m_parent_index;
      //      const TinySpatialMotionVector& parent_a =
      //          (parent >= 0) ? m_links[parent].m_a : m_baseAcceleration;
      //
      //      link.m_a = link.m_X_parent2.apply(parent_a);

      if (link.m_joint_type != JOINT_FIXED) {
        tau[tau_index] = link.m_S.dot(
            link.m_f);  // link.m_S.dot(link.m_IA.mul_inv(link.m_a) + link.m_f);
        ++tau_index;
      }
    }
  }

  void integrate(TinyScalar dt) { integrate(m_q, m_qd, m_qdd, dt); }

  void integrate_q(TinyScalar dt) {
    std::vector<TinyScalar>& qd = m_qd;
    std::vector<TinyScalar>& qdd = m_qdd;

    assert(static_cast<int>(qd.size()) == dof_qd());
    assert(static_cast<int>(qdd.size()) == dof_qd());
    m_baseVelocity += m_baseAcceleration * dt;
    m_baseAcceleration.set_zero();

    int qd_offset;
    if (m_isFloating) {
      qd_offset = 3;

    } else {
      qd_offset = 0;
    }

    for (int i = 0; i < dof_qd() - qd_offset; i++) {
      int qdindex = i + qd_offset;
      qd[qdindex] += qdd[qdindex] * dt;
      qdd[qdindex] = TinyConstants::zero();
    }
    if (m_isFloating) {
      for (int i = 0; i < 6; i++) {
        m_qd[i] = m_baseVelocity[i];
        m_qdd[i] = TinyConstants::zero();
      }
    }
  }

  void integrate1(std::vector<TinyScalar>& q, std::vector<TinyScalar>& qd,
                  const std::vector<TinyScalar>& qdd, TinyScalar dt) {
    integrate(q, qd, qdd, dt);
  }
  void integrate(std::vector<TinyScalar>& q, std::vector<TinyScalar>& qd,
                 const std::vector<TinyScalar>& qdd, TinyScalar dt) {
    assert(static_cast<int>(q.size()) == dof());
    assert(static_cast<int>(qd.size()) == dof_qd());
    assert(static_cast<int>(qdd.size()) == dof_qd());

    int q_offset, qd_offset;
    if (m_isFloating) {
      m_baseAcceleration.m_topVec.setValue(qdd[0], qdd[1], qdd[2]);
      m_baseAcceleration.m_bottomVec.setValue(qdd[3], qdd[4], qdd[5]);

      m_baseVelocity.m_topVec.setValue(qd[0], qd[1], qd[2]);
      m_baseVelocity.m_bottomVec.setValue(qd[3], qd[4], qd[5]);

      m_baseVelocity += m_baseAcceleration * dt;

      TinyVector3 linear_velocity = m_baseVelocity.m_bottomVec;
      m_base_X_world.m_translation += linear_velocity * dt;

      // update base orientation using Quaternion derivative
      TinyVector3 angular_velocity = m_baseVelocity.m_topVec;

      TinyQuaternion base_rot;
      m_base_X_world.m_rotation.getRotation(base_rot);
      // update 4-dimensional q from 3-dimensional qd for the base rotation
      // angular_velocity.setValue(qd[0], qd[1], qd[2]);
      base_rot += (angular_velocity * base_rot) * (dt * TinyConstants::half());
      base_rot.normalize();
      m_base_X_world.m_rotation.setRotation(base_rot);

      q[0] = base_rot.getX();
      q[1] = base_rot.getY();
      q[2] = base_rot.getZ();
      q[3] = base_rot.getW();
      q_offset = 4;
      qd_offset = 3;
    } else {
      q_offset = 0;
      qd_offset = 0;
    }

    if (m_integration_type == INT_EULER_SYMPLECTIC) {
      for (int i = 0; i < dof_qd() - qd_offset; i++) {
        int qindex = i + q_offset;
        int qdindex = i + qd_offset;
        qd[qdindex] += qdd[qdindex] * dt;
        q[qindex] += qd[qdindex] * dt;
      }
    } else if (m_integration_type == INT_EULER) {
      for (int i = 0; i < dof_qd() - qd_offset; i++) {
        int qindex = i + q_offset;
        int qdindex = i + qd_offset;
        q[qindex] += qd[qdindex] * dt;
        qd[qdindex] += qdd[qdindex] * dt;
      }
    }
  }

  /**
   * Transforms a point in body coordinates to world coordinates.
   */
  inline TinyVector3 body_to_world(int link_index,
                                   const TinyVector3& point) const {
    return get_world_transform(link_index).apply(point);
  }
  /**
   * Transforms a point in world coordinates to bodyt coordinates.
   */
  inline TinyVector3 world_to_body(int link_index,
                                   const TinyVector3& point) const {
    return get_world_transform(link_index).apply_inverse(point);
  }

  /**
   * Computes the body Jacobian for a point in world frame on certain link.
   * This function uses the internal body world frames that have been computed
   * by forward kinematics before.
   */
  TinyMatrix3xX point_jacobian(int link_index, const TinyVector3& point) const {
    return point_jacobian(m_q, link_index, point);
  }
  TinyMatrix3xX point_jacobian1(int link_index,
                                const TinyVector3& point) const {
    return point_jacobian(m_q, link_index, point);
  }

  /**
   * Computes the body Jacobian for a point in world frame on certain link.
   * This function does not update the robot configuration with the given
   * joint positions.
   */
  TinyMatrix3xX point_jacobian(const std::vector<TinyScalar>& q, int link_index,
                               const TinyVector3& world_point) const {
    assert(q.size() == dof());
    assert(link_index < static_cast<int>(m_links.size()));
    TinyMatrix3xX jac(3, dof_qd());
    jac.set_zero();
    std::vector<TinySpatialTransform> links_X_world;
    std::vector<TinySpatialTransform> links_X_base;
    TinySpatialTransform base_X_world;
    forward_kinematics_q(q, &base_X_world, &links_X_world, &links_X_base);
    TinySpatialTransform point_tf;
    point_tf.set_identity();
    point_tf.m_translation = world_point;
    if (m_isFloating) {
      // see (Eq. 2.238) in
      // https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/FloatingBaseKinematics.pdf
      TinyVector3 base_to_point = world_point - base_X_world.m_translation;
      TinyMatrix3x3 cr = TinyVectorCrossMatrix(base_to_point);
      jac[0] = cr[0];
      jac[1] = cr[1];
      jac[2] = cr[2];
      jac[3][0] = TinyConstants::one();
      jac[4][1] = TinyConstants::one();
      jac[5][2] = TinyConstants::one();
    } else {
      point_tf.m_translation = world_point;
    }
    // loop over all links that lie on the path from the given link to world
    if (link_index >= 0) {
      const TinyLink* body = &m_links[link_index];
      while (true) {
        int i = body->m_index;
        if (body->m_joint_type != JOINT_FIXED) {
          TinySpatialMotionVector st =
              links_X_world[i].apply_inverse(body->m_S);
          TinySpatialMotionVector xs = point_tf.apply(st);
          jac[body->m_qd_index] = xs.m_bottomVec;
        }
        if (body->m_parent_index < 0) break;
        body = &m_links[body->m_parent_index];
      }
    }
    return jac;
  }

  /**
   * Estimate the point Jacobian using finite differences.
   * This function should only be called for testing purposes.
   */
  TinyMatrix3xX point_jacobian_fd(
      const std::vector<TinyScalar>& q, int link_index,
      const TinyVector3& start_point,
      TinyScalar eps = TinyConstants::fraction(1, 1000)) const {
    assert(q.size() == dof());
    assert(link_index < static_cast<int>(m_links.size()));
    TinyMatrix3xX jac(3, dof_qd());
    jac.set_zero();
    std::vector<TinySpatialTransform> links_X_world;
    TinySpatialTransform base_X_world;
    // compute world point transform for the initial joint angles
    forward_kinematics_q(q, &base_X_world, &links_X_world);
    // convert start point in world coordinates to link frame
    const TinyVector3 base_point =
        links_X_world[link_index].apply_inverse(start_point);
    TinyVector3 world_point;

    std::vector<TinyScalar> q_x;
    TinySpatialTransform base_X_world_temp;
    for (int i = 0; i < dof_qd(); ++i) {
      q_x = q;
      if (m_isFloating && i < 3) {
        // special handling of quaternion differencing via angular velocity
        TinyQuaternion base_rot;
        base_X_world.m_rotation.getRotation(base_rot);

        TinyVector3 angular_velocity;
        angular_velocity.set_zero();
        angular_velocity[i] = TinyConstants::one();

        base_rot +=
            (angular_velocity * base_rot) * (eps * TinyConstants::half());
        base_rot.normalize();
        q_x[0] = base_rot.getX();
        q_x[1] = base_rot.getY();
        q_x[2] = base_rot.getZ();
        q_x[3] = base_rot.getW();
      } else {
        // adjust for the +1 offset with the 4 DOF orientation in q vs. 3 in qd
        int q_index = m_isFloating ? i + 1 : i;
        q_x[q_index] += eps;
      }
      forward_kinematics_q(q_x, &base_X_world_temp, &links_X_world);
      world_point = links_X_world[link_index].apply(base_point);
      for (int j = 0; j < 3; ++j)
        jac(j, i) = (world_point[j] - start_point[j]) / eps;
    }

    return jac;
  }

  /**
   * Computes the actuator forward dynamics, integrates it, and computes the
   * control output and applies the resulting joint torques.
   * The control output is added to the joint torques, not assigned, so that
   * the forces computed from contact dynamics etc. are not eliminated.
   * If no actuator is defined, the control inputs are directly applied as
   * torques.
   * @param dt Time step in seconds.
   * @param u Control input (must be of dimension `dof_actuated()`).
   */
  void control(const TinyScalar& dt, const std::vector<TinyScalar>& u) {
    if (u.size() != m_control_indices.size()) {
      fprintf(stderr,
              "TinyMultiBody::control has incorrect input dimensions: u (%i) "
              "must match control indices (%i).\n",
              static_cast<int>(u.size()),
              static_cast<int>(m_control_indices.size()));
      return;
    }
    assert(u.size() == m_control_indices.size());
    using std::min, std::max;
    if (!m_actuator) {
      // if no actuator is defined, directly apply control inputs as torques
      for (int i = 0; i < dof_actuated(); ++i) {
        int ci = m_control_indices[i];
        //!!! we add (not assign) the control output
        m_tau[ci] += u[i];
      }
      return;
    }
    // only consider subset of q, dd for the controllable DOF
    std::vector<TinyScalar> q(u.size()), qd(u.size()), tau;
    int q_offset = m_isFloating ? 7 : 0;
    int qd_offset = m_isFloating ? 6 : 0;
    for (int i = 0; i < dof_actuated(); ++i) {
      int ci = m_control_indices[i];
      q[i] = m_q[q_offset + ci];
      qd[i] = m_qd[qd_offset + ci];
    }
    // TODO move integration step out of this function
    m_actuator->integrate(dt, q, qd, u);
    m_actuator->compute_torques(q, qd, u, tau);
    // apply control output to the right joint torques
    for (int i = 0; i < dof_actuated(); ++i) {
      int ci = m_control_indices[i];
      //!!! we add (not assign) the control output
      m_tau[ci] += tau[i];
    }
  }
};

#endif  // TINY_MULTIBODY_H
