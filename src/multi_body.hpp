#ifndef _MULTI_BODY_HPP
#define _MULTI_BODY_HPP

#pragma once

#include <vector>

#include "geometry.hpp"
#include "link.hpp"

namespace tds {
template <typename Algebra>
class MultiBody {
  template <typename OtherAlgebra>
  friend class MultiBody;
  template <typename>
  friend struct UrdfToMultiBody;

  using Scalar = typename Algebra::Scalar;
  using Index = typename Algebra::Index;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using Matrix3 = typename Algebra::Matrix3;
  using Matrix6 = typename Algebra::Matrix6;
  using Quaternion = typename Algebra::Quaternion;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::MotionVector<Algebra> MotionVector;
  typedef tds::ForceVector<Algebra> ForceVector;
  typedef tds::Link<Algebra> Link;
  typedef tds::RigidBodyInertia<Algebra> RigidBodyInertia;
  typedef tds::ArticulatedBodyInertia<Algebra> ArticulatedBodyInertia;
  typedef tds::Geometry<Algebra> Geometry;
  typedef std::vector<Link> LinkCollection;

  /**
   * Number of positional degrees of freedom, excluding floating-base
   * coordinates.
   */
  int dof_q_{0};

  /**
   * Number of velocity/acceleration degrees of freedom, excluding floating-base
   * coordinates.
   */
  int dof_qd_{0};

  /**
   * Damping factor, applied to spherical joints, see integrator.hpp
   */

  Scalar joint_damping_{0.995};

  /**
   * Whether this system is floating or fixed to the world frame.
   */
  bool is_floating_{false};

  /**
   * Indices in `tau` that are controllable, i.e. actuated.
   * For floating-base system, the index 0 corresponds to the first degree of
   * freedom not part of the 6D floating-base coordinates.
   */
  std::vector<int> control_indices_;

  // quantities related to floating base
  mutable MotionVector base_velocity_;       // v_0
  mutable MotionVector base_acceleration_;   // a_0
  ForceVector base_applied_force_;           // f_ext_0 in world frame
  mutable ForceVector base_force_;           // f_0 (used by RNEA)
  mutable ForceVector base_bias_force_;      // pA_0
  RigidBodyInertia base_rbi_;                // I_0
  mutable ArticulatedBodyInertia base_abi_;  // IA_0
  mutable Transform base_X_world_;

  std::vector<int> visual_shape_uids_;
  std::vector<int> visual_instance_uids_;

  // offset of geometry (relative to the base frame)
  std::vector<Transform> X_visuals_;

  std::vector<const Geometry *> collision_geometries_;
  // offset of collision geometries (relative to this link frame)
  std::vector<Transform> X_collisions_;

 public:
  VectorX q_, qd_, qdd_, tau_;

  std::string name_;

  LinkCollection links_;

  explicit MultiBody(bool isFloating = false) : is_floating_(isFloating) {}

  template <typename AlgebraTo = Algebra>
  MultiBody<AlgebraTo> clone() const {
    typedef Conversion<Algebra, AlgebraTo> C;
    MultiBody<AlgebraTo> conv(is_floating_);
    conv.name_ = name_;
    conv.dof_q_ = dof_q_;
    conv.dof_qd_ = dof_qd_;
    conv.q_ = C::convert(q_);
    conv.qd_ = C::convert(qd_);
    conv.qdd_ = C::convert(qdd_);
    conv.tau_ = C::convert(tau_);
    conv.joint_damping_ = C::convert(joint_damping_);
    conv.control_indices_ = control_indices_;
    for (const auto &link : links_) {
      conv.links_.push_back(link.template clone<AlgebraTo>());
    }
    conv.control_indices_ = control_indices_;
    conv.visual_shape_uids_ = visual_shape_uids_;
    for (const auto &x : X_visuals_) {
      conv.X_visuals_.push_back(x.template clone<AlgebraTo>());
    }
    for (const auto &x : X_collisions_) {
      conv.X_collisions_.push_back(x.template clone<AlgebraTo>());
    }
    for (const auto *geom : collision_geometries_) {
      conv.collision_geometries_.push_back(
          tds::clone<Algebra, AlgebraTo>(geom));
    }
    conv.base_rbi_ = base_rbi_.template clone<AlgebraTo>();
    conv.base_applied_force_ = base_applied_force_.template clone<AlgebraTo>();
    return conv;
  }

  std::string &name() { return name_; }
  const std::string &name() const { return name_; }

  TINY_INLINE const LinkCollection &links() const { return links_; }
  TINY_INLINE std::size_t num_links() const { return links_.size(); }
  TINY_INLINE std::size_t size() const { return links_.size(); }

  TINY_INLINE const Link &operator[](std::size_t i) const { return links_[i]; }
  TINY_INLINE Link &operator[](std::size_t i) { return links_[i]; }
  TINY_INLINE typename LinkCollection::iterator begin() {
    return links_.begin();
  }
  TINY_INLINE typename LinkCollection::iterator end() { return links_.end(); }
  TINY_INLINE typename LinkCollection::const_iterator begin() const {
    return links_.begin();
  }
  TINY_INLINE typename LinkCollection::const_iterator end() const {
    return links_.end();
  }
  TINY_INLINE typename LinkCollection::const_iterator cbegin() const {
    return links_.cbegin();
  }
  TINY_INLINE typename LinkCollection::const_iterator cend() const {
    return links_.cend();
  }
  TINY_INLINE bool empty() const { return links_.empty(); }

  /**
   * Dimensionality of joint positions q (including 7-DoF floating-base
   * coordinates if this system is floating-base).
   */
  TINY_INLINE int dof() const { 
      return is_floating_ ? dof_q_ + 7 : dof_q_; 
  }
  /**
   * Dimensionality of joint velocities qd and accelerations qdd (including
   * 6-DoF base velocity and acceleration, if this system is floating-base).
   */
  TINY_INLINE int dof_qd() const {
    return is_floating_ ? dof_qd_ + 6 : dof_qd_;
  }

  /**
   * Dimensionality of control input, i.e. number of actuated DOFs.
   */
  TINY_INLINE int dof_actuated() const {
    return static_cast<int>(control_indices_.size());
  }

  TINY_INLINE bool is_floating() const { return is_floating_; }
  void set_floating_base(bool is_floating = true) {
    is_floating_ = is_floating;
  }

  TINY_INLINE std::vector<int> control_indices() { return control_indices_; }
  void set_control_indices(const std::vector<int> &control_indices) {
    control_indices_ = control_indices;
    tau_ = Algebra::zerox(control_indices.size());
  }

  TINY_INLINE VectorX &q() { return q_; }
  TINY_INLINE const VectorX &q() const { return q_; }
  TINY_INLINE Scalar &q(Index i) { return q_[i]; }
  TINY_INLINE const Scalar &q(Index i) const { return q_[i]; }
  TINY_INLINE VectorX &qd() { return qd_; }
  TINY_INLINE const VectorX &qd() const { return qd_; }
  TINY_INLINE Scalar &qd(Index i) { return qd_[i]; }
  TINY_INLINE const Scalar &qd(Index i) const { return qd_[i]; }
  TINY_INLINE VectorX &qdd() { return qdd_; }
  TINY_INLINE const VectorX &qdd() const { return qdd_; }
  TINY_INLINE Scalar &qdd(Index i) { return qdd_[i]; }
  TINY_INLINE const Scalar &qdd(Index i) const { return qdd_[i]; }
  TINY_INLINE VectorX &tau() { return tau_; }
  TINY_INLINE const VectorX &tau() const { return tau_; }
  TINY_INLINE Scalar &tau(Index i) { return tau_[i]; }
  TINY_INLINE const Scalar &tau(Index i) const { return tau_[i]; }

  TINY_INLINE MotionVector &base_velocity() { return base_velocity_; }
  TINY_INLINE const MotionVector &base_velocity() const {
    return base_velocity_;
  }
  TINY_INLINE MotionVector &base_acceleration() { return base_acceleration_; }
  TINY_INLINE const MotionVector &base_acceleration() const {
    return base_acceleration_;
  }
  TINY_INLINE ForceVector &base_applied_force() { return base_applied_force_; }
  TINY_INLINE const ForceVector &base_applied_force() const {
    return base_applied_force_;
  }
  TINY_INLINE ForceVector &base_force() { return base_force_; }
  TINY_INLINE const ForceVector &base_force() const { return base_force_; }
  TINY_INLINE ForceVector &base_bias_force() { return base_bias_force_; }
  TINY_INLINE const ForceVector &base_bias_force() const {
    return base_bias_force_;
  }
  TINY_INLINE RigidBodyInertia &base_rbi() { return base_rbi_; }
  TINY_INLINE const RigidBodyInertia &base_rbi() const { return base_rbi_; }
  TINY_INLINE ArticulatedBodyInertia &base_abi() { return base_abi_; }
  TINY_INLINE const ArticulatedBodyInertia &base_abi() const {
    return base_abi_;
  }
  TINY_INLINE Transform &base_X_world() { return base_X_world_; }
  TINY_INLINE const Transform &base_X_world() const { return base_X_world_; }

  TINY_INLINE std::vector<int> &visual_shape_ids() {
    return visual_shape_uids_;
  }
  TINY_INLINE const std::vector<int> &visual_shape_ids() const {
    return visual_shape_uids_;
  }

  TINY_INLINE std::vector<int> &visual_instance_uids() {
    return visual_instance_uids_;
  }
  TINY_INLINE const std::vector<int> &visual_instance_uids() const {
    return visual_instance_uids_;
  }

  TINY_INLINE std::vector<Transform> &X_visuals() { return X_visuals_; }
  TINY_INLINE const std::vector<Transform> &X_visuals() const {
    return X_visuals_;
  }

  TINY_INLINE std::vector<const Geometry *> &collision_geometries() {
    return collision_geometries_;
  }
  TINY_INLINE const std::vector<const Geometry *> &collision_geometries()
      const {
    return collision_geometries_;
  }
  TINY_INLINE std::vector<const Geometry *> &collision_geometries(int link_id) {
    if (link_id == -1) return collision_geometries_;
    return links_[link_id].collision_geometries;
  }
  TINY_INLINE const std::vector<const Geometry *> &collision_geometries(
      int link_id) const {
    if (link_id == -1) return collision_geometries_;
    return links_[link_id].collision_geometries;
  }
  TINY_INLINE std::vector<Transform> &collision_transforms() {
    return X_collisions_;
  }
  TINY_INLINE const std::vector<Transform> &collision_transforms() const {
    return X_collisions_;
  }
  TINY_INLINE std::vector<Transform> &collision_transforms(int link_id) {
    if (link_id == -1) return X_collisions_;
    return links_[link_id].X_collisions;
  }
  TINY_INLINE const std::vector<Transform> &collision_transforms(
      int link_id) const {
    if (link_id == -1) return X_collisions_;
    return links_[link_id].X_collisions;
  }

  /**
   * Set 3D base position in world coordinates.
   */
  void set_position(const Vector3 &initial_position) {
    base_X_world_.translation = initial_position;
    if (is_floating_) {
      q_[4] = initial_position[0];
      q_[5] = initial_position[1];
      q_[6] = initial_position[2];
    }
  }

  void set_orientation(const Quaternion &initial_orientation) {
    base_X_world_.rotation = Algebra::quat_to_matrix(initial_orientation);
    if (is_floating_) {
      q_[0] = Algebra::quat_x(initial_orientation);
      q_[1] = Algebra::quat_y(initial_orientation);
      q_[2] = Algebra::quat_z(initial_orientation);
      q_[3] = Algebra::quat_w(initial_orientation);
    }
  }
  void set_orientation(const Matrix3 &rotation) {
    base_X_world_.rotation = rotation;
    if (is_floating_) {
      auto quat = Algebra::matrix_to_quat(rotation);
      q_[0] = Algebra::quat_x(quat);
      q_[1] = Algebra::quat_y(quat);
      q_[2] = Algebra::quat_z(quat);
      q_[3] = Algebra::quat_w(quat);
    }
  }

  Vector3 get_position() const { return get_world_transform(-1).translation; }

  Quaternion get_orientation() const {
    return Algebra::matrix_to_quat(get_world_transform(-1).rotation);
  }

  /**
   * Ensures that the joint coordinates q, qd, qdd, tau are initialized
   * properly in the MultiBody member variables.
   */
  void initialize() {
    // make sure dof and the q / qd indices in the links are accurate
    int q_index = is_floating_ ? 7 : 0;
    int qd_index = is_floating_ ? 6 : 0;
    dof_q_ = 0;   // excludes floating-base DOF
    dof_qd_ = 0;  // excludes floating-base DOF

    for (Link &link : links_) {
      assert(link.index >= 0);
      link.q_index = q_index;
      link.qd_index = qd_index;
      if (link.joint_type == JOINT_SPHERICAL) {
        q_index += 4;
        qd_index += 3;
        dof_q_ += 4;
        dof_qd_ += 3;
      } else if (link.joint_type != JOINT_FIXED) {
        ++q_index;
        ++qd_index;
        ++dof_q_;
        ++dof_qd_;
      } else {
        link.q_index = -2;
        link.qd_index = -2;
      }
    }

    q_ = Algebra::zerox(q_index);
    // Initialize the quaternions for spherical joints
    for (Link &link : links_) {
      if (link.joint_type == JOINT_SPHERICAL) {
        q_[link.q_index + 3] = Algebra::one();
      }
    }
    qd_ = Algebra::zerox(dof_qd());
    qdd_ = Algebra::zerox(dof_qd());
    tau_ = Algebra::zerox(dof_actuated());
    if (is_floating_) {
      q_[3] = Algebra::one();  // make sure orientation is valid
    }

    clear_forces();
    base_acceleration_.set_zero();
    base_abi_ = base_rbi_;

    if (is_floating_ && !base_abi_.is_invertible()) {
      fprintf(stderr,
              "Error: floating-base inertia matrix (ABI) is not invertible. "
              "Are you sure the model should be floating-base?\n");
      // requires to_double, which breaks Algebra::print("Floating-base ABI",
      // base_abi_);
      assert(0);
      exit(1);
    }
  }

  /**
   * Copy constructor. Skips visualization members, temporary variables.
   */
  template <typename Algebra2>
  MultiBody(const MultiBody<Algebra2> &mb)
      : links_(mb.links_),
        dof_q_(mb.dof_q_),
        dof_qd_(mb.dof_qd_),
        control_indices_(mb.control_indices_),
        is_floating_(mb.is_floating_),
        base_velocity_(mb.base_velocity_),
        base_acceleration_(mb.base_acceleration_),
        base_applied_force_(mb.base_applied_force_),
        base_force_(mb.base_force_),
        base_bias_force_(mb.base_bias_force_),
        base_rbi_(mb.base_rbi_),
        base_X_world_(mb.base_X_world_),
        collision_geometries_(mb.collision_geometries_),
        X_collisions_(mb.X_collisions_),
        q_(mb.q_),
        qd_(mb.qd_),
        qdd_(mb.qdd_),
        tau_(mb.tau_) {}

  void print_state(bool print_tau = true, bool print_qdd = true,
                   bool print_qd = true, bool print_q = true) const {
    if (print_q) {
      printf("q: [");
      for (int i = 0; i < dof(); ++i) {
        if (i > 0) printf(" ");
        printf("%.2f", Algebra::to_double(q_[i]));
      }
      printf("]\t");
    }
    if (print_qd) {
      printf("qd: [");
      for (int i = 0; i < dof_qd(); ++i) {
        if (i > 0) printf(" ");
        printf("%.2f", Algebra::to_double(qd_[i]));
      }
      printf("]\t");
    }
    if (print_qdd) {
      printf("qdd: [");
      for (int i = 0; i < dof_qd(); ++i) {
        if (i > 0) printf(" ");
        printf("%.2f", Algebra::to_double(qdd_[i]));
      }
      printf("]\t");
    }
    if (print_tau) {
      printf("tau: [");
      for (int i = 0; i < dof_actuated(); ++i) {
        if (i > 0) printf(" ");
        printf("%.2f", Algebra::to_double(tau_[i]));
      }
      printf("]");
    }
    if (print_q || print_qd || print_qdd || print_tau) {
      printf("\n");
    }
  }

  const Transform &get_world_transform(int link) const {
    if (link == -1) {
      return base_X_world_;
    } else {
      return links_[link].X_world;
    }
  }

  /**
   * Transforms a point in body coordinates to world coordinates.
   */
  inline Vector3 body_to_world(int link_index, const Vector3 &point) const {
    return get_world_transform(link_index).apply(point);
  }
  /**
   * Transforms a point in world coordinates to bodyt coordinates.
   */
  inline Vector3 world_to_body(int link_index, const Vector3 &point) const {
    return get_world_transform(link_index).apply_inverse(point);
  }

  /**
   * Compute center of mass of link in world coordinates.
   * @param link Index of link in `links`.
   * @return 3D coordinates of center of mass in world coordinates.
   */
  const Vector3 get_world_com(int link) const {
    const Transform &tf = get_world_transform(link);
    if (link == -1) {
      return tf.apply(base_rbi_.com);
    } else {
      return tf.apply(links_[link].rbi.com);
    }
  }

  TINY_INLINE void set_q(const VectorX &q) { q_ = q; }

  //  TINY_INLINE Scalar get_q_for_link(const VectorX &q, int link_index) const
  //  {
  //    if (Algebra::size(q) == 0) return Algebra::zero();
  //    const Link &link = links_[link_index];
  //    return link.joint_type == JOINT_FIXED ? Algebra::zero() :
  //    q[link.q_index];
  //  }
  //  TINY_INLINE Scalar get_q_for_link(int link_index) const {
  //    get_q_for_link(q_, link_index);
  //  }
  TINY_INLINE VectorX get_q_for_link(const VectorX &q, int link_index) const {
    const Link &link = links_[link_index];

    if (Algebra::size(q) == 0 || link.joint_type == JOINT_FIXED) {
      return link.joint_type == JOINT_SPHERICAL ? Algebra::zerox(4)
                                                : Algebra::zerox(1);
    }

    return link.joint_type == JOINT_SPHERICAL ? q.segment(link.q_index, 4)
                                              : q.segment(link.q_index, 1);
  }
  TINY_INLINE VectorX get_q_for_link(int link_index) const {
    get_q_for_link(q_, link_index);
  }

  //  TINY_INLINE Scalar get_qd_for_link(const VectorX &qd, int link_index)
  //  const {
  //    if (Algebra::size(qd) == 0) return Algebra::zero();
  //    const Link &link = links_[link_index];
  //    return link.joint_type == JOINT_FIXED ? Algebra::zero() :
  //    qd[link.qd_index];
  //  }
  //  TINY_INLINE Scalar get_qd_for_link(int link_index) const {
  //    return get_qd_for_link(qd_, link_index);
  //  }

  TINY_INLINE VectorX get_qd_for_link(const VectorX &qd, int link_index) const {
    const Link &link = links_[link_index];

    if (Algebra::size(qd) == 0 || link.joint_type == JOINT_FIXED) {
      return link.joint_type == JOINT_SPHERICAL ? Algebra::zerox(3)
                                                : Algebra::zerox(1);
    }

    return link.joint_type == JOINT_SPHERICAL ? qd.segment(link.qd_index, 3)
                                              : qd.segment(link.qd_index, 1);
  }
  TINY_INLINE VectorX get_qd_for_link(int link_index) const {
    return get_qd_for_link(qd_, link_index);
  }

  TINY_INLINE VectorX get_qdd_for_link(const VectorX &qdd,
                                       int link_index) const {
    return get_qd_for_link(qdd, link_index);
  }
  TINY_INLINE VectorX get_qdd_for_link(int link_index) const {
    return get_qdd_for_link(qdd_, link_index);
  }
  //  TINY_INLINE Scalar get_qdd_for_link(const VectorX &qdd,
  //                                      int link_index) const {
  //    return get_qd_for_link(qdd, link_index);
  //  }
  //  TINY_INLINE Scalar get_qdd_for_link(int link_index) const {
  //    return get_qdd_for_link(qdd_, link_index);
  //  }

  //  TINY_INLINE Scalar get_tau_for_link(const VectorX &tau,
  //                                      int link_index) const {
  //    if (Algebra::size(tau) == 0) return Algebra::zero();
  //    const Link &link = links_[link_index];
  //    int offset = is_floating_ ? -6 : 0;
  //    return link.joint_type == JOINT_FIXED ? Algebra::zero()
  //                                          : tau[link.qd_index + offset];
  //  }
  //  TINY_INLINE Scalar get_tau_for_link(int link_index) const {
  //    return get_tau_for_link(tau_, link_index);
  //  }
  TINY_INLINE VectorX get_tau_for_link(const VectorX &tau,
                                       int link_index) const {
    const Link &link = links_[link_index];

    if (Algebra::size(tau) == 0 || link.joint_type == JOINT_FIXED) {
      return link.joint_type == JOINT_SPHERICAL ? Algebra::zerox(3)
                                                : Algebra::zerox(1);
    }
    int offset = is_floating_ ? -6 : 0;

    return link.joint_type == JOINT_SPHERICAL
               ? tau.segment(link.qd_index + offset, 3)
               : tau.segment(link.qd_index + offset, 1);
  }
  TINY_INLINE VectorX get_tau_for_link(int link_index) const {
    return get_tau_for_link(tau_, link_index);
  }

  /**
   * Set joint torques and external forces in all links and the base to zero.
   */
  void clear_forces() {
    base_applied_force_.set_zero();
    for (Link &link : links_) {
      link.f_ext.set_zero();
    }
    for (int i = 0; i < dof_actuated(); ++i) {
      tau_[i] = Algebra::zero();
    }
  }

  static std::string joint_type_name(JointType t) {
    static std::string names[] = {
        "JOINT_FIXED",       "JOINT_PRISMATIC_X",    "JOINT_PRISMATIC_Y",
        "JOINT_PRISMATIC_Z", "JOINT_PRISMATIC_AXIS", "JOINT_REVOLUTE_X",
        "JOINT_REVOLUTE_Y",  "JOINT_REVOLUTE_Z",     "JOINT_REVOLUTE_AXIS",
        "JOINT_SPHERICAL", "JOINT_INVALID",
    };
    return names[int(t) + 1];
  }

  // attaches a new link, setting parent to the last link
  size_t attach(Link &link, bool is_controllable = true) {
    int parent_index = -1;
    if (!links_.empty()) parent_index = static_cast<int>(links_.size()) - 1;
    return attach(link, parent_index, is_controllable);
  }

  size_t attach_link(Link &link, int parent_index,
                     bool is_controllable = true) {
    return attach(link, parent_index, is_controllable);
  }
  size_t attach(Link &link, int parent_index, bool is_controllable = true) {
    int sz = static_cast<int>(links_.size());
    assert(parent_index < sz);
    link.index = sz;
    link.parent_index = parent_index;
    if (link.joint_type == JOINT_SPHERICAL) {
      // How to do this assertion?
      // assert(Algebra::norm(link.S) > Algebra::zero());
      link.q_index = dof();
      link.qd_index = dof_qd();
      dof_q_ += 4;
      dof_qd_ += 3;
      // not sure about this:
      if (is_controllable) {
        if (control_indices_.empty()) {
          control_indices_.push_back(0);
          control_indices_.push_back(control_indices_.back() + 1);
          control_indices_.push_back(control_indices_.back() + 1);
        } else {
          control_indices_.push_back(control_indices_.back() + 1);
          control_indices_.push_back(control_indices_.back() + 1);
          control_indices_.push_back(control_indices_.back() + 1);
        }
      }
    } else if (link.joint_type != JOINT_FIXED) {
      assert(Algebra::norm(link.S) > Algebra::zero());
      // Is this redundant? Or can a multibody be created without a call to
      // initialize()? If redundant, delete? Else, how to fix now that dof !=
      // length q_?
      link.q_index = dof();
      link.qd_index = dof_qd();
      dof_q_++;
      dof_qd_++;
      if (is_controllable) {
        if (control_indices_.empty()) {
          control_indices_.push_back(0);
        } else {
          control_indices_.push_back(control_indices_.back() + 1);
        }
      }
    } else {
      link.q_index = -2;
      link.qd_index = -2;
    }
#ifdef DEBUG
    printf(
        "Attached link %i of type %s (parent: %i, index q: %i, index qd: "
        "%i).\n",
        link.index, joint_type_name(link.joint_type).c_str(), link.parent_index,
        link.q_index, link.qd_index);
//    link.S.print("joint.S");
#endif
    links_.push_back(link);
    return links_.size() - 1;
  }

  void set_joint_damping(Scalar damping) {
    joint_damping_ = Algebra::clamp(damping, 0, 1);
  }
  Scalar joint_damping() const { return joint_damping_; }
};

template <typename AlgebraFrom, typename AlgebraTo = AlgebraFrom>
static TINY_INLINE MultiBody<AlgebraTo> clone(
    const MultiBody<AlgebraFrom> &mb) {
  return mb.template clone<AlgebraTo>();
}
}  // namespace tds
#endif  //_MULTI_BODY_HPP
