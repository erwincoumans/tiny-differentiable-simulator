#pragma once

// A world of URDF robots and objects on a ground plane, built from a
// tds::WorldSpec (world_spec.hpp), with functions that map input arrays to
// output arrays and have no branches on the input values, so that
// world_codegen.hpp can trace them into batched kernels.
//
// state:   q of all bodies, then qd of all bodies (see Body::q_offset).
//          A floating base has q = (quaternion xyzw, position, joints) and
//          qd = (angular velocity, linear velocity in world coordinates,
//          joints).
// control: per controlled body, the PD targets, kp and kd of each actuated
//          joint ("pd"), or the joint torques ("torque").
//
// A time step either runs as one function, step: state, control ->
// next_state, or split into three, so that the iterations of the contact
// solver are not unrolled in the generated code:
//   contact:   state, control -> velocity (the joint velocities after the
//              unconstrained dynamics), mlcp (the matrix A and vector b of
//              the contact problem), jacobian (of the contact velocities),
//              factor (of the mass matrices) and contact_distance
//   pgs:       mlcp, lambda_init -> lambda, one projected Gauss-Seidel
//              iteration, repeated by the kernel (a loop function)
//   integrate: state, velocity, jacobian, factor, lambda -> next_state
// lambda are the contact impulses: the normal impulses of all contacts, then
// the impulses along the first and the second friction direction.
//
// The contact points are the same in every step: each pair of colliding
// geometries gives a fixed number of contact points, which only push when
// their distance is negative. All contacts of all bodies are solved together.
//
// With the solver "external" (WorldSpec::solver), another engine finds and
// solves all contacts of the bodies' links, and the kernels have no
// contacts. Each time step of length dt runs
//   predict:      state, control -> velocity, as the contact function above
//   articulation: state, control -> factor (of the mass matrices, with the
//                 armature of the implicit PD) and dof_axis (see
//                 write_dof_axes), as often as the engine needs them
//   advance:      state, velocity -> next_state (the positions integrated
//                 with the joint velocities that the engine solved) and
//                 links (the poses of the links at next_state)
// The engine can build the contact rows of the links from dof_axis and the
// response of the bodies to their impulses from factor.

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "dynamics/jacobian.hpp"
#include "dynamics/kinematics.hpp"
#include "dynamics/mass_matrix.hpp"
#include "urdf/urdf_cache.hpp"
#include "world.hpp"
#include "world_spec.hpp"

namespace tds {

template <typename Algebra>
class WorldModel {
 public:
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using VectorX = typename Algebra::VectorX;
  using MatrixX = typename Algebra::MatrixX;
  using Matrix3X = typename Algebra::Matrix3X;
  using MultiBody = tds::MultiBody<Algebra>;
  using Transform = tds::Transform<Algebra>;
  using Pose = tds::Pose<Algebra>;
  using Geometry = tds::Geometry<Algebra>;
  using ContactPoint = tds::ContactPoint<Algebra>;

  struct Body {
    std::string name;
    MultiBody* mb{nullptr};
    bool is_ground{false};
    // name of the root link of the URDF
    std::string base_link;
    // offsets of q and qd of the body in the state, and of qd in the
    // velocity array
    int q_offset{0};
    int qd_offset{0};
    int velocity_offset{0};
    // control inputs
    std::string control_mode{"none"};
    int control_offset{0};
    std::vector<int> actuated_links;
    std::vector<double> default_kp;
    std::vector<double> default_kd;
    // maximum torque per actuated joint, < 0 for none
    std::vector<double> effort_limits;
    double armature{0};
    bool implicit_pd{true};
    std::vector<double> default_q;
    double friction{-1};
    bool collide_all{true};
    std::set<std::string> collide_with;
    // the bodies that a collision geometry collides with, if it is not the
    // bodies of the body
    std::map<const Geometry*, std::pair<bool, std::set<std::string>>>
        geometry_collides_with;
    // offsets in the poses and links arrays (in poses, 7 numbers each)
    int visual_offset{0};
    int num_visuals{0};
    int link_offset{0};
    int num_links{0};
    int index{0};
    // the factorization of the mass matrix M = L^T D L: the parent of each
    // degree of freedom in the kinematic tree (-1 for none), and in the
    // factor array, D(i) at factor_index[i] followed by L(i, j) for the
    // ancestors j = parent[i], parent[parent[i]], ...
    std::vector<int> parent;
    std::vector<int> factor_index;
    int factor_offset{0};
    int factor_size{0};
    bool in_contact{false};
    // the degrees of freedom that move each link (index link + 1, the base
    // is -1), ancestors first
    std::vector<std::vector<int>> link_dofs;

    int control_dim() const {
      const int n = static_cast<int>(actuated_links.size());
      return control_mode == "pd" ? 3 * n : control_mode == "torque" ? n : 0;
    }
  };

  // a pair of geometries that collide
  struct GeometryPair {
    int body_a, link_a, geom_a;
    int body_b, link_b, geom_b;
    int num_points;
    double friction;
  };

  struct Contact {
    int body_a, link_a, body_b, link_b;
    ContactPoint point;
    double friction;
  };

  WorldSpec spec;
  UrdfCache<Algebra> cache;
  World<Algebra> world;
  CollisionDispatcher<Algebra> dispatcher;
  std::vector<Body> bodies;
  std::vector<GeometryPair> pairs;
  // the contact problem
  int num_contacts{0};
  // warnings about the world, such as unsupported collision pairs
  std::vector<std::string> warnings;

  Scalar dt;

  explicit WorldModel(const WorldSpec& world_spec) : spec(world_spec) {
    dt = Algebra::from_double(spec.dt);
    world.set_gravity(Vector3(Algebra::from_double(spec.gravity[0]),
                              Algebra::from_double(spec.gravity[1]),
                              Algebra::from_double(spec.gravity[2])));
    if (spec.has_ground && !external_contacts()) add_ground();
    for (const auto& body_spec : spec.bodies) add_body(body_spec);
    assign_offsets();
    if (!external_contacts()) find_geometry_pairs();
    setup_factorization();
  }

  // whether another engine solves the contacts, see above
  bool external_contacts() const { return spec.solver == "external"; }

  WorldModel(const WorldModel&) = delete;
  WorldModel& operator=(const WorldModel&) = delete;

  int state_dim() const { return state_dim_; }
  int control_dim() const { return control_dim_; }
  // sum of the qd of all bodies
  int velocity_dim() const { return velocity_dim_; }
  // number of rows of the contact problem
  int mlcp_rows() const { return 3 * num_contacts; }
  // the nonzero elements of the matrix A (see a_index_), and the vector b
  int mlcp_dim() const { return a_nnz_ + mlcp_rows(); }
  int jacobian_dim() const { return mlcp_rows() * velocity_dim_; }
  // the factorizations of the mass matrices of the bodies in contact
  int factor_dim() const { return factor_dim_; }
  // the world motion vectors of the degrees of freedom, see write_dof_axes
  int dof_axis_dim() const { return 6 * velocity_dim_; }
  int num_visuals() const { return num_visuals_; }
  int poses_dim() const { return 7 * num_visuals_; }
  int num_links() const { return num_links_; }
  int links_dim() const { return 7 * num_links_; }

  const Body& body(const std::string& name) const {
    for (const auto& b : bodies) {
      if (b.name == name) return b;
    }
    throw std::runtime_error("No body \"" + name + "\".");
  }

  std::vector<double> default_state() const {
    std::vector<double> state(state_dim_, 0.);
    for (const auto& b : bodies) {
      std::copy(b.default_q.begin(), b.default_q.end(),
                state.begin() + b.q_offset);
    }
    return state;
  }

  // the PD targets are the default joint positions
  std::vector<double> default_control() const {
    std::vector<double> control(control_dim_, 0.);
    for (const auto& b : bodies) {
      if (b.control_mode != "pd") continue;
      const int n = static_cast<int>(b.actuated_links.size());
      for (int i = 0; i < n; ++i) {
        const auto& link = b.mb->links()[b.actuated_links[i]];
        control[b.control_offset + i] = b.default_q[link.q_index];
        control[b.control_offset + n + i] = b.default_kp[i];
        control[b.control_offset + 2 * n + i] = b.default_kd[i];
      }
    }
    return control;
  }

  void set_state(const Scalar* state) {
    for (auto& b : bodies) {
      if (b.is_ground) continue;
      b.mb->initialize();
      for (int i = 0; i < b.mb->dof(); ++i) {
        b.mb->q(i) = state[b.q_offset + i];
      }
      for (int i = 0; i < b.mb->dof_qd(); ++i) {
        b.mb->qd(i) = state[b.qd_offset + i];
      }
    }
  }

  void get_state(Scalar* state) const {
    for (const auto& b : bodies) {
      if (b.is_ground) continue;
      for (int i = 0; i < b.mb->dof(); ++i) {
        state[b.q_offset + i] = b.mb->q(i);
      }
      for (int i = 0; i < b.mb->dof_qd(); ++i) {
        state[b.qd_offset + i] = b.mb->qd(i);
      }
    }
  }

  // Joint torques from the control, forward dynamics and the integration of
  // the accelerations into the velocities. Leaves the link transforms at the
  // positions of the state.
  void dynamics(const Scalar* control) {
    for (auto& b : bodies) {
      if (b.is_ground) continue;
      apply_control(b, control);
      tds::forward_dynamics(*b.mb, world.get_gravity());
      b.mb->clear_forces();
      integrate_euler_qdd(*b.mb, dt);
    }
  }

  // the contact points of all geometry pairs, after dynamics()
  std::vector<Contact> compute_contacts() {
    std::vector<Contact> contacts;
    std::vector<ContactPoint> points;
    for (const auto& pair : pairs) {
      const MultiBody* mb_a = bodies[pair.body_a].mb;
      const MultiBody* mb_b = bodies[pair.body_b].mb;
      const Pose pose_a = geometry_pose(*mb_a, pair.link_a, pair.geom_a);
      const Pose pose_b = geometry_pose(*mb_b, pair.link_b, pair.geom_b);
      points.resize(0);
      const int n = dispatcher.compute_contacts(
          mb_a->collision_geometries(pair.link_a)[pair.geom_a], pose_a,
          mb_b->collision_geometries(pair.link_b)[pair.geom_b], pose_b,
          points);
      if (n != pair.num_points || static_cast<int>(points.size()) != n) {
        throw std::runtime_error(
            "WorldModel: unexpected number of contact points.");
      }
      for (const auto& p : points) {
        contacts.push_back(
            {pair.body_a, pair.link_a, pair.body_b, pair.link_b, p,
             pair.friction});
      }
    }
    return contacts;
  }

  // Sets up the contact problem after dynamics(): the matrix A (row-major)
  // and vector b in mlcp, the constraint Jacobian J (row-major, mlcp_rows x
  // velocity_dim) of the contact velocities, and the factorization of the
  // mass matrices for integrate() (see factor_dim()).
  //
  // The mass matrix M of each body is factorized as M = L^T D L, where L has
  // the sparsity of the kinematic tree (Featherstone, Rigid Body Dynamics
  // Algorithms, section 6.5), then A = J M^-1 J^T = Z^T D^-1 Z with
  // Z = L^-T J^T.
  void contact_problem(Scalar* mlcp, Scalar* jacobian, Scalar* factor,
                       Scalar* contact_distance) {
    const std::vector<Contact> contacts = compute_contacts();
    const int n_c = static_cast<int>(contacts.size());
    const int m = 3 * n_c;
    const int n = velocity_dim_;
    // constraint Jacobian rows: normal of each contact, then the friction
    // directions, J = [J_a, -J_b] of the contact point velocities
    MatrixX jac(m, n);
    Algebra::set_zero(jac);
    VectorX b(m);
    Algebra::set_zero(b);
    const Scalar erp = Algebra::from_double(spec.contact.erp);
    const Scalar restitution = Algebra::from_double(spec.contact.restitution);
    for (int i = 0; i < n_c; ++i) {
      const Contact& c = contacts[i];
      const ContactPoint& cp = c.point;
      contact_distance[i] = cp.distance;
      const Scalar collision = where_lt(cp.distance, Algebra::zero(),
                                        Algebra::one(), Algebra::zero());
      Body& body_a = bodies[c.body_a];
      Body& body_b = bodies[c.body_b];
      const int n_a = body_a.mb->dof_qd();
      const int n_b = body_b.mb->dof_qd();
      Matrix3X jac_a(3, n_a), jac_b(3, n_b);
      Vector3 vel_a = Algebra::zero3(), vel_b = Algebra::zero3();
      if (n_a > 0) {
        jac_a = point_jacobian2(*body_a.mb, c.link_a, cp.world_point_on_a,
                                false);
        vel_a = jac_a * VectorX(body_a.mb->qd());
      }
      if (n_b > 0) {
        jac_b = point_jacobian2(*body_b.mb, c.link_b, cp.world_point_on_b,
                                false);
        vel_b = jac_b * VectorX(body_b.mb->qd());
      }
      const Vector3 rel_vel = vel_a - vel_b;
      const Scalar normal_rel_vel = Algebra::dot(cp.world_normal_on_b, rel_vel);
      // Baumgarte stabilization
      const Scalar baumgarte_rel_vel = erp * cp.distance / dt;
      b[i] = (-(Algebra::one() + restitution) * normal_rel_vel -
              baumgarte_rel_vel) *
             collision;
      Vector3 fr_direction1, fr_direction2;
      MultiBodyConstraintSolver<Algebra>::plane_space(
          cp.world_normal_on_b, fr_direction1, fr_direction2);
      fr_direction1 *= collision;
      fr_direction2 *= collision;
      b[n_c + i] = -Algebra::dot(fr_direction1, rel_vel);
      b[2 * n_c + i] = -Algebra::dot(fr_direction2, rel_vel);
      const Vector3 directions[3] = {cp.world_normal_on_b * collision,
                                     fr_direction1, fr_direction2};
      for (int d = 0; d < 3; ++d) {
        const int row = d * n_c + i;
        if (n_a > 0) {
          VectorX ja = Algebra::mul_transpose(jac_a, directions[d]);
          for (int k : link_dofs(body_a, c.link_a)) {
            jac(row, body_a.velocity_offset + k) += ja[k];
          }
        }
        if (n_b > 0) {
          VectorX jb = Algebra::mul_transpose(jac_b, directions[d]);
          for (int k : link_dofs(body_b, c.link_b)) {
            jac(row, body_b.velocity_offset + k) -= jb[k];
          }
        }
      }
    }
    for (int r = 0; r < m; ++r) {
      for (int k = 0; k < n; ++k) jacobian[r * n + k] = jac(r, k);
    }
    MatrixX A(m, m);
    Algebra::set_zero(A);
    for (auto& body : bodies) {
      const int n_k = body.mb->dof_qd();
      if (n_k == 0 || !body.in_contact) continue;
      MatrixX mass(n_k, n_k);
      mass_matrix(*body.mb, &mass);
      factorize(body, mass, factor + body.factor_offset);
      // Z = L^-T J^T, of the rows of J that involve the body
      std::vector<int> rows;
      for (int r = 0; r < m; ++r) {
        const int c = r % n_c;
        if (contacts[c].body_a == body.index ||
            contacts[c].body_b == body.index) {
          rows.push_back(r);
        }
      }
      std::vector<std::vector<Scalar>> z(rows.size());
      for (size_t r = 0; r < rows.size(); ++r) {
        z[r].resize(n_k);
        for (int k = 0; k < n_k; ++k) {
          z[r][k] = jac(rows[r], body.velocity_offset + k);
        }
        solve_lt(body, factor + body.factor_offset, z[r].data());
      }
      // A += Z^T D^-1 Z
      std::vector<Scalar> inv_d(n_k);
      for (int k = 0; k < n_k; ++k) {
        inv_d[k] = Algebra::one() /
                   factor[body.factor_offset + body.factor_index[k]];
      }
      for (size_t r = 0; r < rows.size(); ++r) {
        for (size_t s = 0; s <= r; ++s) {
          Scalar sum = Algebra::zero();
          for (int k = 0; k < n_k; ++k) sum += z[r][k] * inv_d[k] * z[s][k];
          A(rows[r], rows[s]) += sum;
          if (s != r) A(rows[s], rows[r]) += sum;
        }
      }
    }
    // constraint force mixing, necessary for fixed base systems that cannot
    // move along the friction directions
    const Scalar cfm = Algebra::from_double(spec.contact.cfm);
    for (int r = 0; r < m; ++r) {
      for (int s = 0; s < m; ++s) {
        const int k = a_index_[r * m + s];
        if (k >= 0) mlcp[k] = r == s ? A(r, s) + cfm : A(r, s);
      }
      mlcp[a_nnz_ + r] = b[r];
    }
  }

  // One projected Gauss-Seidel iteration of the contact problem: the normal
  // impulses are in [0, 1e5], the friction impulses in
  // [-mu lambda_n, mu lambda_n].
  void pgs_iteration(const Scalar* mlcp, const Scalar* lambda,
                     Scalar* lambda_next) const {
    const int n_c = num_contacts;
    const int m = 3 * n_c;
    const Scalar* A = mlcp;
    const Scalar* b = mlcp + a_nnz_;
    for (int i = 0; i < m; ++i) lambda_next[i] = lambda[i];
    Scalar* x = lambda_next;
    for (int i = 0; i < m; ++i) {
      Scalar delta = Algebra::zero();
      for (int j = 0; j < m; j++) {
        const int k = a_index_[i * m + j];
        if (j != i && k >= 0) delta += A[k] * x[j];
      }
      x[i] = (b[i] - delta) / A[a_index_[i * m + i]];
      if (i < n_c) {
        x[i] = Algebra::max(x[i], Algebra::zero());
        x[i] = Algebra::min(x[i], Algebra::fraction(100000, 1));
      } else {
        Scalar s = x[i % n_c];
        s = where_lt(s, Algebra::zero(), Algebra::zero(), s);
        const Scalar mu = Algebra::from_double(contact_friction_[i % n_c]);
        x[i] = Algebra::max(x[i], -mu * s);
        x[i] = Algebra::min(x[i], mu * s);
      }
    }
  }

  // The contact setup of a time step, see the split time step above.
  void contact(const Scalar* state, const Scalar* control, Scalar* velocity,
               Scalar* mlcp, Scalar* jacobian, Scalar* factor,
               Scalar* contact_distance) {
    set_state(state);
    dynamics(control);
    for (const auto& b : bodies) {
      for (int i = 0; i < b.mb->dof_qd(); ++i) {
        velocity[b.velocity_offset + i] = b.mb->qd(i);
      }
    }
    contact_problem(mlcp, jacobian, factor, contact_distance);
  }

  // Integrates the joint velocities after the contact impulses lambda,
  // qd = velocity + M^-1 J^T lambda.
  void integrate(const Scalar* state, const Scalar* velocity,
                 const Scalar* jacobian, const Scalar* factor,
                 const Scalar* lambda, Scalar* next_state) {
    set_state(state);
    const int n = velocity_dim_;
    const int n_c = num_contacts;
    std::vector<Scalar> g(n, Algebra::zero());
    // J^T lambda, only the nonzero elements of J
    int c = 0;
    for (const auto& pair : pairs) {
      for (int p = 0; p < pair.num_points; ++p, ++c) {
        for (int d = 0; d < 3; ++d) {
          const int row = d * n_c + c;
          for (int side = 0; side < 2; ++side) {
            const Body& b = bodies[side == 0 ? pair.body_a : pair.body_b];
            const int link = side == 0 ? pair.link_a : pair.link_b;
            if (b.mb->dof_qd() == 0) continue;
            for (int k : link_dofs(b, link)) {
              const int column = b.velocity_offset + k;
              g[column] += jacobian[row * n + column] * lambda[row];
            }
          }
        }
      }
    }
    for (auto& b : bodies) {
      if (b.is_ground) continue;
      const int n_k = b.mb->dof_qd();
      Scalar* x = g.data() + b.velocity_offset;
      if (b.in_contact) solve(b, factor + b.factor_offset, x);
      for (int i = 0; i < n_k; ++i) {
        b.mb->qd(i) = velocity[b.velocity_offset + i] + x[i];
      }
      tds::integrate_euler(*b.mb, dt);
    }
    get_state(next_state);
  }

  // A whole time step, with iterations of the contact solver.
  void step(const Scalar* state, const Scalar* control, Scalar* next_state,
            int iterations = -1) {
    if (iterations < 0) iterations = spec.contact.iterations;
    const int m = mlcp_rows();
    std::vector<Scalar> velocity(velocity_dim_), mlcp(mlcp_dim()),
        jacobian(jacobian_dim()), factor(factor_dim()),
        distance(num_contacts), lambda(m), lambda_next(m);
    contact(state, control, velocity.data(), mlcp.data(), jacobian.data(),
            factor.data(), distance.data());
    for (int r = 0; r < m; ++r) lambda[r] = Algebra::zero();
    for (int it = 0; it < iterations; ++it) {
      pgs_iteration(mlcp.data(), lambda.data(), lambda_next.data());
      std::swap(lambda, lambda_next);
    }
    integrate(state, velocity.data(), jacobian.data(), factor.data(),
              lambda.data(), next_state);
  }

  // The functions of a time step with the solver "external", see above.

  // The joint velocities after the unconstrained dynamics.
  void predict(const Scalar* state, const Scalar* control, Scalar* velocity) {
    set_state(state);
    dynamics(control);
    for (const auto& b : bodies) {
      for (int i = 0; i < b.mb->dof_qd(); ++i) {
        velocity[b.velocity_offset + i] = b.mb->qd(i);
      }
    }
  }

  // The factorization of the mass matrices (see factor_dim()), with the
  // armature that the implicit PD of the control adds, and the world motion
  // vectors of the degrees of freedom (see write_dof_axes).
  void articulation(const Scalar* state, const Scalar* control, Scalar* factor,
                    Scalar* dof_axis) {
    set_state(state);
    for (auto& b : bodies) {
      const int n = b.mb->dof_qd();
      if (b.is_ground || n == 0) continue;
      apply_control(b, control);
      b.mb->clear_forces();
      tds::forward_kinematics(*b.mb, b.mb->q(), b.mb->qd());
      MatrixX mass(n, n);
      mass_matrix(*b.mb, &mass);
      factorize(b, mass, factor + b.factor_offset);
      write_dof_axes(b, dof_axis);
    }
  }

  // The positions integrated with the joint velocities, and the world poses
  // of the links at the new positions (as links()).
  void advance(const Scalar* state, const Scalar* velocity, Scalar* next_state,
               Scalar* links) {
    set_state(state);
    for (auto& b : bodies) {
      if (b.is_ground) continue;
      for (int i = 0; i < b.mb->dof_qd(); ++i) {
        b.mb->qd(i) = velocity[b.velocity_offset + i];
      }
      tds::integrate_euler(*b.mb, dt);
    }
    get_state(next_state);
    write_links(links);
  }

  // world poses (position, quaternion xyzw) of the visual shapes of all
  // bodies, for rendering
  void poses(const Scalar* state, Scalar* poses) {
    set_state(state);
    int j = 0;
    for (auto& b : bodies) {
      if (b.is_ground) continue;
      tds::forward_kinematics(*b.mb, b.mb->q(), b.mb->qd());
      for (const auto& X_visual : b.mb->X_visuals()) {
        write_pose(b.mb->base_X_world() * X_visual, poses, j);
      }
      for (const auto& link : *b.mb) {
        for (const auto& X_visual : link.X_visuals) {
          write_pose(link.X_world * X_visual, poses, j);
        }
      }
    }
  }

  // world poses of the base and the links of all bodies
  void links(const Scalar* state, Scalar* links) {
    set_state(state);
    write_links(links);
  }

  // friction coefficient of each contact
  const std::vector<double>& contact_friction() const {
    return contact_friction_;
  }

 private:
  int state_dim_{0};
  int control_dim_{0};
  int velocity_dim_{0};
  int factor_dim_{0};
  // A(r, s) of the contact problem is mlcp[a_index_[r * m + s]] if it is
  // not -1: the rows of two contacts are coupled if the contacts share a
  // moving body
  std::vector<int> a_index_;
  int a_nnz_{0};
  int num_visuals_{0};
  int num_links_{0};
  std::vector<double> contact_friction_;

  const std::vector<int>& link_dofs(const Body& b, int link) const {
    return b.link_dofs[link + 1];
  }

  // the kinematic tree of the degrees of freedom of each body, and the
  // layout of the factorizations of the mass matrices
  void setup_factorization() {
    for (const auto& pair : pairs) {
      bodies[pair.body_a].in_contact = true;
      bodies[pair.body_b].in_contact = true;
    }
    if (external_contacts()) {
      // any link can touch anything in the other engine
      for (auto& b : bodies) b.in_contact = !b.is_ground;
    }
    factor_dim_ = 0;
    for (auto& b : bodies) {
      const int n = b.mb->dof_qd();
      const int num_links = static_cast<int>(b.mb->num_links());
      const int base = b.mb->is_floating() ? 6 : 0;
      b.parent.assign(n, -1);
      for (int i = 1; i < base; ++i) b.parent[i] = i - 1;
      // the last degree of freedom that moves each link
      std::vector<int> last_dof(num_links, -1);
      for (const auto& link : *b.mb) {
        int dof = link.parent_index >= 0 ? last_dof[link.parent_index]
                                         : base - 1;
        if (link.joint_type != JOINT_FIXED) {
          const int count = link.joint_type == JOINT_SPHERICAL ? 3 : 1;
          for (int d = 0; d < count; ++d) {
            if (link.qd_index + d <= dof) {
              throw std::runtime_error(
                  "WorldModel: the links are not in topological order.");
            }
            b.parent[link.qd_index + d] = dof;
            dof = link.qd_index + d;
          }
        }
        last_dof[link.index] = dof;
      }
      auto chain = [&](int dof) {
        std::vector<int> dofs;
        for (int d = dof; d >= 0; d = b.parent[d]) dofs.push_back(d);
        std::reverse(dofs.begin(), dofs.end());
        return dofs;
      };
      b.link_dofs.assign(num_links + 1, {});
      b.link_dofs[0] = chain(base - 1);
      for (int l = 0; l < num_links; ++l) b.link_dofs[l + 1] = chain(last_dof[l]);
      b.factor_index.assign(n, 0);
      b.factor_size = 0;
      for (int i = 0; i < n; ++i) {
        b.factor_index[i] = b.factor_size;
        b.factor_size += static_cast<int>(chain(i).size());
      }
      b.factor_offset = factor_dim_;
      if (b.in_contact) factor_dim_ += b.factor_size;
    }
    // the sparsity of the matrix A
    std::vector<std::pair<int, int>> contact_bodies;
    for (const auto& pair : pairs) {
      for (int p = 0; p < pair.num_points; ++p) {
        contact_bodies.push_back({pair.body_a, pair.body_b});
      }
    }
    const int n_c = num_contacts;
    const int m = 3 * n_c;
    auto moves = [&](int body) { return bodies[body].mb->dof_qd() > 0; };
    auto coupled = [&](int c, int d) {
      for (int x : {contact_bodies[c].first, contact_bodies[c].second}) {
        if (!moves(x)) continue;
        if (x == contact_bodies[d].first || x == contact_bodies[d].second) {
          return true;
        }
      }
      return false;
    };
    a_index_.assign(m * m, -1);
    a_nnz_ = 0;
    for (int r = 0; r < m; ++r) {
      for (int s = 0; s < m; ++s) {
        if (r == s || coupled(r % n_c, s % n_c)) a_index_[r * m + s] = a_nnz_++;
      }
    }
  }

  // Factorizes the mass matrix H of the body as H = L^T D L, where L is
  // lower triangular with ones on the diagonal and has the sparsity of the
  // kinematic tree (Featherstone, Rigid Body Dynamics Algorithms, table 6.3),
  // and writes D and L to factor, see Body::factor_index.
  static void factorize(const Body& b, MatrixX H, Scalar* factor) {
    const int n = b.mb->dof_qd();
    for (int k = n - 1; k >= 0; --k) {
      for (int i = b.parent[k]; i >= 0; i = b.parent[i]) {
        const Scalar a = H(k, i) / H(k, k);
        for (int j = i; j >= 0; j = b.parent[j]) H(i, j) -= a * H(k, j);
        H(k, i) = a;
      }
    }
    for (int i = 0; i < n; ++i) {
      int o = b.factor_index[i];
      factor[o++] = H(i, i);
      for (int j = b.parent[i]; j >= 0; j = b.parent[j]) factor[o++] = H(i, j);
    }
  }

  // solves L^T x = y in place
  static void solve_lt(const Body& b, const Scalar* factor, Scalar* x) {
    const int n = b.mb->dof_qd();
    for (int i = n - 1; i >= 0; --i) {
      int o = b.factor_index[i] + 1;
      for (int j = b.parent[i]; j >= 0; j = b.parent[j]) {
        x[j] -= factor[o++] * x[i];
      }
    }
  }

  // solves H x = y in place, with H = L^T D L
  static void solve(const Body& b, const Scalar* factor, Scalar* x) {
    const int n = b.mb->dof_qd();
    solve_lt(b, factor, x);
    for (int i = 0; i < n; ++i) x[i] = x[i] / factor[b.factor_index[i]];
    for (int i = 0; i < n; ++i) {
      int o = b.factor_index[i] + 1;
      for (int j = b.parent[i]; j >= 0; j = b.parent[j]) {
        x[i] -= factor[o++] * x[j];
      }
    }
  }

  // the world poses of the bases and links at the positions of the bodies
  void write_links(Scalar* links) {
    int j = 0;
    for (auto& b : bodies) {
      if (b.is_ground) continue;
      tds::forward_kinematics(*b.mb, b.mb->q(), b.mb->qd());
      write_pose(b.mb->base_X_world(), links, j);
      for (const auto& link : *b.mb) write_pose(link.X_world, links, j);
    }
  }

  // Writes the world motion vector of each degree of freedom of the body,
  // after forward kinematics: 6 numbers at dof_axis[6 * (velocity_offset +
  // i)], the angular velocity w and the linear velocity v of the point at the
  // world origin for qd(i) = 1, so that a point p of a link moves with the
  // sum of (v + w x p) qd(i) over the degrees of freedom of the link
  // (link_dofs). The floating base velocity is the angular velocity and the
  // velocity of the base origin x, in world coordinates: (e_k, x x e_k) and
  // (0, e_k).
  void write_dof_axes(const Body& b, Scalar* dof_axis) const {
    MultiBody& mb = *b.mb;
    Scalar* out = dof_axis + 6 * b.velocity_offset;
    auto write = [&](int dof, const Vector3& w, const Vector3& v) {
      for (int k = 0; k < 3; ++k) {
        out[6 * dof + k] = w[k];
        out[6 * dof + 3 + k] = v[k];
      }
    };
    if (mb.is_floating()) {
      const Vector3 x = mb.base_X_world().translation;
      for (int k = 0; k < 3; ++k) {
        Vector3 e = Algebra::zero3();
        e[k] = Algebra::one();
        write(k, e, Algebra::cross(x, e));
        write(3 + k, Algebra::zero3(), e);
      }
    }
    for (const auto& link : mb) {
      if (link.joint_type == JOINT_FIXED) continue;
      if (link.joint_type == JOINT_SPHERICAL) {
        throw std::runtime_error(
            "WorldModel: spherical joints are not supported with the solver "
            "\"external\".");
      }
      // the motion subspace in world coordinates at the world origin
      const tds::MotionVector<Algebra> s = link.X_world.apply_inverse(link.S);
      write(link.qd_index, s.top, s.bottom);
    }
  }

  static void write_pose(const Transform& tr, Scalar* out, int& j) {
    const auto orn = Algebra::matrix_to_quat(tr.rotation);
    out[j++] = tr.translation[0];
    out[j++] = tr.translation[1];
    out[j++] = tr.translation[2];
    out[j++] = Algebra::quat_x(orn);
    out[j++] = Algebra::quat_y(orn);
    out[j++] = Algebra::quat_z(orn);
    out[j++] = Algebra::quat_w(orn);
  }

  static Pose geometry_pose(const MultiBody& mb, int link, int geom) {
    const Transform tr =
        mb.get_world_transform(link) * mb.collision_transforms(link)[geom];
    Pose pose;
    pose.position_ = tr.translation;
    pose.orientation_ = Algebra::normalize(Algebra::matrix_to_quat(tr.rotation));
    return pose;
  }

  void add_ground() {
    Body ground;
    ground.name = "ground";
    ground.is_ground = true;
    ground.mb = world.create_multi_body("ground");
    ground.mb->set_floating_base(false);
    ground.mb->initialize();
    auto* plane = new Plane<Algebra>(
        spec.up_axis == 1 ? Algebra::unit3_y() : Algebra::unit3_z(),
        Algebra::from_double(spec.ground_height));
    owned_geometries_.emplace_back(plane);
    ground.mb->collision_geometries(-1).push_back(plane);
    Transform identity;
    identity.set_identity();
    ground.mb->collision_transforms(-1).push_back(identity);
    ground.friction = spec.ground_friction;
    ground.index = static_cast<int>(bodies.size());
    ground.link_dofs.resize(1);
    bodies.push_back(ground);
  }

  static bool name_matches(const std::string& pattern,
                           const std::string& name) {
    if (pattern == name) return true;
    try {
      return std::regex_match(name, std::regex(pattern));
    } catch (const std::regex_error&) {
      return false;
    }
  }

  void add_body(const WorldSpec::Body& s) {
    for (const auto& b : bodies) {
      if (b.name == s.name) {
        throw std::runtime_error("Two bodies are named \"" + s.name + "\".");
      }
    }
    Body body;
    body.name = s.name;
    body.mb = cache.construct(s.urdf, world, false, s.floating, s.name);
    const auto& urdf = cache.retrieve(s.urdf);
    if (!urdf.base_links.empty()) body.base_link = urdf.base_links[0].link_name;
    MultiBody& mb = *body.mb;

    // initial state
    body.default_q.assign(mb.dof(), 0.);
    if (s.floating) {
      for (int i = 0; i < 4; ++i) body.default_q[i] = s.orientation[i];
      for (int i = 0; i < 3; ++i) body.default_q[4 + i] = s.position[i];
    } else {
      mb.base_X_world().translation =
          Vector3(Algebra::from_double(s.position[0]),
                  Algebra::from_double(s.position[1]),
                  Algebra::from_double(s.position[2]));
      mb.base_X_world().rotation = Algebra::quat_to_matrix(
          Algebra::from_double(s.orientation[0]),
          Algebra::from_double(s.orientation[1]),
          Algebra::from_double(s.orientation[2]),
          Algebra::from_double(s.orientation[3]));
    }
    std::set<std::string> joint_names;
    for (const auto& link : mb) {
      if (link.joint_type == JOINT_FIXED) continue;
      joint_names.insert(link.joint_name);
      auto it = s.joint_positions.find(link.joint_name);
      if (it != s.joint_positions.end()) {
        if (link.joint_type == JOINT_SPHERICAL) {
          throw std::runtime_error("Body \"" + s.name +
                                   "\": cannot set the position of the "
                                   "spherical joint " + link.joint_name);
        }
        body.default_q[link.q_index] = it->second;
      } else if (link.joint_type == JOINT_SPHERICAL) {
        body.default_q[link.q_index + 3] = 1.;
      }
    }
    for (const auto& [name, value] : s.joint_positions) {
      if (!joint_names.count(name)) {
        throw std::runtime_error("Body \"" + s.name + "\" has no joint \"" +
                                 name + "\".");
      }
    }

    // control
    const auto& c = s.control;
    body.control_mode = c.mode;
    body.armature = c.armature;
    body.implicit_pd = c.implicit;
    if (c.mode != "none") {
      for (const auto& link : mb) {
        if (link.joint_type == JOINT_FIXED) continue;
        if (link.joint_type == JOINT_SPHERICAL) {
          throw std::runtime_error("Body \"" + s.name +
                                   "\": spherical joints cannot be actuated.");
        }
        body.actuated_links.push_back(link.index);
        double kp = c.kp, kd = c.kd;
        for (const auto& [pattern, gains] : c.gains) {
          if (name_matches(pattern, link.joint_name)) {
            kp = gains.first;
            kd = gains.second;
            break;
          }
        }
        body.default_kp.push_back(kp);
        body.default_kd.push_back(kd);
        auto it = c.effort_limits.find(link.joint_name);
        body.effort_limits.push_back(it != c.effort_limits.end() ? it->second
                                                                 : -1.);
      }
    }

    // collision geometry
    const auto& col = s.collision;
    auto keep_link = [&](const std::string& link_name) {
      if (col.mode == "all") return true;
      if (col.mode == "none") return false;
      for (const auto& pattern : col.links) {
        if (name_matches(pattern, link_name)) return true;
      }
      return false;
    };
    if (!keep_link(body.base_link)) {
      mb.collision_geometries(-1).clear();
      mb.collision_transforms(-1).clear();
    }
    for (auto& link : mb) {
      if (!keep_link(link.link_name)) {
        link.collision_geometries.clear();
        link.X_collisions.clear();
      }
    }
    for (const auto& sphere : col.spheres) {
      int link_index = -2;
      if (sphere.link.empty() || sphere.link == body.base_link) {
        link_index = -1;
      }
      for (const auto& link : mb) {
        if (link.link_name == sphere.link) link_index = link.index;
      }
      if (link_index == -2) {
        throw std::runtime_error("Body \"" + s.name + "\" has no link \"" +
                                 sphere.link + "\" for a collision sphere.");
      }
      auto* geom = world.create_sphere(Algebra::from_double(sphere.radius));
      Transform tr;
      tr.set_identity();
      tr.translation = Vector3(Algebra::from_double(sphere.position[0]),
                               Algebra::from_double(sphere.position[1]),
                               Algebra::from_double(sphere.position[2]));
      mb.collision_geometries(link_index).push_back(geom);
      mb.collision_transforms(link_index).push_back(tr);
      if (sphere.has_with) {
        body.geometry_collides_with[geom] = {
            sphere.with_all,
            std::set<std::string>(sphere.with.begin(), sphere.with.end())};
      }
    }
    body.collide_all = col.with_all;
    body.collide_with.insert(col.with.begin(), col.with.end());
    body.friction = s.friction;
    body.index = static_cast<int>(bodies.size());
    bodies.push_back(body);
  }

  void assign_offsets() {
    int q = 0;
    for (auto& b : bodies) {
      b.q_offset = q;
      q += b.mb->dof();
    }
    int qd = q;
    int velocity = 0;
    for (auto& b : bodies) {
      b.qd_offset = qd;
      b.velocity_offset = velocity;
      qd += b.mb->dof_qd();
      velocity += b.mb->dof_qd();
    }
    state_dim_ = qd;
    velocity_dim_ = velocity;
    int control = 0;
    int visuals = 0;
    int links = 0;
    for (auto& b : bodies) {
      b.control_offset = control;
      control += b.control_dim();
      if (b.is_ground) continue;
      b.visual_offset = visuals;
      b.num_visuals = static_cast<int>(b.mb->X_visuals().size());
      for (const auto& link : *b.mb) {
        b.num_visuals += static_cast<int>(link.X_visuals.size());
      }
      visuals += b.num_visuals;
      b.link_offset = links;
      b.num_links = 1 + static_cast<int>(b.mb->num_links());
      links += b.num_links;
    }
    control_dim_ = control;
    num_visuals_ = visuals;
    num_links_ = links;
  }

  static int num_contact_points(int type_a, int type_b) {
    auto is = [&](int a, int b) {
      return (type_a == a && type_b == b) || (type_a == b && type_b == a);
    };
    if (is(TINY_SPHERE_TYPE, TINY_SPHERE_TYPE)) return 1;
    if (is(TINY_PLANE_TYPE, TINY_SPHERE_TYPE)) return 1;
    if (is(TINY_PLANE_TYPE, TINY_CAPSULE_TYPE)) return 2;
    if (is(TINY_PLANE_TYPE, TINY_BOX_TYPE)) return 8;
    if (is(TINY_SPHERE_TYPE, TINY_BOX_TYPE)) return 1;
    if (is(TINY_CAPSULE_TYPE, TINY_SPHERE_TYPE)) return 2;
    return 0;
  }

  static const char* geometry_name(int type) {
    switch (type) {
      case TINY_SPHERE_TYPE:
        return "sphere";
      case TINY_PLANE_TYPE:
        return "plane";
      case TINY_CAPSULE_TYPE:
        return "capsule";
      case TINY_MESH_TYPE:
        return "mesh";
      case TINY_BOX_TYPE:
        return "box";
      case TINY_CYLINDER_TYPE:
        return "cylinder";
    }
    return "unknown";
  }

  std::string link_name(const Body& b, int link) const {
    return link < 0 ? b.base_link : b.mb->links()[link].link_name;
  }

  // whether the geometry of body x collides with body y
  static bool allows(const Body& x, const Geometry* geometry, const Body& y) {
    auto it = x.geometry_collides_with.find(geometry);
    if (it != x.geometry_collides_with.end()) {
      return it->second.first || it->second.second.count(y.name) > 0;
    }
    return x.collide_all || x.collide_with.count(y.name) > 0;
  }

  void find_geometry_pairs() {
    std::map<std::string, int> unsupported;
    for (size_t i = 0; i < bodies.size(); ++i) {
      for (size_t j = i + 1; j < bodies.size(); ++j) {
        const Body& a = bodies[i];
        const Body& b = bodies[j];
        double friction = spec.contact.friction;
        if (a.friction >= 0 || b.friction >= 0) {
          const double fa = a.friction >= 0 ? a.friction : friction;
          const double fb = b.friction >= 0 ? b.friction : friction;
          friction = std::sqrt(fa * fb);
        }
        for (int la = -1; la < static_cast<int>(a.mb->num_links()); ++la) {
          const auto& geoms_a = a.mb->collision_geometries(la);
          for (int ga = 0; ga < static_cast<int>(geoms_a.size()); ++ga) {
            for (int lb = -1; lb < static_cast<int>(b.mb->num_links());
                 ++lb) {
              const auto& geoms_b = b.mb->collision_geometries(lb);
              for (int gb = 0; gb < static_cast<int>(geoms_b.size()); ++gb) {
                if (!allows(a, geoms_a[ga], b) || !allows(b, geoms_b[gb], a)) {
                  continue;
                }
                const int ta = geoms_a[ga]->get_type();
                const int tb = geoms_b[gb]->get_type();
                const int n = num_contact_points(ta, tb);
                if (n == 0) {
                  std::string key = std::string(geometry_name(ta)) + "-" +
                                    geometry_name(tb) + " (" + a.name + "/" +
                                    link_name(a, la) + ", " + b.name + "/" +
                                    link_name(b, lb) + ")";
                  ++unsupported[key];
                  continue;
                }
                pairs.push_back({static_cast<int>(i), la, ga,
                                 static_cast<int>(j), lb, gb, n, friction});
                num_contacts += n;
                for (int k = 0; k < n; ++k) {
                  contact_friction_.push_back(friction);
                }
              }
            }
          }
        }
      }
    }
    for (const auto& [key, count] : unsupported) {
      warnings.push_back("ignored unsupported collision pair " + key +
                         (count > 1 ? " x" + std::to_string(count) : ""));
    }
  }

  void apply_control(Body& b, const Scalar* control) {
    MultiBody& mb = *b.mb;
    const int n = static_cast<int>(b.actuated_links.size());
    const int tau_offset = mb.is_floating() ? 6 : 0;
    for (int i = 0; i < n; ++i) {
      auto& link = mb[b.actuated_links[i]];
      const Scalar q = mb.q(link.q_index);
      const Scalar qd = mb.qd(link.qd_index);
      Scalar force = Algebra::zero();
      link.armature = Algebra::from_double(b.armature);
      if (b.control_mode == "pd") {
        const Scalar q_target = control[b.control_offset + i];
        const Scalar kp = control[b.control_offset + n + i];
        const Scalar kd = control[b.control_offset + 2 * n + i];
        // Implicit PD: the torque uses the joint state at the end of the
        // time step, qd' = qd + dt qdd and q' = q + dt qd':
        //   tau = kp (q_target - q') - kd qd'
        //       = kp (q_target - q - dt qd) - kd qd - (dt kd + dt^2 kp) qdd
        // where the last term acts as additional joint inertia (armature).
        Scalar extra_armature = Algebra::zero();
        if (b.implicit_pd) {
          force = kp * (q_target - q - dt * qd) - kd * qd;
          extra_armature = dt * kd + dt * dt * kp;
        } else {
          force = kp * (q_target - q) - kd * qd;
        }
        if (b.effort_limits[i] >= 0) {
          // a saturated joint has a constant torque, without the implicit
          // PD inertia
          const Scalar limit = Algebra::from_double(b.effort_limits[i]);
          extra_armature = where_gt(force, limit, Algebra::zero(),
                                    where_lt(force, -limit, Algebra::zero(),
                                             extra_armature));
          force = where_gt(force, limit, limit,
                           where_lt(force, -limit, -limit, force));
        }
        link.armature += extra_armature;
      } else if (b.control_mode == "torque") {
        force = control[b.control_offset + i];
        if (b.effort_limits[i] >= 0) {
          const Scalar limit = Algebra::from_double(b.effort_limits[i]);
          force = where_gt(force, limit, limit,
                           where_lt(force, -limit, -limit, force));
        }
      }
      mb.tau_[link.qd_index - tau_offset] = force;
    }
  }

  std::vector<std::unique_ptr<Geometry>> owned_geometries_;
};

}  // namespace tds
