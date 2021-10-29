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

#pragma once

#include <string>
#include <vector>

#include "geometry.hpp"
#include "mb_constraint_solver.hpp"
#include "multi_body.hpp"
#include "rb_constraint_solver.hpp"
#include "rigid_body.hpp"

namespace tds {
template <typename Algebra>
class World {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Matrix3 = typename Algebra::Matrix3;
  typedef tds::RigidBody<Algebra> RigidBody;
  typedef tds::MultiBody<Algebra> MultiBody;
  typedef tds::Geometry<Algebra> Geometry;
  typedef tds::Transform<Algebra> Transform;
  typedef tds::RigidBodyContactPoint<Algebra> RigidBodyContactPoint;
  typedef tds::MultiBodyContactPoint<Algebra> MultiBodyContactPoint;
  typedef tds::Pose<Algebra> Pose;

  typedef tds::Capsule<Algebra> Capsule;
  typedef tds::Sphere<Algebra> Sphere;
  typedef tds::Plane<Algebra> Plane;
  typedef tds::Box<Algebra> Box;

  std::vector<RigidBody*> rigid_bodies_;
  std::vector<MultiBody*> multi_bodies_;

  Vector3 gravity_acceleration_;

  std::vector<Geometry*> geoms_;

  

  CollisionDispatcher<Algebra> dispatcher_;
  RigidBodyConstraintSolver<Algebra>* rb_constraint_solver_{nullptr};
  MultiBodyConstraintSolver<Algebra>* mb_constraint_solver_{nullptr};

 public:
  SubmitProfileTiming profile_timing_func_{nullptr};
  std::vector<RigidBodyContactPoint> rb_contacts_;
  std::vector<std::vector<MultiBodyContactPoint>> mb_contacts_;

  int num_solver_iterations{1};

  // default contact settings
  Scalar default_friction{Algebra::fraction(5, 10)};
  Scalar default_restitution{Algebra::zero()};

  explicit World(Scalar gravity_z = Algebra::fraction(-981, 100))
      : gravity_acceleration_(Algebra::zero(), Algebra::zero(), gravity_z),
        rb_constraint_solver_(new RigidBodyConstraintSolver<Algebra>),
        mb_constraint_solver_(new MultiBodyConstraintSolver<Algebra>) {}

  virtual ~World() { clear(); }

  size_t num_rigid_bodies() const { return rigid_bodies_.size(); }
  size_t num_multi_bodies() const { return multi_bodies_.size(); }
  size_t num_geoms() const { return geoms_.size(); }

  inline void submit_profile_timing(const char* name=0) const {
    if (profile_timing_func_) {
      profile_timing_func_(name);
    }
  }

  void set_mb_constraint_solver(MultiBodyConstraintSolver<Algebra>* solver) {
    if (mb_constraint_solver_) delete mb_constraint_solver_;
    mb_constraint_solver_ = solver;
  }

  MultiBodyConstraintSolver<Algebra>* get_mb_constraint_solver() {
    return mb_constraint_solver_;
  }
  
  RigidBodyConstraintSolver<Algebra>* get_rb_constraint_solver() {
      return rb_constraint_solver_;
  }

  void clear() {
    while (!geoms_.empty()) {
      delete geoms_.back(), geoms_.pop_back();
    }
    while (!rigid_bodies_.empty()) {
      delete rigid_bodies_.back(), rigid_bodies_.pop_back();
    }
    while (!multi_bodies_.empty()) {
      delete multi_bodies_.back(), multi_bodies_.pop_back();
    }

    delete rb_constraint_solver_;
    rb_constraint_solver_ = nullptr;

    delete mb_constraint_solver_;
    mb_constraint_solver_ = nullptr;
  }

  const Vector3& get_gravity() const { return gravity_acceleration_; }

  void set_gravity(const Vector3& gravity) { gravity_acceleration_ = gravity; }

  Capsule* create_capsule(const Scalar& radius, const Scalar& length) {
    Capsule* capsule = new Capsule(radius, length);
    geoms_.push_back(capsule);
    return capsule;
  }

  Plane* create_plane() {
    Plane* plane = new Plane();
    geoms_.push_back(plane);
    return plane;
  }

  Sphere* create_sphere(const Scalar& radius) {
    Sphere* sphere = new Sphere(radius);
    geoms_.push_back(sphere);
    return sphere;
  }
  
   Box* create_box (const Vector3& extents) {
      Box* box = new Box(extents);
      geoms_.push_back(box);
      return box;
  }

  CollisionDispatcher<Algebra> get_collision_dispatcher() {
    return dispatcher_;
  }

  RigidBody* create_rigid_body(const Scalar& mass, const Geometry* geom) {
    RigidBody* body = new RigidBody(mass, geom);
    rigid_bodies_.push_back(body);
    return body;
  }

  MultiBody* create_multi_body(const std::string& name = "") {
    MultiBody* body = new MultiBody();
    body->name() = name;
    multi_bodies_.push_back(body);
    return body;
  }

  std::vector<ContactPoint<Algebra>> contacts;

  static void compute_contacts_rigid_body_internal(
      std::vector<RigidBody*> bodies, CollisionDispatcher<Algebra>* dispatcher,
      std::vector<RigidBodyContactPoint>& contactsOut,
      const Scalar& restitution, const Scalar& friction) {
    std::vector<ContactPoint<Algebra>> contacts;
    {
      for (std::size_t i = 0; i < bodies.size(); i++) {
        for (std::size_t j = i + 1; j < bodies.size(); j++) {
          contacts.reserve(1);
          contacts.resize(0);

          int numContacts = dispatcher->compute_contacts(
              bodies[i]->geometry(), bodies[i]->world_pose(),
              bodies[j]->geometry(), bodies[j]->world_pose(), contacts);
          for (int c = 0; c < numContacts; c++) {
            RigidBodyContactPoint rb_pt;
            ContactPoint<Algebra>& pt = rb_pt;
            pt = contacts[c];
            rb_pt.rigid_body_a = bodies[i];
            rb_pt.rigid_body_b = bodies[j];
            // TODO(erwincoumans): combine friction and restitution based on
            // material properties of the two touching bodies
            rb_pt.restitution = restitution;
            rb_pt.friction = friction;
            contactsOut.push_back(rb_pt);
          }
        }
      }
    }
  }

  std::vector<RigidBodyContactPoint> compute_contacts_rigid_body(
      std::vector<RigidBody*> bodies,
      CollisionDispatcher<Algebra>* dispatcher) {
    std::vector<RigidBodyContactPoint> contactsOut;
    compute_contacts_rigid_body_internal(bodies, dispatcher, contactsOut,
                                         default_restitution, default_friction);
    return contactsOut;
  }

  static void compute_contacts_multi_body_internal(
      std::vector<MultiBody*> multi_bodies_,
      CollisionDispatcher<Algebra>* dispatcher,
      std::vector<std::vector<MultiBodyContactPoint>>& contacts_out,
      const Scalar& restitution, const Scalar& friction) {
    int num_multi_bodies_ = multi_bodies_.size();
    for (int i = 0; i < num_multi_bodies_; i++) {
      MultiBody* mb_a = multi_bodies_[i];
      int num_links_a = mb_a->links().size();
      for (int j = i + 1; j < multi_bodies_.size(); j++) {
        std::vector<ContactPoint<Algebra>> contacts;

        MultiBody* mb_b = multi_bodies_[j];
        int num_links_b = mb_b->links().size();
        std::vector<MultiBodyContactPoint> contacts_ab;
        for (int ii = -1; ii < num_links_a; ii++) {
          const Transform& world_transform_a = mb_a->get_world_transform(ii);

          int num_geoms__a = mb_a->collision_geometries(ii).size();
          for (int iii = 0; iii < num_geoms__a; iii++) {
            const Geometry* geom_a = mb_a->collision_geometries(ii)[iii];
            Pose pose_a;
            const Transform& local_a = mb_a->collision_transforms(ii)[iii];
            Transform tr_a = world_transform_a * local_a;
            pose_a.position_ = tr_a.translation;
            pose_a.orientation_ =
                Algebra::normalize(Algebra::matrix_to_quat(tr_a.rotation));

            for (int jj = -1; jj < num_links_b; jj++) {
              const Transform& world_transform_b =
                  mb_b->get_world_transform(jj);
              int num_geoms__b = mb_b->collision_geometries(jj).size();
              for (int jjj = 0; jjj < num_geoms__b; jjj++) {
                const Geometry* geom_b = mb_b->collision_geometries(jj)[jjj];
                Pose pose_b;
                const Transform& local_b = mb_b->collision_transforms(jj)[jjj];
                Transform tr_b = world_transform_b * local_b;
                pose_b.position_ = tr_b.translation;
                pose_b.orientation_ =
                    Algebra::normalize(Algebra::matrix_to_quat(tr_b.rotation));

                // printf("\tworld_transform_b: %.3f  %.3f  %.3f\n",
                // world_transform_b.translation[0],
                // world_transform_b.translation[1],
                // world_transform_b.translation[2]);
                //                printf("\tpose_b: %.3f  %.3f  %.3f\n",
                //                pose_b.position[0], pose_b.position[1],
                //                pose_b.position[2]);
                contacts.reserve(1);
                contacts.resize(0);
                int numContacts = dispatcher->compute_contacts(
                    geom_a, pose_a, geom_b, pose_b, contacts);
                for (int c = 0; c < numContacts; c++) {
                  MultiBodyContactPoint mb_pt;
                  ContactPoint<Algebra>& pt = mb_pt;
                  pt = contacts[c];
                  mb_pt.multi_body_a = multi_bodies_[i];
                  mb_pt.multi_body_b = multi_bodies_[j];
                  mb_pt.link_a = ii;
                  mb_pt.link_b = jj;
                  // TODO(erwincoumans): combine friction and restitution
                  // based on material properties of the two touching bodies
                  mb_pt.restitution = restitution;
                  mb_pt.friction = friction;
                  contacts_ab.push_back(mb_pt);
                }
              }
            }
            //            printf("\n");
            //            fflush(stdout);
          }
        }

        contacts_out.push_back(contacts_ab);
      }
    }
  }

  std::vector<std::vector<MultiBodyContactPoint>> compute_contacts_multi_body(
      std::vector<MultiBody*> bodies,
      CollisionDispatcher<Algebra>* dispatcher) {
    std::vector<std::vector<MultiBodyContactPoint>> contactsOut;
    compute_contacts_multi_body_internal(bodies, dispatcher, contactsOut,
                                         default_restitution, default_friction);
    return contactsOut;
  }

  void step(const Scalar& dt) {
    {
      rb_contacts_.reserve(1024);

      rb_contacts_.resize(0);

      mb_contacts_.reserve(1024);
      mb_contacts_.resize(0);
      submit_profile_timing("apply forces");
      for (std::size_t i = 0; i < rigid_bodies_.size(); i++) {
        RigidBody* b = rigid_bodies_[i];

        b->apply_gravity(gravity_acceleration_);

        b->apply_force_impulse(dt);

        b->clear_forces();
      }
      submit_profile_timing();
    }

    {
      submit_profile_timing("compute contacts");

      compute_contacts_rigid_body_internal(rigid_bodies_, &dispatcher_,
                                           rb_contacts_, default_restitution,
                                           default_friction);
      submit_profile_timing();
    }

    {
      submit_profile_timing("compute multi body contacts");
      compute_contacts_multi_body_internal(multi_bodies_, &dispatcher_,
                                           mb_contacts_, default_restitution,
                                           default_friction);
      submit_profile_timing();
      // mb_contacts_.insert(mb_contacts_.end(),
      //                     additional_MultiBodyContacts.begin(),
      //                     additional_MultiBodyContacts.end());
    }

    {
      submit_profile_timing("solve constraints");
      for (int i = 0; i < num_solver_iterations; i++) {
        for (std::size_t c = 0; c < rb_contacts_.size(); c++) {
          rb_constraint_solver_->resolve_collision(rb_contacts_[c], dt);
        }
      }
      // use outer loop in case the multi-body constraint solver requires it
      // (e.g. sequential impulse method)
      int mb_solver_iters;
      if (!mb_constraint_solver_->needs_outer_iterations()) {
        mb_solver_iters = 1;
      } else {
        mb_solver_iters = num_solver_iterations;
      }
      // std::cout << "Resolving " << mb_contacts_.size()
      //           << " contacts.\n";
      for (int i = 0; i < mb_solver_iters; i++) {
        for (std::size_t c = 0; c < mb_contacts_.size(); c++) {
          mb_constraint_solver_->resolve_collision(mb_contacts_[c], dt);
        }
      }
      submit_profile_timing();
    }

    {
      submit_profile_timing("integrate");
      for (RigidBody* rb : rigid_bodies_) {
        rb->integrate(dt);
      }
      submit_profile_timing();
    }
  }
};
}  // namespace tds
