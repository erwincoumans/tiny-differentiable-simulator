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

#ifndef TINY_WORLD_H
#define TINY_WORLD_H

#include <string>
#include <vector>

#include "tiny_constraint_solver.h"
#include "tiny_geometry.h"
#include "tiny_mb_constraint_solver.h"
#include "tiny_multi_body.h"
#include "tiny_rigid_body.h"

template <typename TinyScalar, typename TinyConstants>
class TinyWorld {
  typedef ::TinyRigidBody<TinyScalar, TinyConstants> TinyRigidBody;
  typedef ::TinyMultiBody<TinyScalar, TinyConstants> TinyMultiBody;
  typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
  typedef ::TinyGeometry<TinyScalar, TinyConstants> TinyGeometry;
  typedef ::TinySpatialTransform<TinyScalar, TinyConstants>
      TinySpatialTransform;

  typedef ::TinyCapsule<TinyScalar, TinyConstants> TinyCapsule;
  typedef ::TinySphere<TinyScalar, TinyConstants> TinySphere;
  typedef ::TinyPlane<TinyScalar, TinyConstants> TinyPlane;
  std::vector<TinyRigidBody*> m_bodies;

  std::vector<TinyMultiBody*> m_multi_bodies;

  TinyVector3 m_gravity_acceleration;

  std::vector<TinyGeometry*> m_geoms;

  TinyCollisionDispatcher<TinyScalar, TinyConstants> m_dispatcher;

 public:
  TinySubmitProfileTiming m_profileTimingFunc{nullptr};
  TinyConstraintSolver<TinyScalar, TinyConstants>* m_constraint_solver{nullptr};
  TinyMultiBodyConstraintSolver<TinyScalar, TinyConstants>*
      m_mb_constraint_solver{nullptr};

  int m_num_solver_iterations{50};

  // contact settings
  TinyScalar default_friction{TinyConstants::fraction(2, 10)};
  TinyScalar default_restitution{TinyConstants::zero()};

  explicit TinyWorld(TinyScalar gravity_z = TinyConstants::fraction(-98, 10))
      : m_gravity_acceleration(TinyConstants::zero(), TinyConstants::zero(),
                               gravity_z),
        m_constraint_solver(
            new TinyConstraintSolver<TinyScalar, TinyConstants>),
        m_mb_constraint_solver(
            new TinyMultiBodyConstraintSolver<TinyScalar, TinyConstants>) {}

  inline void submitProfileTiming(const std::string& name) {
    if (m_profileTimingFunc) {
      m_profileTimingFunc(name);
    }
  }
  virtual ~TinyWorld() { clear(); }

  void clear() {
    for (int i = 0; i < m_geoms.size(); i++) {
      delete m_geoms[i];
    }
    m_geoms.clear();

    for (int i = 0; i < m_bodies.size(); i++) {
      delete m_bodies[i];
    }
    m_bodies.clear();

    for (int i = 0; i < m_multi_bodies.size(); i++) {
      delete m_multi_bodies[i];
    }
    m_multi_bodies.clear();

    if (m_constraint_solver) {
      delete m_constraint_solver;
      m_constraint_solver = nullptr;
    }
  }

  const TinyVector3& get_gravity() const { return m_gravity_acceleration; }

  void set_gravity(const TinyVector3& gravity) {
    m_gravity_acceleration = gravity;
  }

  TinyConstraintSolver<TinyScalar, TinyConstants>* get_constraint_solver() {
    return m_constraint_solver;
  }

  TinyCapsule* create_capsule(TinyScalar radius, TinyScalar length) {
    TinyCapsule* capsule = new TinyCapsule(radius, length);
    m_geoms.push_back(capsule);
    return capsule;
  }

  TinyPlane* create_plane() {
    TinyPlane* plane = new TinyPlane();
    m_geoms.push_back(plane);
    return plane;
  }

  TinySphere* create_sphere(TinyScalar radius) {
    TinySphere* sphere = new TinySphere(radius);
    m_geoms.push_back(sphere);
    return sphere;
  }

  TinyCollisionDispatcher<TinyScalar, TinyConstants>
  get_collision_dispatcher() {
    return m_dispatcher;
  }

  TinyRigidBody* create_rigid_body(TinyScalar mass, const TinyGeometry* geom) {
    TinyRigidBody* body = new TinyRigidBody(mass, geom);
    this->m_bodies.push_back(body);
    return body;
  }

  TinyMultiBody* create_multi_body() {
    TinyMultiBody* body = new TinyMultiBody();
    this->m_multi_bodies.push_back(body);
    return body;
  }

  std::vector<TinyContactPointRigidBody<TinyScalar, TinyConstants>>
      m_allContacts;
  std::vector<std::vector<TinyContactPointMultiBody<TinyScalar, TinyConstants>>>
      m_allMultiBodyContacts;

  std::vector<TinyContactPoint<TinyScalar, TinyConstants>> m_contacts;

  static void compute_contacts_rigid_body_internal(
      std::vector<TinyRigidBody*> bodies,
      TinyCollisionDispatcher<TinyScalar, TinyConstants>* dispatcher,
      std::vector<TinyContactPointRigidBody<TinyScalar, TinyConstants>>&
          contactsOut,
      const TinyScalar& restitution, const TinyScalar& friction) {
    std::vector<TinyContactPoint<TinyScalar, TinyConstants>> contacts;
    {
      for (int i = 0; i < bodies.size(); i++) {
        for (int j = i + 1; j < bodies.size(); j++) {
          contacts.reserve(1);
          contacts.resize(0);

          int numContacts = dispatcher->computeContacts(
              bodies[i]->m_geometry, bodies[i]->m_world_pose,
              bodies[j]->m_geometry, bodies[j]->m_world_pose, contacts);
          for (int c = 0; c < numContacts; c++) {
            TinyContactPointRigidBody<TinyScalar, TinyConstants> rb_pt;
            TinyContactPoint<TinyScalar, TinyConstants>& pt = rb_pt;
            pt = contacts[c];
            rb_pt.m_rigid_body_a = bodies[i];
            rb_pt.m_rigid_body_b = bodies[j];
            // TODO(erwincoumans): combine friction and restitution based on
            // material properties of the two touching bodies
            rb_pt.m_restitution = restitution;
            rb_pt.m_friction = friction;
            contactsOut.push_back(rb_pt);
          }
        }
      }
    }
  }

  std::vector<TinyContactPointRigidBody<TinyScalar, TinyConstants>>
  compute_contacts_rigid_body(
      std::vector<TinyRigidBody*> bodies,
      TinyCollisionDispatcher<TinyScalar, TinyConstants>* dispatcher) {
    std::vector<TinyContactPointRigidBody<TinyScalar, TinyConstants>>
        contactsOut;
    compute_contacts_rigid_body_internal(bodies, dispatcher, contactsOut,
                                         default_restitution, default_friction);
    return contactsOut;
  }

  static void compute_contacts_multi_body_internal(
      std::vector<TinyMultiBody*> multi_bodies,
      TinyCollisionDispatcher<TinyScalar, TinyConstants>* dispatcher,
      std::vector<
          std::vector<TinyContactPointMultiBody<TinyScalar, TinyConstants>>>&
          contacts_out,
      const TinyScalar& restitution, const TinyScalar& friction) {
    int num_multi_bodies = multi_bodies.size();
    for (int i = 0; i < num_multi_bodies; i++) {
      TinyMultiBody* mb_a = multi_bodies[i];
      int num_links_a = mb_a->m_links.size();
      for (int j = i + 1; j < multi_bodies.size(); j++) {
        std::vector<TinyContactPoint<TinyScalar, TinyConstants>> contacts;

        TinyMultiBody* mb_b = multi_bodies[j];
        int num_links_b = mb_b->m_links.size();
        std::vector<TinyContactPointMultiBody<TinyScalar, TinyConstants>>
            contacts_ab;
        for (int ii = -1; ii < num_links_a; ii++) {
          const TinySpatialTransform& world_transform_a =
              mb_a->get_world_transform(ii);

          int num_geoms_a = mb_a->get_collision_geometries(ii).size();
          for (int iii = 0; iii < num_geoms_a; iii++) {
            const TinyGeometry* geom_a =
                mb_a->get_collision_geometries(ii)[iii];
            TinyPose<TinyScalar, TinyConstants> pose_a;
            const TinySpatialTransform& local_a =
                mb_a->get_collision_transforms(ii)[iii];
            TinySpatialTransform tr_a = world_transform_a * local_a;
            pose_a.m_position = tr_a.m_translation;
            tr_a.m_rotation.getRotation(pose_a.m_orientation);

            for (int jj = -1; jj < num_links_b; jj++) {
              const TinySpatialTransform& world_transform_b =
                  mb_b->get_world_transform(jj);
              int num_geoms_b = mb_b->get_collision_geometries(jj).size();
              for (int jjj = 0; jjj < num_geoms_b; jjj++) {
                const TinyGeometry* geom_b =
                    mb_b->get_collision_geometries(jj)[jjj];
                TinyPose<TinyScalar, TinyConstants> pose_b;
                const TinySpatialTransform& local_b =
                    mb_b->get_collision_transforms(jj)[jjj];
                TinySpatialTransform tr_b = world_transform_b * local_b;
                pose_b.m_position = tr_b.m_translation;
                tr_b.m_rotation.getRotation(pose_b.m_orientation);

                // printf("\tworld_transform_b: %.3f  %.3f  %.3f\n",
                // world_transform_b.m_translation[0],
                // world_transform_b.m_translation[1],
                // world_transform_b.m_translation[2]);
                //                printf("\tpose_b: %.3f  %.3f  %.3f\n",
                //                pose_b.m_position[0], pose_b.m_position[1],
                //                pose_b.m_position[2]);
                contacts.reserve(1);
                contacts.resize(0);
                int numContacts = dispatcher->computeContacts(
                    geom_a, pose_a, geom_b, pose_b, contacts);
                for (int c = 0; c < numContacts; c++) {
                  TinyContactPointMultiBody<TinyScalar, TinyConstants> mb_pt;
                  TinyContactPoint<TinyScalar, TinyConstants>& pt = mb_pt;
                  pt = contacts[c];
                  mb_pt.m_multi_body_a = multi_bodies[i];
                  mb_pt.m_multi_body_b = multi_bodies[j];
                  mb_pt.m_link_a = ii;
                  mb_pt.m_link_b = jj;
                  // TODO(erwincoumans): combine friction and restitution
                  // based on material properties of the two touching bodies
                  mb_pt.m_restitution = restitution;
                  mb_pt.m_friction = friction;
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

  std::vector<std::vector<TinyContactPointMultiBody<TinyScalar, TinyConstants>>>
  compute_contacts_multi_body(
      std::vector<TinyMultiBody*> bodies,
      TinyCollisionDispatcher<TinyScalar, TinyConstants>* dispatcher) {
    std::vector<
        std::vector<TinyContactPointMultiBody<TinyScalar, TinyConstants>>>
        contactsOut;
    compute_contacts_multi_body_internal(bodies, dispatcher, contactsOut,
                                         default_restitution, default_friction);
    return contactsOut;
  }

  void step(TinyScalar dt) {
    {
      m_allContacts.reserve(1024);
      m_allContacts.resize(0);
      m_allMultiBodyContacts.reserve(1024);
      m_allMultiBodyContacts.resize(0);
      submitProfileTiming("apply forces");
      for (int i = 0; i < m_bodies.size(); i++) {
        TinyRigidBody* b = m_bodies[i];
        b->apply_gravity(m_gravity_acceleration);
        b->apply_force_impulse(dt);
        b->clear_forces();
      }
      submitProfileTiming("");
    }
    {
      submitProfileTiming("compute contacts");
      compute_contacts_rigid_body_internal(m_bodies, &m_dispatcher,
                                           m_allContacts, default_restitution,
                                           default_friction);
      submitProfileTiming("");
    }
    {
      submitProfileTiming("compute multi body contacts");
      compute_contacts_multi_body_internal(
          m_multi_bodies, &m_dispatcher, m_allMultiBodyContacts,
          default_restitution, default_friction);
      submitProfileTiming("");
    }

    {
      submitProfileTiming("solve constraints");
      for (int i = 0; i < m_num_solver_iterations; i++) {
        for (int c = 0; c < m_allContacts.size(); c++) {
          m_constraint_solver->resolveCollision(m_allContacts[c], dt);
        }
      }
      // use outer loop in case the multi-body constraint solver requires it
      // (e.g. sequential impulse method)
      int mb_solver_iters;
      if (!m_mb_constraint_solver->needs_outer_iterations) {
        mb_solver_iters = 1;
      } else {
        mb_solver_iters = m_num_solver_iterations;
      }
      for (int i = 0; i < mb_solver_iters; i++) {
        for (int c = 0; c < m_allMultiBodyContacts.size(); c++) {
          m_mb_constraint_solver->resolveCollision(m_allMultiBodyContacts[c],
                                                   dt);
        }
      }
      submitProfileTiming("");
    }

    {
      submitProfileTiming("integrate");
      for (int i = 0; i < m_bodies.size(); i++) {
        TinyRigidBody* b = m_bodies[i];
        b->integrate(dt);
      }
      submitProfileTiming("");
    }
  }
};

#endif  // TINY_WORLD_H
