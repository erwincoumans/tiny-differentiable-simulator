#ifndef _TINY_RAYCAST_H
#define _TINY_RAYCAST_H

#include <vector>
#include "geometry.hpp"
#include "urdf_structures.hpp"

namespace TINY
{
// batch ray-sphere against TinyUrdfCollision objects
template <typename TinyScalar, typename TinyConstants>
struct TinyRaycastResult {
  TinyScalar m_hit_fraction;
  int m_collider_index;
};

template <typename TinyScalar, typename TinyConstants>
bool TinyRaycastResultComparison(
    const TinyRaycastResult<TinyScalar, TinyConstants> &i1,
    const TinyRaycastResult<TinyScalar, TinyConstants> &i2) {
  return (i1.m_hit_fraction < i2.m_hit_fraction);
};

template <typename TinyScalar, typename TinyConstants, typename TinyAlgebra>
struct TinyRaycast {
  typedef ::TINY::TinyRaycastResult<TinyScalar, TinyConstants> TinyRaycastResult;
  typedef typename TinyAlgebra::Vector3 TinyVector3;
  typedef ::tds::UrdfCollision<TinyAlgebra> TinyUrdfCollision;

  bool ray_box(const TinyVector3 &ray_from_local,
               const TinyVector3 &ray_to_local, const TinyUrdfCollision &box,
               TinyRaycastResult &hit0, TinyRaycastResult &hit1) {
    TinyScalar exit_fraction = TinyConstants::one();
    TinyScalar enter_fraction = -TinyConstants::one();
    TinyVector3 cur_hit_normal = TinyAlgebra::zero3();
    int num_faces = 6;

    TinyVector3 faces[6] = {
        TinyVector3(-TinyConstants::one(), TinyConstants::zero(),
                    TinyConstants::zero()),
        TinyVector3(TinyConstants::one(), TinyConstants::zero(),
                    TinyConstants::zero()),
        TinyVector3(TinyConstants::zero(), -TinyConstants::one(),
                    TinyConstants::zero()),
        TinyVector3(TinyConstants::zero(), TinyConstants::one(),
                    TinyConstants::zero()),
        TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                    -TinyConstants::one()),
        TinyVector3(TinyConstants::zero(), TinyConstants::zero(),
                    TinyConstants::one())};
    TinyScalar plane_constants[6] = {
        -box.geometry.box.extents[0], -box.geometry.box.extents[0],
        -box.geometry.box.extents[1], -box.geometry.box.extents[1],
        -box.geometry.box.extents[2], -box.geometry.box.extents[2],
    };

    for (int i = 0; i < num_faces; i++) {
      const TinyVector3 &face = faces[i];
      TinyScalar plane_constant = plane_constants[i];
      TinyScalar from_plane_dist = ray_from_local.dot(face) + plane_constant;
      TinyScalar to_plane_dist = ray_to_local.dot(face) + plane_constant;
      if (from_plane_dist < TinyConstants::zero()) {
        if (to_plane_dist >= TinyConstants::zero()) {
          TinyScalar fraction =
              from_plane_dist / (from_plane_dist - to_plane_dist);
          if (exit_fraction > fraction) {
            exit_fraction = fraction;
          }
        }
      } else {
        if (to_plane_dist <= TinyConstants::zero()) {
          TinyScalar fraction =
              from_plane_dist / (from_plane_dist - to_plane_dist);
          if (enter_fraction <= fraction) {
            enter_fraction = fraction;
            cur_hit_normal = face;
          }
        } else {
          return false;
        }
      }
      if (exit_fraction <= enter_fraction) return false;
    }

    if (enter_fraction < TinyConstants::zero()) return false;

    hit0.m_hit_fraction = enter_fraction;
    hit1.m_hit_fraction = exit_fraction;
    return true;
  }

  std::vector<std::vector<TinyRaycastResult>> cast_rays(
      const std::vector<TinyVector3> &rays_from,
      const std::vector<TinyVector3> &rays_to,
      const std::vector<TinyUrdfCollision> &collision_objects) {
    std::vector<std::vector<TinyRaycastResult>> results;
    results.resize(rays_from.size());

    for (int ray = 0; ray < rays_from.size(); ray++) {
      const TinyVector3 &ray_from = rays_from[ray];
      const TinyVector3 &ray_to = rays_to[ray];
      for (int col = 0; col < collision_objects.size(); col++) {
        const TinyUrdfCollision &collider = collision_objects[col];
        // perform a collision check, and add hit results(s)
        switch (collider.geometry.geom_type) {
          case ::tds::TINY_SPHERE_TYPE: {
            TinyScalar radius = collider.geometry.sphere.radius;
            TinyVector3 rs = ray_from - collider.origin_xyz;
            TinyVector3 ray_dir = ray_to - ray_from;
            TinyScalar a = ray_dir.dot(ray_dir);
            TinyScalar b = rs.dot(ray_dir);
            TinyScalar c = rs.dot(rs) - (radius * radius);
            TinyScalar d = b * b - a * c;
            if (d > TinyConstants::zero()) {
              TinyScalar t0 = (-b - TinyConstants::sqrt1(d)) / a;
              TinyScalar t1 = (-b + TinyConstants::sqrt1(d)) / a;
              if (t0 >= TinyConstants::zero() && t0 <= TinyConstants::one()) {
                TinyRaycastResult hit;
                hit.m_hit_fraction = t0;
                hit.m_collider_index = col;
                results[ray].push_back(hit);
              }
              if (t1 >= TinyConstants::zero() && t1 <= TinyConstants::one()) {
                TinyRaycastResult hit;
                hit.m_hit_fraction = t1;
                hit.m_collider_index = col;
                results[ray].push_back(hit);
              }
            }
            break;
          }
          case ::tds::TINY_BOX_TYPE: {
            // transform ray to local coordinates
            typename TinyAlgebra::Quaternion orn;
            orn = TinyAlgebra::quat_from_euler_rpy(collider.origin_rpy);
            //typename TinyAlgebra::Quaternion orn;
            //orn.set_euler_rpy(collider.origin_rpy);
            tds::Pose<TinyAlgebra> pose(collider.origin_xyz, orn);
            TinyVector3 ray_from_local = pose.inverse_transform(ray_from);
            TinyVector3 ray_to_local = pose.inverse_transform(ray_to);
            TinyRaycastResult hit0, hit1;
            hit0.m_collider_index = col;
            hit1.m_collider_index = col;
            if (ray_box(ray_from_local, ray_to_local, collider, hit0, hit1)) {
              results[ray].push_back(hit0);
              results[ray].push_back(hit1);
            }
            break;
          }
          default: {
            printf("unsupported type in cast_rays: %d\n",
                   collider.geometry.geom_type);
          }
        };
      }
    }

    for (int r = 0; r < results.size(); r++) {
      std::sort(results[r].begin(), results[r].end(),
                TinyRaycastResultComparison<TinyScalar, TinyConstants>);
    }

    return results;
  }

  TinyScalar volume(const std::vector<std::vector<TinyRaycastResult>> &results,
                    int num_objects) {
    TinyScalar vol = TinyConstants::zero();
    std::vector<int> inside_primitive_array;
    for (int ray = 0; ray < results.size(); ray++) {
      TinyScalar prev_fraction = TinyConstants::zero();
      const std::vector<TinyRaycastResult> &hits = results[ray];
      inside_primitive_array.resize(0);
      inside_primitive_array.resize(num_objects, 0);
      int inside_primitives = 0;
      for (int i = 0; i < hits.size(); i++) {
        const TinyRaycastResult &hit = hits[i];
        TinyScalar fraction = hit.m_hit_fraction;
        int prim_uid = hit.m_collider_index;
        if (inside_primitive_array[prim_uid] > 0) {
          // we must be leaving this primitive
          inside_primitive_array[prim_uid]--;
          inside_primitives = inside_primitives - 1;
          if (inside_primitives == 0) {
            vol += (fraction - prev_fraction);
          }
        } else {
          inside_primitive_array[prim_uid]++;
          if (inside_primitives == 0) {
            prev_fraction = fraction;
          }
          inside_primitives = inside_primitives + 1;
        }
      }
    }

    return vol;
  }
  TinyScalar intersection_volume(
      const std::vector<std::vector<TinyRaycastResult>> &results_target,
      const std::vector<std::vector<TinyRaycastResult>> &results_prims,
      int num_objects) {
    TinyScalar intersection_volume = TinyConstants::zero();
    std::vector<int> inside_primitive_array;
    for (int ray = 0; ray < results_target.size(); ray++) {
      const std::vector<TinyRaycastResult> &target_hits = results_target[ray];
      const std::vector<TinyRaycastResult> &prims_hits = results_prims[ray];
      if (target_hits.size() && prims_hits.size()) {
        inside_primitive_array.resize(0);
        inside_primitive_array.resize(num_objects);
        int target_hit_index = 0;
        int prim_hit_index = 0;
        int inside_primitives = 0;
        int inside_target = 0;
        TinyScalar prev_fraction = TinyConstants::zero();
        TinyScalar cur_target_fraction = TinyConstants::zero();
        TinyScalar cur_prim_fraction = TinyConstants::zero();

        while (prim_hit_index < prims_hits.size() &&
               target_hit_index < target_hits.size()) {
          const TinyRaycastResult &next_target_hit =
              target_hits[target_hit_index];
          const TinyRaycastResult &next_prim_hit = prims_hits[prim_hit_index];
          TinyScalar prim_fraction = next_prim_hit.m_hit_fraction;
          TinyScalar target_fraction = next_target_hit.m_hit_fraction;
          int prim_uid = next_prim_hit.m_collider_index;

          if (prim_fraction <= target_fraction) {
            cur_prim_fraction = prim_fraction;
            // are we entering a new or existing primitive or leaving a existing
            // primitive ?
            if (inside_primitive_array[prim_uid] > 0) {
              // we must be leaving this primitive
              inside_primitive_array[prim_uid]--;
              inside_primitives--;
              if (inside_target && (inside_primitives == 0)) {
                intersection_volume += (prim_fraction - prev_fraction);
              }
            } else {
              // we must be entering this primitive
              if (inside_primitives == 0) {
                prev_fraction = prim_fraction;
              }
              inside_primitive_array[prim_uid]++;
              inside_primitives++;
            }
            prim_hit_index++;
          } else {
            // entering or leaving target ?
            if (inside_target) {
              if (inside_primitives > 0) {
                intersection_volume += (target_fraction - prev_fraction);
              }
              inside_target = inside_target - 1;
            } else {
              // we are entering the target
              prev_fraction = target_fraction;
              inside_target = inside_target + 1;
            }
            target_hit_index += 1;
          }
        }
      }
    }
    return intersection_volume;
  }
};
};
#endif  // _TINY_RAYCAST_H
