#ifndef TINY_RAYCAST_H
#define TINY_RAYCAST_H

#include "tiny_geometry.h"
#include <vector>

//batch ray-sphere against TinyUrdfCollision objects
template <typename TinyScalar, typename TinyConstants>
struct TinyRaycastResult
{
	TinyScalar m_hit_fraction;
	int m_collider_index;
};

template <typename TinyScalar, typename TinyConstants>
bool TinyRaycastResultComparison(const TinyRaycastResult<TinyScalar, TinyConstants>& i1,
	const TinyRaycastResult<TinyScalar, TinyConstants>& i2)
{
	return (i1.m_hit_fraction < i2.m_hit_fraction);
};

template <typename TinyScalar, typename TinyConstants>
struct TinyRaycast
{
	typedef ::TinyRaycastResult<TinyScalar, TinyConstants> TinyRaycastResult;
	typedef ::TinyVector3<TinyScalar, TinyConstants> TinyVector3;
	typedef ::TinyUrdfCollision<TinyScalar, TinyConstants> TinyUrdfCollision;
	
	bool ray_box(const TinyVector3& ray_from_local, const TinyVector3& ray_to_local, const TinyUrdfCollision& box,
		TinyRaycastResult& hit0, TinyRaycastResult& hit1)
	{
		TinyScalar exit_fraction = TinyConstants::one();
		TinyScalar enter_fraction = -TinyConstants::one();
		TinyVector3 cur_hit_normal = TinyVector3::zero();
		int num_faces = 6;
		
		TinyVector3 faces[6] = {
			TinyVector3(-TinyConstants::one(), TinyConstants::zero(), TinyConstants::zero()),
			TinyVector3(TinyConstants::one(), TinyConstants::zero(), TinyConstants::zero()),
			TinyVector3(TinyConstants::zero(), -TinyConstants::one(), TinyConstants::zero()),
			TinyVector3(TinyConstants::zero(), TinyConstants::one(), TinyConstants::zero()),
			TinyVector3(TinyConstants::zero(), TinyConstants::zero(), -TinyConstants::one()),
			TinyVector3(TinyConstants::zero(), TinyConstants::zero(), TinyConstants::one()) };
		TinyScalar plane_constants[6] = {
			-box.geometry.m_box.m_extents[0],
			-box.geometry.m_box.m_extents[0],
			-box.geometry.m_box.m_extents[1],
			-box.geometry.m_box.m_extents[1],
			-box.geometry.m_box.m_extents[2],
			-box.geometry.m_box.m_extents[2],
		};

		for (int i = 0; i < num_faces; i++)
		{
			const TinyVector3& face = faces[i];
			TinyScalar plane_constant = plane_constants[i];
			TinyScalar from_plane_dist = ray_from_local.dot(face) + plane_constant;
			TinyScalar to_plane_dist = ray_to_local.dot(face) + plane_constant;
			if (from_plane_dist < TinyConstants::zero())
			{
				if (to_plane_dist >= TinyConstants::zero())
				{
					TinyScalar fraction = from_plane_dist / (from_plane_dist - to_plane_dist);
					if (exit_fraction > fraction)
					{
						exit_fraction = fraction;
					}
				}
			}
			else
			{
				if (to_plane_dist <= TinyConstants::zero())
				{
					TinyScalar fraction = from_plane_dist / (from_plane_dist - to_plane_dist);
					if (enter_fraction <= fraction)
					{
						enter_fraction = fraction;
						cur_hit_normal = face;
					}
				}
				else
				{
					return false;
				}
			}
			if (exit_fraction <= enter_fraction)
				return false;
		}

		if (enter_fraction < TinyConstants::zero())
			return false;

		hit0.m_hit_fraction = enter_fraction;
		hit1.m_hit_fraction = exit_fraction;
		return true;
	}


	std::vector<std::vector<TinyRaycastResult> > cast_rays(const std::vector<TinyVector3>& rays_from,
													const std::vector<TinyVector3>& rays_to,
													const std::vector<TinyUrdfCollision>& collision_objects)
	{
		std::vector < std::vector < TinyRaycastResult> > results;
		results.resize(rays_from.size());

		for (int ray = 0; ray < rays_from.size(); ray++)
		{
			const TinyVector3& ray_from = rays_from[ray];
			const TinyVector3& ray_to = rays_to[ray];
			for (int col = 0; col < collision_objects.size(); col++)
			{
				const TinyUrdfCollision& collider = collision_objects[col];
				//perform a collision check, and add hit results(s)
				switch (collider.geometry.geom_type)
				{
				case TINY_SPHERE_TYPE:
				{
					TinyScalar radius = collider.geometry.m_sphere.m_radius;
					TinyVector3 rs = ray_from - collider.origin_xyz;
					TinyVector3 ray_dir = ray_to - ray_from;
					TinyScalar a = ray_dir.dot(ray_dir);
					TinyScalar b = rs.dot(ray_dir);
					TinyScalar c = rs.dot(rs) - (radius * radius);
					TinyScalar d = b * b - a * c;
					if (d > TinyConstants::zero())
					{
						TinyScalar t0 = (-b - TinyConstants::sqrt1(d)) / a;
						TinyScalar t1 = (-b + TinyConstants::sqrt1(d)) / a;
						if (t0 >= TinyConstants::zero() && t0 <= TinyConstants::one())
						{
							TinyRaycastResult hit;
							hit.m_hit_fraction = t0;
							hit.m_collider_index = col;
							results[ray].push_back(hit);
						}
						if (t1 >= TinyConstants::zero() && t1 <= TinyConstants::one())
						{
							TinyRaycastResult hit;
							hit.m_hit_fraction = t1;
							hit.m_collider_index = col;
							results[ray].push_back(hit);
						}
					}
					break;
				}
				case TINY_BOX_TYPE:
				{
					//transform ray to local coordinates
					TinyQuaternion<TinyScalar, TinyConstants> orn;
					orn.set_euler_rpy(collider.origin_rpy);
					TinyPose<TinyScalar, TinyConstants> pose(collider.origin_xyz, orn);
					TinyVector3 ray_from_local = pose.inverse_transform(ray_from);
					TinyVector3 ray_to_local = pose.inverse_transform(ray_to);
					TinyRaycastResult hit0, hit1;
					hit0.m_collider_index = col;
					hit1.m_collider_index = col;
					if (ray_box(ray_from_local, ray_to_local, collider, hit0, hit1))
					{
						results[ray].push_back(hit0);
						results[ray].push_back(hit1);
					}
					break;
				}
				default:
				{
					printf("unsupported type in cast_rays: %d\n", collider.geometry.geom_type);
				}
				};
			}
		}
		
		for (int r = 0; r < results.size(); r++)
		{
			std::sort(results[r].begin(), results[r].end(), TinyRaycastResultComparison<TinyScalar, TinyConstants>);
		}

		return results;
	}
};


#endif //TINY_RAYCAST_H