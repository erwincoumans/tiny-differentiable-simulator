// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TINY_CAMERA_H
#define TINY_CAMERA_H

#include "math/tiny/tiny_float_utils.h"
#include <array>


struct TinyCamera {
  struct TinyCameraInternalData* m_data;

  TinyCamera(const TinyCamera& other);

  TinyCamera();
  virtual ~TinyCamera();

  void update();
  virtual void get_camera_projection_matrix(float m[16]) const;
  virtual void get_camera_view_matrix(float m[16]) const;

  virtual std::array<float, 16>  get_camera_projection_matrix2() const
	{
		std::array<float, 16> proj;
		get_camera_projection_matrix(&proj[0]);
		return proj;		
	}
  virtual std::array<float, 16>  get_camera_view_matrix2() const
  {
		std::array<float, 16> view;
		get_camera_view_matrix(&view[0]);
		return view;
	}
  virtual void get_camera_target_position(::TINY::TinyVector3f& pos) const;
  virtual void get_camera_position(::TINY::TinyVector3f& pos) const;

  virtual void get_camera_target_position(double pos[3]) const;

  virtual void set_camera_target_position(float x, float y, float z);
  virtual void set_camera_distance(float dist);
  virtual float get_camera_distance() const;

  virtual void set_camera_up_vector(float x, float y, float z);
  void get_camera_up_vector(float up[3]) const;

  void get_camera_forward_vector(float fwd[3]) const;

  /// the set_camera_up_axis will call the 'set_camera_up_vector' and
  /// 'setCameraForwardVector'
  virtual void set_camera_up_axis(int axis);
  virtual int get_camera_up_axis() const;

  virtual void set_camera_yaw(float yaw);
  virtual float get_camera_yaw() const;

  virtual void set_camera_pitch(float pitch);
  virtual float get_camera_pitch() const;

  virtual void set_aspect_ratio(float ratio);
  virtual float get_aspect_ratio() const;

  virtual float get_camera_frustum_far() const;
  virtual float get_camera_frustum_near() const;

  virtual void set_camera_frustum_far(float far);
  virtual void set_camera_frustum_near(float near);

  virtual void copy_data(const TinyCamera& other);
};

#endif  // TINY_CAMERA_H
