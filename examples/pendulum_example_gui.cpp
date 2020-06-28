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

#include <fenv.h>
#include <stdio.h>

#include <chrono>  // std::chrono::seconds
#include <thread>  // std::this_thread::sleep_for
#include "opengl_window/tiny_opengl3_app.h"

#include "fix64_scalar.h"
#include "pendulum.h"

#include "tiny_double_utils.h"
#include "tiny_file_utils.h"
#include "tiny_multi_body.h"
#include "tiny_rigid_body.h"
#include "tiny_world.h"

int main(int argc, char* argv[]) {

  TinyOpenGL3App app("pendulum_example_gui", 1024, 768);
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);
  //install ffmpeg in path and uncomment, to enable video recording
  //app.dump_frames_to_video("test.mp4");
 

  // Set NaN trap
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  
  TinyWorld<double, DoubleUtils> world;

  typedef TinyRigidBody<double, DoubleUtils> TinyRigidBodyDouble;

  std::vector<TinyRigidBody<double, DoubleUtils>*> bodies;
  std::vector<int> visuals;

  std::vector<TinyMultiBody<double, DoubleUtils>*> mbbodies;
  std::vector<int> mbvisuals;

  int num_spheres = 5;

  TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();
  init_compound_pendulum<double, DoubleUtils>(*mb, world, num_spheres);
  mbbodies.push_back(mb);

  int sphere_shape = app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  

  for (int i = 0; i < num_spheres; i++)
  {
      TinyVector3f pos(0, i*0.1, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f color(0.6, 0.6, 1);
      TinyVector3f scaling(0.05, 0.05, 0.05);
      int instance = app.m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
      mbvisuals.push_back(instance);
  }
  

#if 0
  if (visualizer->canSubmitCommand()) {
    for (int i = 0; i < mb->m_links.size(); i++) {
      int sphereId = visualizer->loadURDF("sphere_small.urdf");
      mbvisuals.push_back(sphereId);
      // apply some linear joint damping
      mb->m_links[i].m_damping = 5.;
    }
  }
#endif
  std::vector<double> q(mb->dof(), DoubleUtils::zero());
  std::vector<double> qd(mb->dof_qd(), DoubleUtils::zero());
  std::vector<double> tau(mb->dof_qd(), DoubleUtils::zero());
  std::vector<double> qdd(mb->dof_qd(), DoubleUtils::zero());

  TinyVector3<double, DoubleUtils> gravity(0., 0., -9.81);

  TinyMatrixXxX<double, DoubleUtils> M(mb->m_links.size(), mb->m_links.size());

  double dt = 1. / 240.;
  app.set_mp4_fps(1./dt);
  int upAxis = 2;
  while (!app.m_window->requested_exit()) 
  {
    app.m_renderer->update_camera(upAxis);
    DrawGridData data;
    data.upAxis = upAxis;
    app.draw_grid(data);

    // mb->clear_forces();
    mb->forward_kinematics(q, qd);

    { world.step(dt); }

    { mb->forward_dynamics(q, qd, tau, gravity, qdd); }

    {
      mb->integrate(q, qd, qdd, dt);
      // printf("q: [%.3f %.3f] \tqd: [%.3f %.3f]\n", q[0], q[1], qd[0], qd[1]);
      mb->mass_matrix(q, &M);
      M.print("M");
      if (qd[0] < -1e4) {
        assert(0);
      }
    }

    
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    // sync transforms
    int visual_index = 0;
    TinyVector3f prev_pos(0,0,0);
    TinyVector3f color(0,0,1);
    float line_width = 1;

    if (!mbvisuals.empty()) {
    for (int b = 0; b < mbbodies.size(); b++) {
        for (int l = 0; l<mbbodies[b]->m_links.size();l++) {
        const TinyMultiBody<double, DoubleUtils>* body = mbbodies[b];
        if (body->m_links[l].m_X_visuals.empty()) continue;

        int sphereId = mbvisuals[visual_index++];

        TinyQuaternion<double, DoubleUtils> rot;
        const TinySpatialTransform<double, DoubleUtils>& geom_X_world =
            body->m_links[l].m_X_world * body->m_links[l].m_X_visuals[0];
        TinyVector3f base_pos(geom_X_world.m_translation.getX(),
                            geom_X_world.m_translation.getY(),
                            geom_X_world.m_translation.getZ());
        geom_X_world.m_rotation.getRotation(rot);
        TinyQuaternionf base_orn(rot.getX(), rot.getY(), rot.getZ(),
                                rot.getW());
        if (l>=0)
        {
          printf("b=%d\n",b);
          app.m_renderer->draw_line(prev_pos, base_pos,color, line_width);
        }
        else
	{
		printf("!! b=%d\n",b);
	}
        prev_pos = base_pos;
        app.m_renderer->write_single_instance_transform_to_cpu(base_pos, base_orn, sphereId);
        }
    }
    }
    app.m_renderer->render_scene();
    app.m_renderer->write_transforms();
    app.swap_buffer();
  }

  
  return 0;
}
