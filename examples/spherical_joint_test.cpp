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


#include "math/tiny/fix64_scalar.h"
#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "utils/pendulum_spherical_joints.hpp"


#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "multi_body.hpp"
#include "world.hpp"

using namespace TINY;
using namespace tds;
#include "visualizer/opengl/tiny_opengl3_app.h"

#ifdef USE_TINY
  #include "math/tiny/tiny_algebra.hpp"
#else
  #include "math/eigen_algebra.hpp"
#endif


int main(int argc, char* argv[]) {

#ifdef USE_TINY
  typedef TinyAlgebra<double, DoubleUtils> Algebra;
#else
  typedef EigenAlgebra Algebra;
#endif
  typedef typename Algebra::Vector3 Vector3;
  typedef typename Algebra::Quaternion Quaternion;
  typedef typename Algebra::VectorX VectorX;
  typedef typename Algebra::Matrix3 Matrix3;
  typedef typename Algebra::Matrix3X Matrix3X;
  typedef typename Algebra::MatrixX MatrixX;
  typedef tds::RigidBody<Algebra> RigidBody;
  typedef tds::RigidBodyContactPoint<Algebra> RigidBodyContactPoint;
  typedef tds::MultiBody<Algebra> MultiBody;
  typedef tds::MultiBodyContactPoint<Algebra> MultiBodyContactPoint;
  typedef tds::Transform<Algebra> Transform;

#ifdef USE_TINY
  TinyOpenGL3App app("spherical_joint_tiny", 1024, 768);
#else
  TinyOpenGL3App app("spherical_joint_eigen", 1024, 768);
#endif
  app.m_renderer->init();
  app.set_up_axis(2);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_yaw(90);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);
  //install ffmpeg in path and uncomment, to enable video recording
  //app.dump_frames_to_video("test.mp4");
 

  // Set NaN trap
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  
  tds::World<Algebra> world;

  std::vector<RigidBody*> bodies;
  std::vector<int> visuals;

  std::vector<MultiBody*> mbbodies;
  std::vector<int> mbvisuals;

  int num_spheres = 5;
  int num_multibodies = 1;


  bool add_plane = true;
  if(add_plane)
  {
      MultiBody* planemb = world.create_multi_body();
      planemb->set_floating_base(false);
      Transform base_X_geom;
      base_X_geom.set_identity();
      tds::Plane<Algebra>* plane = new tds::Plane<Algebra>();
      planemb->collision_geometries().push_back(plane);
      planemb->collision_transforms(-1).push_back(base_X_geom);
  }

  std::vector<MatrixX> MassM;
  tds::UrdfCache<Algebra> cache;
  for (int ii = 0; ii < num_multibodies; ii++){

#define USE_URDF
#ifdef USE_URDF
    
    std::string urdf_filename;
    tds::FileUtils::find_file("pendulum5spherical.urdf", urdf_filename);
    MultiBody* mb = cache.construct(urdf_filename, world, false, false);
    
    mb->qd()[2] = Algebra::fraction(1, 2);
#else
    MultiBody* mb = world.create_multi_body();
    init_spherical_compound_pendulum<Algebra>(*mb, world, num_spheres);
    // Set initial orientation and velocity
    mb->q()[0] = Algebra::sqrt(Algebra::fraction(1, 2));
    mb->q()[3] = Algebra::sqrt(Algebra::fraction(1, 2));
    mb->qd()[1] = Algebra::fraction(1, 2);
#endif
    mb->set_position(Vector3(ii, 0, 2));

    
    mbbodies.push_back(mb);
    //MatrixX M(mb->dof_qd(), mb->dof_qd());
    //MassM.push_back(M);
  }

  int sphere_shape = app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
  

  for (int i = 0; i < num_spheres * num_multibodies; i++)
  {
      TinyVector3f pos(0, i*0.1, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f color(0.6, 0.6, 1);
      TinyVector3f scaling(0.05, 0.05, 0.05);
      int instance = app.m_renderer->register_graphics_instance(sphere_shape, pos, orn, color, scaling);
      mbvisuals.push_back(instance);
  }

// Set some stiffness and/or damping for test purposes
  for (auto &link: mbbodies[0]->links_){
      link.damping = Algebra::one() * 0.5;
//      link.stiffness = Algebra::one() * 300;
  }

  Vector3 gravity(0., 0., -9.81);

  int fps = 480;
  double dt = 1. / fps;
  //app.set_mp4_fps(fps);
  int upAxis = 2;
  
  int sync_counter = 0;
  int frameskip_gfx_sync = 10;

  while (!app.m_window->requested_exit()) 
  {
    app.m_renderer->update_camera(upAxis);
    DrawGridData data;
    data.drawAxis = true;
    data.upAxis = upAxis;
    app.draw_grid(data);
    for (auto &mb: mbbodies){
      mb->clear_forces();
      tds::forward_kinematics(*mb);

      { world.step(dt); }

      { tds::forward_dynamics(*mb, gravity); }

      {
        tds::integrate_euler(*mb, mb->q(), mb->qd(), mb->qdd(), dt);


        // printf("q: [%.3f %.3f] \tqd: [%.3f %.3f]\n", q[0], q[1], qd[0], qd[1]);
        //tds::mass_matrix(*mb, &MassM[0]);

        //M.print("M");
        if (mb->qd()[0] < -1e4) {
          assert(0);
        }
      }
    }

    
   
    // sync transforms
    int visual_index = 0;
    TinyVector3f prev_pos(0,0,0);
    TinyVector3f color(0,0,1);
    float line_width = 1;

    sync_counter++;

    if (!mbvisuals.empty() && (sync_counter > frameskip_gfx_sync)) {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt*frameskip_gfx_sync));
        sync_counter = 0;
        for (int b = 0; b < mbbodies.size(); b++) {
            for (int l = 0; l<mbbodies[b]->links().size();l++) {
                const MultiBody* body = mbbodies[b];
                if (body->links()[l].X_visuals.empty()) continue;

                int sphereId = mbvisuals[visual_index++];

                Quaternion rot;
                const Transform& geom_X_world = 
                       body->links()[l].X_world * body->links()[l].X_visuals[0];

                TinyVector3f base_pos(geom_X_world.translation.x(),
                                    geom_X_world.translation.y(),
                                    geom_X_world.translation.z());
                rot = Algebra::matrix_to_quat(geom_X_world.rotation);
                TinyQuaternionf base_orn(rot.x(), rot.y(), rot.z(),
                                        rot.w());
                if (l == 0){
                    prev_pos.setValue(body->get_world_transform(-1).translation[0],
                        body->get_world_transform(-1).translation[1],
                        body->get_world_transform(-1).translation[2]);
                  app.m_renderer->draw_line(prev_pos, base_pos,color, line_width);
                }
                else if (l>0)
                {
                  //printf("b=%d\n",b);
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
        app.m_renderer->write_transforms();
        app.m_renderer->render_scene();
        
        app.swap_buffer();
    }
   
  }

  
  return 0;
}
