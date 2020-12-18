// clang-format off
#include "utils/differentiation.hpp"
#include "utils/cuda_codegen.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "urdf/urdf_cache.hpp"
#include "utils/stopwatch.hpp"
#include "visualizer/opengl/visualizer.h"
#include "opengl_urdf_visualizer.h"
#include "utils/file_utils.hpp"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

// clang-format on
using namespace TINY;

double knee_angle = -0.5;
double abduction_angle = 0.2;
double initial_poses[] = {
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
    abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
};

// function to be converted to CUDA code
template <typename Algebra>
struct LaikagoSimulation {
    using Scalar = typename Algebra::Scalar;
    tds::UrdfCache<Algebra> cache;
    std::string m_urdf_filename;
    tds::World<Algebra> world;
    tds::MultiBody<Algebra>* system = nullptr;

    int num_timesteps{ 1 };
    Scalar dt{ Algebra::from_double(1e-3) };

    int input_dim() const { return system->dof() + system->dof_qd(); }
    int state_dim() const {
        return system->dof() + system->dof_qd() + system->num_links() * 7;
    }
    int output_dim() const { return num_timesteps * state_dim(); }

    LaikagoSimulation() {
        std::string plane_filename;
        tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
        cache.construct(plane_filename, world, false, false);
        tds::FileUtils::find_file("laikago/laikago_toes_zup.urdf", m_urdf_filename);
        system = cache.construct(m_urdf_filename, world, false, true);
        system->base_X_world().translation = Algebra::unit3_z();
    }

    std::vector<Scalar> operator()(const std::vector<Scalar>& v) {
        assert(static_cast<int>(v.size()) == input_dim());
        system->initialize();
        //copy input into q, qd
        for (int i = 0; i < system->dof(); ++i) {
            system->q(i) = v[i];
        }
        for (int i = 0; i < system->dof_qd(); ++i) {
            system->qd(i) = v[i + system->dof()];
        }
        std::vector<Scalar> result(output_dim());
        for (int t = 0; t < num_timesteps; ++t) {


            // pd control
            if (1) {
                // use PD controller to compute tau
                int qd_offset = system->is_floating() ? 6 : 0;
                int q_offset = system->is_floating() ? 7 : 0;
                int num_targets = system->tau_.size() - qd_offset;
                std::vector<double> q_targets;
                q_targets.resize(system->tau_.size());

                Scalar kp = 150;
                Scalar kd = 3;
                Scalar max_force = 550;
                int param_index = 0;

                for (int i = 0; i < system->tau_.size(); i++) {
                    system->tau_[i] = 0;
                }
                int tau_index = 0;
                int pose_index = 0;
                for (int i = 0; i < system->links_.size(); i++) {
                    if (system->links_[i].joint_type != tds::JOINT_FIXED) {
                        Scalar q_desired = initial_poses[pose_index++];
                        Scalar q_actual = system->q_[q_offset];
                        Scalar qd_actual = system->qd_[qd_offset];
                        Scalar position_error = (q_desired - q_actual);
                        Scalar desired_velocity = 0;
                        Scalar velocity_error = (desired_velocity - qd_actual);
                        Scalar force = kp * position_error + kd * velocity_error;

                        force = Algebra::max(force, -max_force);
                        force = Algebra::min(force, max_force);

                        system->tau_[tau_index] = force;
                        q_offset++;
                        qd_offset++;
                        param_index++;
                        tau_index++;
                    }
                }
            }

            tds::forward_dynamics(*system, world.get_gravity());

            system->clear_forces();

            integrate_euler_qdd(*system, dt);

            world.step(dt);

            tds::integrate_euler(*system, dt);

            //copy q, qd, link world poses (for rendering) to output
            int j = 0;
            for (int i = 0; i < system->dof(); ++i, ++j) {
                result[j] = system->q(i);
            }
            for (int i = 0; i < system->dof_qd(); ++i, ++j) {
                result[j] = system->qd(i);
            }
            for (const auto link : *system) {
                //assert(link.X_visuals.size());
                {
                    tds::Transform visual_X_world = link.X_world * link.X_visuals[0];
                    result[j++] = visual_X_world.translation[0];
                    result[j++] = visual_X_world.translation[1];
                    result[j++] = visual_X_world.translation[2];
                    Algebra::Quaternion orn = Algebra::matrix_to_quat(visual_X_world.rotation);
                    result[j++] = orn.x();
                    result[j++] = orn.y();
                    result[j++] = orn.z();
                    result[j++] = orn.w();
                }
                
            }
        }
        return result;
    }
};



int main(int argc, char* argv[]) {
  using Scalar = double;
  using CGScalar = typename CppAD::cg::CG<Scalar>;
  using Dual = typename CppAD::AD<CGScalar>;

  using DiffAlgebra =
      tds::default_diff_algebra<tds::DIFF_CPPAD_CODEGEN_AUTO, 0, Scalar>::type;

  LaikagoSimulation<DiffAlgebra> simulation;

  // trace function with all zeros as input
  std::vector<Dual> ax(simulation.input_dim(), Dual(0));
  //quaternion 'w' = 1
  ax[3] = 1;
  //height of Laikago at 0.7 meter
  ax[6] = 0.7;
  
  CppAD::Independent(ax);
  std::vector<Dual> ay(simulation.output_dim());
  std::cout << "Tracing function for code generation...\n";
  ay = simulation(ax);

  CppAD::ADFun<CGScalar> tape;
  tape.Dependent(ax, ay);
  tape.optimize();

  tds::Stopwatch timer;
  timer.start();
  std::string model_name = "cuda_model";
  tds::CudaSourceGen<Scalar> cgen(tape, model_name);
  cgen.setCreateForwardZero(true);

  tds::CudaLibraryProcessor p(&cgen);
#if CPPAD_CG_SYSTEM_WIN
  std::string nvcc_path = tds::exec("where nvcc");
#else
  std::string nvcc_path = tds::exec("which nvcc");
#endif
  std::cout << "Using [" << nvcc_path << "]" << std::endl;
  p.nvcc_path() = nvcc_path;
  p.generate_code();
  
  //comment-out to re-use previously build CUDA shared library
  p.create_library();

  // CppAD::cg::ModelLibraryCSourceGen<Scalar> libcgen(cgen);
  // libcgen.setVerbose(true);
  // CppAD::cg::DynamicModelLibraryProcessor<Scalar> p(libcgen);
  // auto compiler = std::make_unique<CppAD::cg::ClangCompiler<Scalar>>();
  // compiler->setSourcesFolder("cgen_srcs");
  // compiler->setSaveToDiskFirst(true);
  // compiler->addCompileFlag("-O" + std::to_string(1));
  // p.setLibraryName(model_name);
  // p.createDynamicLibrary(*compiler, false);

  // create model to load shared library
  tds::CudaModel<Scalar> model(model_name);

  // how many threads to run on the GPU
  int num_total_threads = 1024;

  std::vector<std::vector<Scalar>> outputs(
      num_total_threads, std::vector<Scalar>(simulation.output_dim()));

  std::vector<std::vector<Scalar>> inputs(num_total_threads);

  OpenGLUrdfVisualizer<DiffAlgebra> visualizer;
  visualizer.delete_all();
  TinyOpenGL3App& app = visualizer.m_opengl_app;
  //TinyOpenGL3App app("CUDA Pendulum", 1024, 768);
  app.m_renderer->init();
  int upAxis = 2;
  app.set_up_axis(upAxis);
  app.m_renderer->get_active_camera()->set_camera_distance(4);
  app.m_renderer->get_active_camera()->set_camera_pitch(-30);
  app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);
  // install ffmpeg in path and uncomment, to enable video recording
  // app.dump_frames_to_video("test.mp4");




  {
      std::vector<int> shape_ids;
      std::string plane_filename;
      tds::FileUtils::find_file("plane100.obj", plane_filename);
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      visualizer.load_obj(plane_filename, pos, orn, scaling, shape_ids);
  }


  bool create_instances = false;
  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string texture_path = "";
  std::string file_and_path;
  tds::FileUtils::find_file("laikago/laikago_toes_zup.urdf", file_and_path);
  auto urdf_structures = simulation.cache.retrieve(file_and_path);// contact_sim.m_urdf_filename);
  tds::FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);


  
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances = 0;
  int sync_counter = 0;
  int frameskip_gfx_sync = 1;

  for (int t = 0; t < num_total_threads; t++)
  {
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      int uid = urdf_structures.base_links[0].urdf_visual_shapes[0].visual_shape_uid;
      OpenGLUrdfVisualizer<DiffAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
      int instance = -1;
      int num_instances_per_link = 0;
      for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
      {
          int sphere_shape = vis_link.visual_shape_uids[v];
          ::TINY::TinyVector3f color(1, 1, 1);
          //visualizer.m_b2vis
          instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
              sphere_shape, pos, orn, color, scaling);
          visual_instances.push_back(instance);
          num_instances_per_link++;
          simulation.system->visual_instance_uids().push_back(instance);
      }
      num_base_instances = num_instances_per_link;

      for (int i = 0; i < simulation.system->num_links(); ++i) {


          int uid = urdf_structures.links[i].urdf_visual_shapes[0].visual_shape_uid;
          OpenGLUrdfVisualizer<DiffAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
          int instance = -1;
          int num_instances_per_link = 0;
          for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
          {
              int sphere_shape = vis_link.visual_shape_uids[v];
              ::TINY::TinyVector3f color(1, 1, 1);
              //visualizer.m_b2vis
              instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                  sphere_shape, pos, orn, color, scaling);
              visual_instances.push_back(instance);
              num_instances_per_link++;

              simulation.system->links_[i].visual_instance_uids.push_back(instance);
          }
          num_instances.push_back(num_instances_per_link);
      }
  }


  model.forward_zero.allocate(num_total_threads);

  std::vector< TinyVector3f> positions;
  std::vector<unsigned int> indices;

  TinyVector3f line_color(0.3, 0.3, 0.3);
  float line_width = 1;
  const int link_pos_id_offset =
      simulation.system->dof() + simulation.system->dof_qd();
  const int square_id = (int)std::sqrt((double)num_total_threads);
  //sim_spacing is the visual distance between independent parallel simulations
  const float sim_spacing = 5.f;
  for (int run = 0; run < 40; ++run) {
    for (int i = 0; i < num_total_threads; ++i) {
      inputs[i] = std::vector<Scalar>(simulation.input_dim(), Scalar(0));
      //quaternion 'w' = 1
      inputs[i][3] = 1;
      //height of Laikago at 0.7 meter
      inputs[i][6] = 0.7;
    }
    for (int t = 0; t < 1000; ++t) {

        positions.resize(0);
        indices.resize(0);

      timer.start();
      // call GPU kernel
      model.forward_zero(&outputs, inputs, 64);
      timer.stop();
      std::cout << "Kernel execution took " << timer.elapsed() << " seconds.\n";

      for (int i = 0; i < num_total_threads; ++i) {
        for (int j = 0; j < simulation.input_dim(); ++j) {
          inputs[i][j] = outputs[i][j];
        }
      }

      sync_counter++;
      
      if (sync_counter > frameskip_gfx_sync) {
          sync_counter = 0;
          if (1) {
              bool manual_sync = false;
              if (manual_sync)
              {
                  //visualizer.sync_visual_transforms(contact_sim.system);
              }
              else
              {
                  float sim_spacing = 2;
                  const int square_id = (int)std::sqrt((double)num_total_threads);
                  int instance_index = 0;
                  int offset = simulation.system->dof() + simulation.system->dof_qd();
                  for (int s = 0; s < num_total_threads; s++)
                  {


                      for (int v = 0; v < num_base_instances; v++)
                      {
                          int visual_instance_id = visual_instances[instance_index++];
                          if (visual_instance_id >= 0)
                          {

                              ::TINY::TinyVector3f pos(outputs[s][4 + 0],
                                  outputs[s][4 + 1],
                                  outputs[s][4 + 2]);
                              ::TINY::TinyQuaternionf orn(outputs[s][0],
                                  outputs[s][1],
                                  outputs[s][2],
                                  outputs[s][3]);

                              pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                              pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                              visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                          }
                      }

                      for (int l = 0; l < simulation.system->links_.size(); l++) {
                          for (int v = 0; v < num_instances[l]; v++)
                          {
                              int visual_instance_id = visual_instances[instance_index++];
                              if (visual_instance_id >= 0)
                              {

                                  ::TINY::TinyVector3f pos(outputs[s][offset + l * 7 + 0],
                                      outputs[s][offset + l * 7 + 1],
                                      outputs[s][offset + l * 7 + 2]);
                                  ::TINY::TinyQuaternionf orn(outputs[s][offset + l * 7 + 3],
                                      outputs[s][offset + l * 7 + 4],
                                      outputs[s][offset + l * 7 + 5],
                                      outputs[s][offset + l * 7 + 6]);

                                  pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                                  pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                                  visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                              }
                          }
    }
  }
              }
          }
          visualizer.render();
          //std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* contact_sim.dt));
      }


    }
  }

  model.forward_zero.deallocate();

  for (const auto& thread : outputs) {
    for (const Scalar& t : thread) {
      std::cout << t << "  ";
    }
    std::cout << std::endl;
  }

  return EXIT_SUCCESS;
}
