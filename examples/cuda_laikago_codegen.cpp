#define DEBUG_MODEL
// clang-format off
#include "utils/differentiation.hpp"
//#include "utils/cuda_codegen.hpp"
#include "utils/cuda/cuda_library_processor.hpp"

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
typedef double TinyDualScalar;
typedef double MyScalar;
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"

void omp_model_ant_forward_zero_kernel(int num_total_threads,
                                            double* output,
                                            const double *input);

//void model_ant_forward_zero_kernel(int num_total_threads,
//                                            double *output,
//                                            const double *input);

//void cuda_model_laikago_forward_zero_kernel(int num_total_threads,
//                                            double *output,
//                                            const double  *input);

typedef TinyAlgebra<double, TINY::DoubleUtils> MyAlgebra;

#include "environments/ant_environment2.h"
//#include "environments/laikago_environment.h"
#include "math/matrix_utils.hpp"

//#define LaikagoSimulation LaikagoContactSimulation
#define system mb_
using namespace TINY;

int main(int argc, char* argv[]) {

  // how many threads to run on the GPU
  int num_total_threads = 16;
  int compile_cuda = 1;

  for (int i = 1; i < argc; i++) {
      if (!strcmp(argv[i], "-n") && (i + 1 < argc) && argv[i + 1][0] != '-')
          num_total_threads = atoi(argv[++i]);
      if (!strcmp(argv[i], "-c"))
          compile_cuda = 0;
  }

  using Scalar = double;
  using CGScalar = typename CppAD::cg::CG<Scalar>;
  using Dual = typename CppAD::AD<CGScalar>;

  using DiffAlgebra =
      tds::default_diff_algebra<tds::DIFF_CPPAD_CODEGEN_AUTO, 0, Scalar>::type;

  //AntContactSimulation<DiffAlgebra> simulation;
  //  AntContactSimulation<MyAlgebra> sim;
  
  AntContactSimulation<DiffAlgebra> simulation(true,"gym/ant_org_xyz_xyzrot.urdf","",get_initial_poses<Dual>(), false);
  AntContactSimulation<MyAlgebra> sim(true,"gym/ant_org_xyz_xyzrot.urdf","",get_initial_poses<Scalar>(), false);
  // 
  // 
  //LocomotionContactSimulation<DiffAlgebra> simulation(true,"laikago/laikago_toes_zup_xyz_xyzrot.urdf",laikago_toes_zup_urdf,get_initial_poses<Dual>(), false);
  //LocomotionContactSimulation<MyAlgebra> sim(true,"laikago/laikago_toes_zup_xyz_xyzrot.urdf",laikago_toes_zup_urdf,get_initial_poses<Scalar>(), false);

  // trace function with all zeros as input
  std::vector<Dual> ax(simulation.input_dim_with_action_and_variables(), Dual(0));
  if (simulation.mb_->is_floating())
  {
      //quaternion 'w' = 1
      ax[3] = 1;
      //height of Laikago at 0.7 meter
      ax[6] = 1;
  } else
  {
      //height of Laikago at 0.7 meter
      ax[2] = 1;
      //quaternion 'w' = 1
      ax[3] = 0.706825;
      ax[6] = 0.706825;//1;
  }

  
    for (int l = 0; l < simulation.mb_->links_.size(); l++) {
    switch (simulation.mb_->links_[l].joint_type) {
    case tds::JOINT_FIXED:
        {
            break;
        }
        case tds::JOINT_SPHERICAL:
        {
            int q_offset = simulation.mb_->links()[l].q_index;
            ax[q_offset++] = DiffAlgebra::zero();
            ax[q_offset++] = DiffAlgebra::zero();
            ax[q_offset++] = DiffAlgebra::zero();
            ax[q_offset++] = DiffAlgebra::one();
            break;
        }
        default:
        {
        }
    }
  }
  CppAD::Independent(ax);
  std::vector<Dual> ay(simulation.output_dim());
  std::cout << "Tracing function for code generation...\n";
  ay = simulation.step_forward1(ax);

  CppAD::ADFun<CGScalar> tape;
  tape.Dependent(ax, ay);
  tape.optimize();

  tds::Stopwatch timer;
  timer.start();
  std::string model_name = "cuda_model_laikago";
  //tds::CudaSourceGen<Scalar> cgen(tape, model_name);
  tds::CudaModelSourceGen<Scalar> cgen(tape, model_name);

  cgen.setCreateForwardZero(true);
  //cgen.setCreateJacobian(true);
  //cgen.setCreateSparseJacobian(true);

  //tds::CudaLibraryProcessor p(&cgen);
  tds::CudaLibraryProcessor2 p(&cgen);


#if CPPAD_CG_SYSTEM_WIN
  std::string nvcc_path = tds::exec("where nvcc");
#else
  std::string nvcc_path = tds::exec("which nvcc");
#endif
  std::cout << "Using [" << nvcc_path << "]" << std::endl;
  p.nvcc_path() = nvcc_path;
  //p.generate_code();
  //p.save_sources();

#if 0
  // create model to load shared library
#ifndef DEBUG_MODEL
  //if not compile_cuda, re-use previously build CUDA shared library
  if (compile_cuda)
  {
    //p.create_library();
  }
  //tds::CudaModel<Scalar> model(model_name);
  #endif
#endif //DEBUG_MODEL




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
  tds::FileUtils::find_file(simulation.urdf_filename_, file_and_path);
  auto urdf_structures = simulation.cache.retrieve(file_and_path);// contact_sim.m_urdf_filename);
  tds::FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  visualizer.convert_visuals(urdf_structures, texture_path);


  
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances = 0;
  int sync_counter = 0;


  int frameskip_gfx_sync = 16;
  
  for (int t = 0; t < num_total_threads; t++)
  {
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      int num_instances_per_link = 0;
      if (urdf_structures.base_links.size() && urdf_structures.base_links[0].urdf_visual_shapes.size())
      {
          int uid = urdf_structures.base_links[0].urdf_visual_shapes[0].visual_shape_uid;
          OpenGLUrdfVisualizer<DiffAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
          int instance = -1;
          for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
          {
              int sphere_shape = vis_link.visual_shape_uids[v];
              ::TINY::TinyVector3f color(1, 1, 1);
              //visualizer.m_b2vis
              instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                  sphere_shape, pos, orn, color, scaling,1, false);
              visual_instances.push_back(instance);
              num_instances_per_link++;
              simulation.system->visual_instance_uids().push_back(instance);
          }
      }
      num_base_instances = num_instances_per_link;

      for (int i = 0; i < simulation.system->num_links(); ++i) {

          num_instances_per_link = 0;

          if (urdf_structures.links[i].urdf_visual_shapes.size())
          {
              int uid = urdf_structures.links[i].urdf_visual_shapes[0].visual_shape_uid;
              OpenGLUrdfVisualizer<DiffAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
              int instance = -1;
              
              for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
              {
                  int sphere_shape = vis_link.visual_shape_uids[v];
                  ::TINY::TinyVector3f color(1, 1, 1);
                  //visualizer.m_b2vis
                  instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                      sphere_shape, pos, orn, color, scaling, 1, false);
                  visual_instances.push_back(instance);
                  num_instances_per_link++;

                  simulation.system->links_[i].visual_instance_uids.push_back(instance);
              }
          }
          num_instances.push_back(num_instances_per_link);
      }
  }

  printf("rebuild_graphics_instances\n"); 
  visualizer.m_opengl_app.m_renderer->rebuild_graphics_instances();
  printf("rebuild_graphics_instances done\n");

#ifndef DEBUG_MODEL
  //model.forward_zero.allocate(num_total_threads);
#endif //DEBUG_MODEL

  std::vector< TinyVector3f> positions;
  std::vector<unsigned int> indices;

  TinyVector3f line_color(0.3, 0.3, 0.3);
  float line_width = 1;
  const int link_pos_id_offset =
      simulation.system->dof() + simulation.system->dof_qd();
  const int square_id = (int)std::sqrt((double)num_total_threads);
  //sim_spacing is the visual distance between independent parallel simulations
  const float sim_spacing = 5.f;
  
  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    for (int i = 0; i < num_total_threads; ++i) {
      inputs[i] = std::vector<Scalar>(simulation.input_dim_with_action_and_variables(), Scalar(0));
      if (simulation.mb_->is_floating())
      {
          inputs[i][0] = 0;
          inputs[i][1] = 0;
          inputs[i][2] = 0;
          inputs[i][3] = 1.;
      
          inputs[i][4] = 0;

          inputs[i][5] = 0;
          inputs[i][6] = 0.5;//z-height
      } else
      {
        inputs[i][0] = 0.;
        inputs[i][1] = 0.;
        inputs[i][2] = 0.48;//z-height
        inputs[i][3] = 0.;
        //inputs[i][3] = 	0.706825;
        inputs[i][4] = 0;
        inputs[i][5] = 0;
        //inputs[i][6] = 0.706825;//1;

      }
      
        int start_link = simulation.mb_->is_floating() ? 0 : 6;
        int index = 0;
        for (int l = start_link; l < simulation.mb_->links_.size(); l++) {
        switch (simulation.mb_->links_[l].joint_type) {
        case tds::JOINT_FIXED:
            {
                break;
            }
            case tds::JOINT_SPHERICAL:
            {
                int q_offset = simulation.mb_->links()[l].q_index;
                inputs[i][q_offset++] = 0;
                inputs[i][q_offset++] = 0;
                inputs[i][q_offset++] = 0;
                inputs[i][q_offset++] = 1;
                break;
            }
            default:
            {
                int q_offset = simulation.mb_->links()[l].q_index;
                inputs[i][q_offset] = get_initial_poses<Scalar>()[index++]+0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
            }
        }
        inputs[i][simulation.mb_->dof() + simulation.mb_->dof_qd() + simulation.action_dim() + 0] = 15.;//100.;
        inputs[i][simulation.mb_->dof() + simulation.mb_->dof_qd() + simulation.action_dim() + 1] = 0.3;//2.;
        inputs[i][simulation.mb_->dof() + simulation.mb_->dof_qd() + simulation.action_dim() + 2] = 3;//50.;
        }
    }
    
    for (int t = 0; t < 1000; ++t) {

        positions.resize(0);
        indices.resize(0);

      
#ifndef DEBUG_MODEL1
      timer.start();
      // call GPU kernel
//#define USE_CUDA
#ifdef USE_CUDA
      model.forward_zero(&outputs, inputs, 64);
#else//USE_CUDA
      //#pragma omp parallel 
      //#pragma omp for 
      for (int n=0;n<num_total_threads;n++) {
        //outputs[n] = sim(inputs[n]);
        outputs[n] = sim.step_forward1(inputs[n]);

        //omp_model_ant_forward_zero_kernel
        //omp_model_ant_forward_zero_kernel(1,&outputs[n][0], &inputs[n][0]);
        //model_ant_forward_zero_kernel(1,&outputs[n][0], &inputs[n][0]);
      }
#endif//USE_CUDA
      timer.stop();
      std::cout << "Kernel execution took " << timer.elapsed() << " seconds.\n";
#endif //DEBUG_MODEL

      

      for (int i = 0; i < num_total_threads; ++i) {
#ifdef DEBUG_MODEL1
          for (int xx = 0; xx < simulation.input_dim(); xx++)
          {
              ax[xx] = inputs[i][xx];
          }
          ay = simulation.step_forward1(ax);

          for (int yy = 0; yy < simulation.output_dim(); yy++)
          {
              outputs[i][yy] = DiffAlgebra::to_double(ay[yy]);
          }
#endif //DEBUG_MODEL
          for (int j = 0; j < simulation.input_dim(); ++j) {
            inputs[i][j] = outputs[i][j];
          }
      }
      //printf("sync_counter\n");
      sync_counter++;
      
      if (sync_counter >= frameskip_gfx_sync) {
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
                          //printf("num_base_instances\n");
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
                      int active_instance_index=0;

                      for (int l = 0; l < simulation.system->links_.size(); l++) {
                          for (int v = 0; v < num_instances[l]; v++)
                          {
                              
                              int visual_instance_id = visual_instances[instance_index++];
                              if (visual_instance_id >= 0)
                              {

                                  ::TINY::TinyVector3f pos(outputs[s][offset + active_instance_index * 7 + 0],
                                      outputs[s][offset + active_instance_index * 7 + 1],
                                      outputs[s][offset + active_instance_index * 7 + 2]);
                                  ::TINY::TinyQuaternionf orn(outputs[s][offset + active_instance_index * 7 + 3],
                                      outputs[s][offset + active_instance_index * 7 + 4],
                                      outputs[s][offset + active_instance_index * 7 + 5],
                                      outputs[s][offset + active_instance_index * 7 + 6]);

                                  pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                                  pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                                  visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                                  active_instance_index++;
                              }
                          }
    }
  }
              }
          }
          visualizer.render();
          std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* 0.001));
      }


    }
  }
#ifndef DEBUG_MODEL
  //model.forward_zero.deallocate();
#endif //DEBUG_MODEL

#if 0
  for (const auto& thread : outputs) {
    for (const Scalar& t : thread) {
      std::cout << t << "  ";
    }
    std::cout << std::endl;
  }
#endif

  return EXIT_SUCCESS;
}
