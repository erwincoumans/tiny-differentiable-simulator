//#define DISABLE_RENDERING
//#define DEBUG_MODEL
// clang-format off
#include "utils/differentiation.hpp"
#include "utils/cuda_codegen.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "urdf/urdf_cache.hpp"
#include "utils/stopwatch.hpp"
#ifndef DISABLE_RENDERING
#include "visualizer/opengl/visualizer.h"
#include "../opengl_urdf_visualizer.h"
#endif
#include "utils/file_utils.hpp"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

// clang-format on
using namespace TINY;

//#define USE_ANT
#ifdef USE_ANT
#include "../environments/ant_environment2.h"
#else
#include "../environments/laikago_environment2.h"
//#include "../environments/laikago_environment.h"
#endif



int main(int argc, char* argv[]) {

    
  // how many threads to run on the GPU
  int num_total_threads = 4096;
  int compile_cuda = 1;
  int use_omp = 0;


  for (int i = 1; i < argc; i++) {
      if (!strcmp(argv[i], "-n") && (i + 1 < argc) && argv[i + 1][0] != '-')
          num_total_threads = atoi(argv[++i]);
      if (!strcmp(argv[i], "-c"))
          compile_cuda = 0;
      //run on GPU
      if (!strcmp(argv[i], "-o"))
          use_omp = 1;
  }

  using Scalar = double;
  typedef ::TINY::DoubleUtils MyTinyConstants;
  typedef TinyAlgebra<double, MyTinyConstants> ScalarAlgebra;
  
  using CGScalar = typename CppAD::cg::CG<Scalar>;
  using Dual = typename CppAD::AD<CGScalar>;

  using DiffAlgebra =
      tds::default_diff_algebra<tds::DIFF_CPPAD_CODEGEN_AUTO, 0, Scalar>::type;

#ifdef USE_ANT
  AntEnv2<DiffAlgebra> diff_env(false);
  AntEnv2<ScalarAlgebra> scalar_env(false);
#else
  LaikagoEnv<DiffAlgebra> diff_env(false);
  LaikagoEnv<ScalarAlgebra> scalar_env(false);
#endif
  
  auto& simulation = diff_env.contact_sim;
  auto& scalar_simulation = scalar_env.contact_sim;

  std::string model_name = "cuda_model_laikago";// + scalar_env.contact_sim.env_name();
  
  // trace function with all zeros as input
  std::vector<Dual> ax(simulation.input_dim_with_action_and_variables(), Dual(0));
  
  

  //height of robot
  ax[2] = 0.48;
  
  CppAD::Independent(ax);
  std::vector<Dual> ay(simulation.output_dim());
  std::cout << "Tracing function for code generation...\n";
  std::vector<Dual> action;
  action.resize(8);
  ay.resize(simulation.output_dim());
  simulation.step_forward_original(ax, ay);


  CppAD::ADFun<CGScalar> tape;
  tape.Dependent(ax, ay);
  tape.optimize();

  tds::Stopwatch timer;
  timer.start();

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
  
  // create model to load shared library
#ifndef DEBUG_MODEL
  //only compile when using CUDA (not OMP)
  if (compile_cuda && (use_omp==0)) {
    p.create_library();
  }
  tds::CudaModel<Scalar>* cuda_model_ptr = 0;
  if (use_omp==0) {
    cuda_model_ptr = new tds::CudaModel<Scalar>(model_name);
  }
  
#endif //DEBUG_MODEL



  std::vector<std::vector<Scalar>> outputs(
      num_total_threads, std::vector<Scalar>(simulation.output_dim()));

  std::vector<std::vector<Scalar>> inputs(num_total_threads);
#ifndef DEBUG_MODEL
  if (use_omp==0) {
    cuda_model_ptr->forward_zero.allocate(num_total_threads);
  }
#endif //DEBUG_MODEL

#ifndef DISABLE_RENDERING
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
  #



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
     
      num_base_instances=0;
      for (int bb=0;bb<urdf_structures.base_links[0].urdf_visual_shapes.size();bb++)
      {
          int uid = urdf_structures.base_links[0].urdf_visual_shapes[bb].visual_shape_uid;
          OpenGLUrdfVisualizer<DiffAlgebra>::TinyVisualLinkInfo& vis_link = visualizer.m_b2vis[uid];
          int instance = -1;
          int num_instances_per_link = 0;
          for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
          {
              int sphere_shape = vis_link.visual_shape_uids[v];
              ::TINY::TinyVector3f color(1, 1, 1);
              //visualizer.m_b2vis
              instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                  sphere_shape, pos, orn, color, scaling, 1., false);
              visual_instances.push_back(instance);
              num_instances_per_link++;
              simulation.mb_->visual_instance_uids().push_back(instance);
          }
          num_base_instances += num_instances_per_link;

      }
      

     for (int i = 0; i < simulation.mb_->num_links(); ++i) {
         
          int num_instances_per_link = 0;
          for (int bb=0;bb<urdf_structures.links[i].urdf_visual_shapes.size();bb++)
          {
              int uid = urdf_structures.links[i].urdf_visual_shapes[bb].visual_shape_uid;
              auto& vis_link = visualizer.m_b2vis[uid];
              int instance = -1;
              
              //num_link_instances.clear();
              for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
              {
                  int sphere_shape = vis_link.visual_shape_uids[v];
                  ::TINY::TinyVector3f color(1, 1, 1);
                  //visualizer.m_b2vis
                  instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
                      sphere_shape, pos, orn, color, scaling, 1., false);
                  visual_instances.push_back(instance);
                  num_instances_per_link++;
                  simulation.mb_->links_[i].visual_instance_uids.push_back(instance);
              }
              
          }
          num_instances.push_back(num_instances_per_link);
          
      }
  }

  visualizer.m_opengl_app.m_renderer->rebuild_graphics_instances();


  std::vector< TinyVector3f> positions;
  std::vector<unsigned int> indices;

  TinyVector3f line_color(0.3, 0.3, 0.3);
  float line_width = 1;
  const int link_pos_id_offset =
      simulation.mb_->dof() + simulation.mb_->dof_qd();
  const int square_id = (int)std::sqrt((double)num_total_threads);
  //sim_spacing is the visual distance between independent parallel simulations
  const float sim_spacing = 5.f;
  for (int i = 0; i < num_total_threads; ++i) {
      inputs[i] = std::vector<Scalar>(simulation.input_dim_with_action_and_variables(), Scalar(0));
  }
  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
#else //DISABLE_RENDERING
  for (int f=0;f<100;f++) {
#endif //DISABLE_RENDERING

    std::vector<Scalar> observations(simulation.output_dim());

    #pragma omp parallel
    #pragma omp for
    for (int i = 0; i < num_total_threads; ++i) {
      scalar_simulation.reset(inputs[i], observations);
    }

    for (int t = 0; t < 1000; ++t) {

#ifndef DISABLE_RENDERING
        positions.resize(0);
        indices.resize(0);
#endif

      std::string platform = "unknown";

      timer.start();
      // call GPU kernel
#ifndef DEBUG_MODEL
      if (use_omp) {
        #pragma omp parallel
        #pragma omp for
          for (int i = 0; i < num_total_threads; ++i) {
            simulation.forward_kernel(1,&outputs[i][0], &inputs[i][0]);
          }
         platform = "OMP";
      } else {
        platform = "CUDA";
        cuda_model_ptr->forward_zero(&outputs, inputs, 64);
      }
#endif //DEBUG_MODEL

      timer.stop();
      
      std::cout << platform << " kernel execution took " << timer.elapsed() << " seconds. (" << double(num_total_threads)/timer.elapsed() << " steps per seconds)\n";

      for (int i = 0; i < num_total_threads; ++i) {
#ifdef DEBUG_MODEL
          for (int xx = 0; xx < simulation.input_dim_with_action(); xx++)
          {
              ax[xx] = inputs[i][xx];
          }
          ay = simulation(ax, action);
          for (int yy = 0; yy < simulation.output_dim(); yy++)
          {
              outputs[i][yy] = DiffAlgebra::to_double(ay[yy]);
          }
#endif //DEBUG_MODEL
          for (int j = 0; j < simulation.input_dim(); ++j) {
            inputs[i][j] = outputs[i][j];
          }
      }

#ifndef DISABLE_RENDERING
      sync_counter++;
      
      if (sync_counter >= frameskip_gfx_sync) {
          sync_counter = 0;
          if (1) {
              bool manual_sync = false;
              if (manual_sync)
              {
                  //visualizer.sync_visual_transforms(contact_sim.mb_);
              }
              else
              {
                  float sim_spacing = 4;
                  const int square_id = (int)std::sqrt((double)num_total_threads);
                  int instance_index = 0;
                  int offset = simulation.mb_->dof() + simulation.mb_->dof_qd();
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

                      int link_index =0;
                      for (int ll = 0; ll < simulation.mb_->links_.size(); ll++) {
                          for (int v = 0; v < num_instances[ll]; v++)
                          {
                              int visual_instance_id = visual_instances[instance_index++];
                              if (visual_instance_id >= 0)
                              {

                                  ::TINY::TinyVector3f pos(outputs[s][offset + link_index * 7 + 0],
                                      outputs[s][offset + link_index * 7 + 1],
                                      outputs[s][offset + link_index * 7 + 2]);
                                  ::TINY::TinyQuaternionf orn(outputs[s][offset + link_index * 7 + 3],
                                      outputs[s][offset + link_index * 7 + 4],
                                      outputs[s][offset + link_index * 7 + 5],
                                      outputs[s][offset + link_index * 7 + 6]);

                                  pos[0] += sim_spacing * (s % square_id) - square_id * sim_spacing / 2;
                                  pos[1] += sim_spacing * (s / square_id) - square_id * sim_spacing / 2;

                                  visualizer.m_opengl_app.m_renderer->write_single_instance_transform_to_cpu(pos, orn, visual_instance_id);
                                  link_index++;
                              }
                          }
                      }
                  }
              }
          }
          visualizer.render();
          //std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* contact_sim.dt));
      }
#endif //DISABLE_RENDERING

    }
  }
#ifndef DEBUG_MODEL
  if (use_omp==0) {
    cuda_model_ptr->forward_zero.deallocate();
    delete cuda_model_ptr;
  }
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
