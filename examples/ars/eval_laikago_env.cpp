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

#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <chrono>
#include <thread>

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
//#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_float_utils.h"
#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "utils/file_utils.hpp"

#ifdef USE_MESHCAT
#include "visualizer/meshcat/meshcat_urdf_visualizer.h"
#else
#include "../opengl_urdf_visualizer.h"
#endif

#include "../environments/laikago_environment2.h"
//#include "../environments/ant_environment.h"

using namespace TINY;
using namespace tds;

typedef float TinyDualScalar;
typedef float MyScalar;
typedef ::TINY::FloatUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
//typedef TinyAlgebra<float, MyTinyConstants> MyAlgebra;
typedef TinyAlgebra<float, MyTinyConstants> MyAlgebra;

//typedef TinyVector3<float, DoubleUtils> Vector3;
//typedef TinyQuaternion<float, DoubleUtils> Quaternion;

int frameskip_gfx_sync = 8;  // use 60Hz, no need for skipping

bool do_sim = true;


typedef LaikagoEnv<MyAlgebra> Environment;
typedef LocomotionContactSimulation<MyAlgebra, LAIKAGO_VARIABLE_SIZE> RobotSim;

//typedef AntEnv<MyAlgebra> Environment;
//typedef AntContactSimulation<MyAlgebra> RobotSim;


int main(int argc, char* argv[]) {
#ifdef USE_MESHCAT
  MeshcatUrdfVisualizer<MyAlgebra> visualizer;
#else
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
#endif
  std::cout << "Waiting for meshcat server" << std::endl;
  visualizer.delete_all();

  int frame = 0;
  
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances;

  Environment env(true);
  
  visualizer.m_path_prefix = env.contact_sim.urdf_full_path_;
  int obs_dim = env.contact_sim.input_dim();
  auto urdf_structures = env.contact_sim.cache.retrieve(env.contact_sim.urdf_filename_);
  std::string texture_path = "laikago_tex.jpg";
  visualizer.convert_visuals(urdf_structures, texture_path,env.contact_sim.mb_);
  
  

  auto obs = env.reset();
  float total_reward = 0;
  int max_steps = 10000;
  int num_steps = 0;

  int num_params =
      env.neural_network.num_weights() + env.neural_network.num_biases();

  // weights trained using cuda ars_train_policy (without observation filter)
  std::vector<MyScalar> x = {
     0.341436,-0.382980,1.473858,-0.100947,0.337707,-3.214974,-0.271588,1.496463,0.347998,-0.182418,-0.890516,1.619615,-1.804447,2.339298,-3.198182,0.209089,-0.220839,-2.033201,0.427946,2.047448,-1.105520,-0.111540,1.369103,-0.125971,-1.623144,0.682733,-0.025228,-3.676054,2.436119,0.853863,0.572019,0.377398,1.288971,-3.797144,-0.252711,-3.133911,1.047999,1.637581,-1.796407,0.517812,-1.293242,2.396125,-0.241165,-0.579746,2.381659,-1.518990,0.576045,0.845260,-0.506156,-0.975242,-0.457697,-1.289399,-2.109809,-0.107872,1.365424,-1.750132,-1.692323,-1.090481,-0.621535,0.236616,-0.110473,2.197656,-1.018896,4.599750,-1.445607,-0.437806,-1.406687,-0.545866,0.747575,0.959466,1.123795,2.435696,-0.163912,1.157814,-1.365901,0.274158,1.213548,-2.492444,2.140807,-0.528972,-1.377189,-0.007434,2.589191,-1.640965,1.405806,-1.017995,-2.850694,-0.359213,3.145937,-1.778374,1.723817,0.850079,2.668517,1.079442,3.388372,-0.734462,-0.155193,0.137627,-2.444018,1.494330,-0.624057,0.095275,-4.294688,0.170894,1.261364,1.656367,2.708757,0.524835,-0.960594,0.088406,1.392671,-2.674084,-2.409675,-0.475161,-0.043124,-1.988352,1.864763,-2.885875,-0.500966,-3.715675,-1.444805,0.558530,3.010658,-2.821033,-0.439848,0.652537,-2.027986,-1.384816,-0.442393,-2.476037,1.719759,-3.313050,0.996906,0.614519,0.103837,1.070068,-1.070471,-2.545218,-0.239586,1.996992,-0.654927,4.628826,-2.592264,0.150619,2.521815,-1.299341,-2.549808,0.673026,1.118770,-3.069061,-0.135503,0.096565,1.181648,1.390011,-1.922187,1.158191,-0.529193,0.470237,0.308073,1.065719,-0.532514,2.448745,-0.410294,2.639646,0.391523,1.826551,1.084658,-0.939162,0.980140,-0.793784,-0.066102,-0.766239,-0.225238,1.312386,1.591630,0.530542,0.300750,-0.761904,-0.669636,0.080775,1.028512,-0.875389,-0.157074,1.037277,0.262421,-0.073994,1.230152,-1.335279,2.707968,2.767648,-0.820395,0.536879,1.190015,1.251261,1.210581,0.568941,-1.591061,3.213564,-0.330560,-0.196432,0.951629,0.086824,0.362154,-2.531032,2.945787,0.347018,-1.567721,-2.089512,1.444064,-0.829190,-0.416205,-2.316878,-0.455070,-0.096060,-0.806708,-0.260170,0.697894,2.458612,-1.613032,-1.328343,-1.703035,1.101839,-0.390114,1.067343,1.459356,-0.955866,2.199086,3.773115,0.141223,-1.961227,0.461975,-0.197365,-1.415224,0.811403,-1.439768,-2.226343,0.484502,0.498078,-2.420139,1.367388,0.981426,-0.153652,-0.060196,-0.033531,-0.144590,0.467919,-2.629455,-1.966825,-0.436422,0.128379,0.530077,0.484659,2.241254,0.622061,2.252414,2.236139,-1.204385,-1.084889,0.038133,-1.866936,-1.493360,2.887547,-2.684584,-0.200430,0.176809,-1.642326,-1.530538,1.111308,-1.232897,-0.155513,2.691931,-2.455890,-0.379389,0.038242,1.974106,-0.769625,1.161304,0.294405,0.704215,-0.558288,-0.140963,-0.757363,0.786246,-2.785134,-0.708384,0.672917,0.309603,-0.303173,-1.951525,0.847966,-1.197357,-0.460295,-0.318921,-3.510597,0.268738,-0.421650,0.015463,1.675172,0.093180,-1.024773,-0.584287,4.298994,0.391455,-1.713227,-0.302529,0.572000,-0.036302,-3.037589,-2.831731,2.010224,-0.107265,1.141096,2.377263,-2.727115,-0.479632,-0.373716,-1.289849,-0.036594,-2.119634,-1.145639,0.249792,0.819643,0.247665,1.022145,-2.363124,1.794220,-1.295929,1.464450,0.000050,-1.457365,1.998757,-0.136696,0.764067,-2.053903,-2.605561,-1.394385,-0.330102,-0.587938,1.746063,-2.402911,2.843604,0.087656,-2.116997,0.659310,1.494353,0.611523,-0.340088,1.202859,1.022631,-1.046155,3.806324,2.169945,0.303889,2.250443,-0.648843,1.255763,-0.977780,-3.467736,-0.171379,0.072903,-2.061870,-1.608999,0.952997,1.520506,2.183207,0.667712,-0.209520,0.832441,-0.859119,1.120183,2.360721,-0.532222,2.057221,-0.474716,0.509019,0.390797,0.207184,-0.130267,-0.087830,-0.730706,-0.209076,-1.376516,-2.700442,-2.228430,1.417947,-1.007177,-0.215530,1.802210,-2.689386,-4.052407,1.114279,1.955401,-0.431302,-0.487472,-0.704123,-0.050940,-0.036050,-0.645731,-2.475331,1.183290,1.406323,-1.755209,2.675263,0.074638,2.255910,-0.814171,1.466480,1.412941,1.485928,1.983024,1.855624,-1.712752,-1.585367,-2.427288,0.682515,0.730792,-1.285732,-0.403423,-2.328921,2.109967,0.601918,0.330208,2.663227,2.921822,-2.542434,-1.803643,-0.072492,0.754258,-0.401320,-0.565745,0.945094,-1.094871,0.176833,-2.590063,1.455718,4.012594,0.082255,0.068777,-0.385901,3.741211,2.073043,1.666235,0.642738,0.099792,
  };

#if 0
  std::ifstream weightsfile_;
  std::string fileName = "d:/ant_weights_5044.074185.bin";
  weightsfile_.open (fileName, std::ios_base::binary);
  int num_weights = 0;
  weightsfile_.read((char*)&num_weights, sizeof(int));
  x.resize(num_weights);
  for (int i=0; i < num_weights; i++)
  {
      weightsfile_.read((char*)&x[i], sizeof(float));
      printf("%f,", x[i]);
  }
  weightsfile_.close();
#endif

  env.init_neural_network(x);
  bool done = false;
  env.contact_sim.dt = 1e-3;
  
  while (!done && num_steps < max_steps) {
    {
      auto action = env.policy(obs);
      MyScalar reward;

      env.step(action, obs, reward, done);
      
      total_reward += reward;
      num_steps++;
      int num_contacts = 0;
      for (int c = 0; c < env.contact_sim.world.mb_contacts_.size(); c++) {
        for (int j = 0; j < env.contact_sim.world.mb_contacts_[c].size(); j++) {
          if (env.contact_sim.world.mb_contacts_[c][j].distance < 0.01) {
            num_contacts++;
          }
        }
      }

      if (done || num_steps >= max_steps) {
        printf("total_reward=%f\n", total_reward);
        total_reward = 0;
        num_steps = 0;
        obs = env.reset();
        break;
      }

      if (frame++>=frameskip_gfx_sync) {
          frame = 0;
          
#ifdef USE_MESHCAT
          visualizer.sync_visual_transforms2(env.contact_sim.mb_,env.sim_state_with_graphics, env.contact_sim.input_dim());
#else
          visualizer.sync_visual_transforms(env.contact_sim.mb_);
#endif
          std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* env.contact_sim.dt));
          visualizer.render();
      }
    }
  }
  return EXIT_SUCCESS;
}
