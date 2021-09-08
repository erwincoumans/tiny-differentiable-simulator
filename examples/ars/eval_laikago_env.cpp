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

#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "math/tiny/tiny_double_utils.h"
#include "math/tiny/tiny_float_utils.h"
#include "urdf/urdf_cache.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "utils/file_utils.hpp"
#include "../environments/laikago_environment.h"
#include "visualizer/meshcat/meshcat_urdf_visualizer.h"

using namespace TINY;
using namespace tds;

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

typedef TinyVector3<double, DoubleUtils> Vector3;
typedef TinyQuaternion<double, DoubleUtils> Quaternion;

int frameskip_gfx_sync = 1;  // use 60Hz, no need for skipping

bool do_sim = true;

#define USE_LAIKAGO
static MyAlgebra::Vector3 start_pos(0, 0, .48);  // 0.4002847
static MyAlgebra::Quaternion start_orn(0, 0, 0, 1);

static const char* urdf_name = "laikago/laikago_toes_zup.urdf";
static bool is_floating = true;
static double hip_angle = 0.07;
static double knee_angle = -0.59;
static double abduction_angle = 0.2;
static std::vector<double> initial_poses = {
    abduction_angle, hip_angle,       knee_angle,      abduction_angle,
    hip_angle,       knee_angle,      abduction_angle, hip_angle,
    knee_angle,      abduction_angle, hip_angle,       knee_angle,
};

typedef LaikagoEnv<MyAlgebra> Environment;
typedef LaikagoContactSimulation<MyAlgebra> RobotSim;

int main(int argc, char* argv[]) {

  MeshcatUrdfVisualizer<MyAlgebra> meshcat_viz;
  std::cout << "Waiting for meshcat server" << std::endl;
  meshcat_viz.delete_all();


  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  //UrdfParser<MyAlgebra> parser;

  //RobotSim contact_sim(false);

  int input_dim = 36;//contact_sim.input_dim();

  int num_total_threads = 1;
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances;

  Environment env(false);
  //env.contact_sim.

  meshcat_viz.m_path_prefix = env.contact_sim.m_laikago_search_path;
  auto urdf_structures = env.contact_sim.cache.retrieve(env.contact_sim.m_laikago_urdf_filename);
  std::string texture_path = "laikago_tex.jpg";
  meshcat_viz.convert_visuals(urdf_structures, texture_path,env.contact_sim.mb_);

  auto obs = env.reset();
  double total_reward = 0;
  int max_steps = 100000;
  int num_steps = 0;

  int num_params =
      env.neural_network.num_weights() + env.neural_network.num_biases();

  // weights trained using cuda ars_train_policy (without observation filter)
  std::vector<double> x = {
      -0.813725, 0.153640,  0.620345,  -0.762425, -0.290226, -0.207007,
      -0.024241, -0.505319, -0.962317, 0.057327,  -0.376934, 0.779542,
      1.051703,  0.733322,  -0.525843, 0.713521,  -0.028265, -0.381209,
      0.394661,  -0.249793, -0.974984, 0.030375,  -0.942774, -0.115925,
      -0.576964, 0.398923,  -0.861880, -0.589853, -0.365675, -0.709396,
      0.919653,  -0.176127, 0.391715,  -0.665977, 0.343730,  0.354476,
      0.709409,  -0.311141, -0.327950, -0.301298, 0.027099,  0.200675,
      -1.043911, -0.169909, 0.005494,  0.492926,  0.122722,  -0.350802,
      -0.479186, 0.407411,  -0.900445, -0.565946, 0.885628,  -0.703020,
      -0.360806, 0.262098,  0.343010,  -0.474963, -0.181439, -0.203667,
      -0.168397, -0.190174, 0.503648,  0.595199,  0.160639,  0.496430,
      -0.508740, -0.122816, -0.281464, 0.050841,  0.023448,  0.228910,
      -0.783007, 0.795470,  -0.899643, -0.391805, -0.382215, 0.729703,
      0.423185,  1.432758,  0.413287,  0.347354,  -0.002167, -0.264826,
      -0.205144, 0.392787,  -0.368816, 0.414569,  0.116034,  0.335022,
      -0.269509, -0.883381, 0.159323,  -0.152752, 0.382080,  -0.886789,
      -0.605755, 0.600607,  -0.105646, -0.222538, 0.044129,  0.257917,
      -0.254228, -0.022470, -0.456781, 0.306946,  0.524339,  0.041591,
      0.096322,  0.078637,  -0.242363, 0.053335,  -0.907784, -0.199252,
      0.999486,  -0.384917, 0.221773,  -0.318896, 0.066927,  -0.127022,
      -0.110809, 0.959233,  -0.398454, -0.353019, 0.441749,  -0.755026,
      -0.010556, -0.401004, -0.339016, 0.523385,  0.895682,  0.351508,
      0.220067,  -0.093660, 0.709168,  -0.692206, -0.549519, -0.018577,
      0.260414,  -0.135189, -0.278187, 0.660773,  -0.161874, 0.112154,
      0.239513,  -0.250705, 0.499927,  0.577550,  0.746060,  0.085785,
      -0.377340, 0.390667,  -0.367283, -0.207354, -0.253046, -0.124539,
      0.638536,  0.443304,  -0.136848, -0.187253, 0.535331,  -0.093092,
      0.356820,  -0.004701, 0.587100,  -0.006582, -0.847391, 0.326158,
      -0.205371, 0.576662,  0.132890,  0.849326,  -0.805228, 0.008724,
      -1.298032, 0.085774,  -0.150333, -0.199664, 0.791736,  -0.358533,
      0.126289,  0.092361,  -0.550331, -0.041191, -0.775460, 0.591444,
      -0.276290, 0.485064,  0.327098,  -0.063167, -0.191013, 0.991792,
      -0.388944, 0.007153,  -0.257342, 0.223409,  -0.206382, 0.017412,
      -0.026796, 0.174157,  -0.070219, -0.210235, 0.048997,  1.057589,
      -0.547644, -0.104499, 0.227302,  -0.430892, 0.384207,  -0.272557,
      -0.605576, -0.057867, 0.221632,  -0.266406, 0.101537,  -0.233992,
      -0.024956, 0.424360,  0.425195,  -0.608896, -0.584088, 0.127175,
      -0.116302, 0.261905,  0.496137,  -0.316031, 0.285309,  0.669585,
      -0.278237, -0.968214, -0.353483, -0.019190, 0.364158,  0.252649,
      -0.142199, -0.073634, 0.747277,  -0.735856, -0.106237, -0.843556,
      0.158160,  0.274503,  -0.586950, -0.928796, 0.202232,  -0.081048,
      -0.492014, -0.730767, 0.049638,  -0.022691, -0.750751, -0.004012,
      0.448692,  0.066139,  0.836136,  0.038657,  1.056391,  -0.019646,
      0.405053,  -0.510902, 0.277062,  0.223673,  -0.072912, 0.306755,
      -1.191276, 0.467798,  0.084404,  -0.656953, -0.108639, 0.171374,
      -0.394464, -0.302027, -0.174815, 0.497606,  0.135248,  0.008017,
      0.527815,  -0.706005, -0.493687, 0.321805,  0.140150,  0.197086,
      0.731930,  -0.557708, -0.198893, -0.392429, 0.095121,  0.525168,
      -0.458424, -0.053601, 0.881863,  -0.920037, 0.827344,  -1.085381,
      -0.016723, -0.386960, -0.287766, 0.254854,  -0.435951, -1.248529,
      0.157811,  0.400469,  -0.676672, 0.015505,  -0.201277, 0.112097,
      -0.514749, 0.105555,  -0.684231, 0.255833,  -0.218307, -0.347373,
      -0.080596, -0.232471, -0.193975, 0.261194,  -0.310226, -0.136579,
      0.194931,  0.723271,  -0.849442, -0.673646, -0.171552, 0.219744,
      -0.475262, -0.305549, -0.108075, 0.109089,  -0.081203, -0.058609,
      -0.168417, -0.266964, 0.894712,  -0.263505, 0.426195,  0.014186,
      -0.510902, 0.686299,  0.145961,  -0.234306, 0.048043,  0.489235,
      -0.108235, -0.910748, -0.135217, -0.276106, -0.066060, -0.675884,
      -0.351254, -0.583840, 0.631143,  -0.036549, 0.063441,  0.806558,
      -0.860634, 0.354770,  0.338252,  -0.108280, -0.298631, -0.103718,
      0.146253,  -0.322464, 0.171770,  -0.488813, 0.682319,  0.379097,
      0.657373,  -0.656978, -0.444326, 0.847194,  -0.364541, -0.168504,
      0.400713,  0.238195,  -0.405876, 0.262933,  0.056835,  0.167583,
      -0.115137, -0.094221, -0.270780, -0.302817, 0.439980,  0.278107,
      0.811362,  0.215725,  0.946341,  0.253938,  -0.151388, -0.408867,
      0.128733,  -0.341038, -0.041583, 0.118221,  -0.594881, 0.023849,
      -0.313212, 0.237087,  0.436373,  -0.244443, -0.286157, 0.132525,
      0.001805,  0.378492,  -0.177620, 1.077808,  0.100800,  -1.022460,
      -0.582441, -0.883396, -0.320787, -0.021144, -0.664334, -0.142002,
      0.036548,  -0.338788, -0.311965, 1.040848,  -0.130050, -0.164359,
      0.436464,  -0.494073, 0.162583,  -0.286102, -0.298982, -0.834764,
      0.761283,  -0.337144, 0.372722,  0.529962,  0.005756,  -0.161948,
      -0.359033, 0.790885,  -0.483433, 0.955678,  0.203270,  -0.364692,
      -0.000354, 0.665794,  0.711877,  0.071472,  0.536178,  0.837003,
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
      weightsfile_.read((char*)&x[i], sizeof(double));
      printf("%f,", x[i]);
  }
  weightsfile_.close();
#endif

  env.init_neural_network(x);
  bool done = false;
  while (!done && num_steps < max_steps) {
    {
      auto action = env.policy(obs);
      double reward;

      env.step(action, obs, reward, done);
      meshcat_viz.sync_visual_transforms(env.contact_sim.mb_);
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
    }
  }
  return EXIT_SUCCESS;
}
