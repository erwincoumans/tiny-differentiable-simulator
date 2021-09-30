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
     0.458041,2.070008,0.270862,1.306498,4.478700,-6.410442,9.031711,11.934303,7.674623,-3.536691,0.913378,
     -5.927113,-7.771820,9.070834,4.161056,-0.080753,-6.633232,6.336754,-5.287468,2.310071,9.343429,0.992043,
     7.903605,-11.038075,-3.098604,-12.962456,-1.162497,3.414714,3.742757,-0.125928,0.233063,4.040175,-0.868665,
     1.998648,-0.091942,-0.800408,1.726024,8.276379,-1.967211,-11.253289,-8.431901,-5.399317,-2.693226,-1.242170,
     -2.400141,-0.239433,6.339885,-6.376172,5.255800,1.744990,0.562929,1.571790,0.402479,1.201172,-0.908683,
     -2.470389,-4.563102,-4.120259,-0.942447,3.298171,-2.671987,-4.014665,-4.140140,-15.147470,0.899651,2.020193,
     8.155767,8.269188,2.153259,1.772659,-4.809921,-6.406003,-3.762839,-19.391378,-10.900243,-9.594451,9.484920,
     -6.498691,-6.999952,-18.011021,-5.813100,5.290406,0.223977,0.499526,-12.185124,9.790847,-2.158479,9.676104,
     -10.089998,-16.979271,-9.662896,-2.943242,-11.098789,4.207147,-5.695531,-2.604592,2.943639,-2.716637,
     0.479335,2.658358,0.562193,3.669782,-3.234418,1.176110,-0.664265,11.378831,-2.731538,7.846519,-3.076277,
     4.837938,-11.956552,-7.714531,1.735192,-10.264249,-0.320473,-3.647845,1.972655,-9.544622,-1.595872,9.085419,
     -3.573825,7.806150,-11.149002,-11.881908,1.215255,-6.505952,-4.546220,-0.738797,1.829197,5.904194,9.682720,
     4.470366,10.172320,2.011851,11.638770,-7.069491,5.222446,0.492524,-8.740653,-0.717994,-6.244493,-3.947638,
     -4.767520,-0.356582,-9.033671,-9.009300,2.383742,7.077710,1.691833,-2.253669,1.269080,3.137438,0.691552,
     2.693824,-0.596719,-8.743863,5.547626,-7.232573,-5.672297,5.190721,-5.151460,-3.912339,-1.032154,0.272305,
     3.458095,-9.097271,11.706057,-7.155313,10.170068,2.388416,9.374838,-8.176277,1.589774,12.456535,10.178099,
     -4.510388,-0.832159,-15.925839,6.056304,-8.792070,-5.106700,11.680099,-0.262523,-1.368715,-3.443487,-1.172675,
     -8.225317,3.832154,-6.410045,3.893067,-3.406958,2.036697,0.835592,7.095712,-3.353410,16.991497,2.173688,
     4.796676,3.960311,-4.394515,1.519854,2.862861,5.072798,-2.942413,5.188905,-2.672966,2.875544,-5.781124,
     4.427196,-7.158517,7.031158,3.488405,9.189523,1.419650,4.877809,2.094451,-0.012795,2.890223,4.065303,-6.307375,
     0.013700,-5.793078,-4.019966,4.417081,-6.655724,-1.201411,-5.464765,-3.818684,-2.626676,-2.743581,-8.295088,
     4.104602,2.100109,-2.880142,4.468326,0.653752,-8.565589,12.443291,3.427295,-1.841930,-0.328363,-7.043695,
     5.889901,2.686107,-0.142993,-5.000157,0.470242,3.479026,1.161798,2.936382,5.785977,0.941731,10.065658,
     -12.892657,6.907018,-4.375372,10.915538,4.330964,-4.255975,1.199342,8.011836,2.593882,10.256890,12.865449,
     3.782358,3.208323,-0.340778,1.980584,-0.376130,-7.007971,3.109487,-11.135583,-4.434865,-3.095873,-11.884381,
     0.322204,3.509161,-10.093325,5.693438,-2.076689,-3.355952,-0.453092,5.865201,4.601952,-0.508892,0.303919,
     -0.691385,-0.553578,-4.711376,-0.028610,-3.063287,-5.830410,4.105757,3.757363,-2.910865,-0.616987,-10.963447,
     4.054583,5.579885,2.666036,0.078567,-13.679192,-4.101527,6.543482,0.074422,-6.952762,3.749843,-4.617672,
     4.499698,-0.392219,-0.888082,7.077960,1.416385,8.518417,-8.925066,-4.251548,-7.976224,-5.591285,-0.064582,
     -3.881281,5.709751,6.288455,1.833189,-1.270009,-9.029793,2.578891,-1.486369,3.432068,-0.591255,0.965944,-1.095231,
     -3.069617,-12.280809,9.659522,-2.053242,-10.541735,-6.065851,6.743480,0.162069,2.598073,9.359251,1.799114,-0.845055,
     -2.658487,0.600131,-2.069429,-7.303693,1.725125,-1.842158,2.001975,-4.364905,5.233257,11.247454,-7.493869,10.827015,
     -1.334650,6.201274,-8.278918,-2.294373,-3.027058,3.156759,5.829819,-5.809123,-11.760243,4.251871,2.399649,-0.626871,
     -2.219638,0.613825,-15.081893,4.523343,-0.657107,6.333433,-10.058569,7.812755,5.895125,-1.339429,2.003990,-6.447964,
     3.938857,3.967478,4.831011,5.911600,0.520466,1.077409,9.098753,-3.624422,3.232649,5.321274,-1.473093,7.898219,5.573065,
     8.440591,2.834907,6.544420,-3.263338,4.676705,-5.153724,2.397868,3.406128,-7.021982,9.791544,-1.464577,-7.607658,
     -3.209544,-7.039963,-1.176062,-0.868801,0.238867,-0.677137,4.827540,-3.626641,1.991321,2.912964,14.939918,6.642797,
     -3.122433,2.503605,3.230850,-0.914946,12.292402,-6.340817,-11.269959,-4.899491,-5.403105,0.741577,5.494512,-2.408425,
     -2.864787,2.528600,-1.117759,-9.398969,-1.894177,3.419286,1.157970,-1.399393,6.194003,-11.188761,2.297355,-8.521357,
     3.478827,-6.758170,-12.057520,16.190165,8.789776,-5.090088,-3.048916,6.026313,-3.020608,-4.630852,3.043641,-4.166497,
     4.227156,-3.163384,2.834442,3.417710,
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
