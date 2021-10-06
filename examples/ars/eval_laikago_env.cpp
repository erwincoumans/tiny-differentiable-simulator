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

  int input_dim = 37;//contact_sim.input_dim();

  int num_total_threads = 1;
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances;

  Environment env(false);
  //env.contact_sim.

  meshcat_viz.m_path_prefix = env.contact_sim.m_laikago_search_path;
  int obs_dim = env.contact_sim.input_dim();
  auto urdf_structures = env.contact_sim.cache.retrieve(env.contact_sim.m_laikago_urdf_filename);
  std::string texture_path = "laikago_tex.jpg";
  meshcat_viz.convert_visuals(urdf_structures, texture_path,env.contact_sim.mb_);

  auto obs = env.reset();
  double total_reward = 0;
  int max_steps = 10000;
  int num_steps = 0;

  int num_params =
      env.neural_network.num_weights() + env.neural_network.num_biases();

  // weights trained using cuda ars_train_policy (without observation filter)
  std::vector<double> x = {
     -1.024009,-1.167130,-0.212025,-0.829941,-0.273994,-0.148460,1.821351,-0.008527,-1.221866,0.195666,0.146996,0.696743,-0.525982,-1.097170,0.490754,-0.241558,1.065449,-0.380558,-1.050630,0.308037,-0.142067,-1.179885,-0.496169,1.016328,-0.130788,-0.043931,-0.653800,-0.773284,-1.009611,-0.248459,-0.175081,0.669306,0.354670,1.079551,-1.193381,-0.270898,-0.158505,0.954343,0.314342,0.941326,0.021687,-0.058339,-0.412108,-0.299077,-0.148040,-1.439724,-0.059109,0.323666,0.732352,0.000947,-0.030328,-0.947481,0.156621,1.061755,-0.398034,-0.384337,-0.952475,0.257197,-0.879942,-1.633425,-0.252664,-0.318770,-1.568318,-1.421026,-0.830230,-0.273782,-2.093512,0.076287,0.550527,-0.187230,-0.586794,0.051669,1.168355,0.906764,-0.677444,1.016172,1.493360,1.091976,0.296339,0.050524,-0.783648,-0.442927,-0.562023,-0.310711,-0.549703,-0.189479,-1.145583,0.768910,-0.440370,0.090838,-0.257266,-0.567644,-0.073137,-1.070842,-1.029749,0.416818,-0.867967,0.007064,0.428393,-0.913034,-0.335825,-1.565874,-0.648598,-0.465493,-0.510110,0.214119,1.062448,-1.298511,0.804992,1.261290,0.584169,1.509129,-0.174545,1.078239,0.238281,-0.994011,-0.609788,-0.471459,0.129356,1.151229,-0.531041,-0.980764,-0.660611,0.049122,-0.567801,1.342441,-0.236984,0.327149,-0.219317,0.509572,-0.488522,-0.750293,0.504004,0.069844,-0.555572,0.149783,-0.217336,0.039563,-0.590840,-0.904153,-0.450911,-0.856312,-0.400225,0.034225,-1.116851,0.247301,-0.166395,0.412200,-0.802118,-0.684978,0.582958,-1.107287,-0.133631,0.056548,0.353572,-1.008300,-0.814079,0.083801,0.740471,-0.891174,-0.349347,-0.229587,0.048187,-0.467476,0.166362,0.741277,-1.065255,0.430241,-0.783233,-0.173187,-0.337646,0.593896,-0.007433,-0.117950,0.434430,0.007546,1.535838,-0.131552,0.843968,0.095349,0.725714,-0.139262,0.454836,-0.005489,-1.911685,1.202686,-0.261819,0.456705,1.095504,0.183617,0.585056,-0.424727,-0.953210,0.074815,-0.458590,0.329668,0.253313,-0.351883,0.004429,1.234280,0.427876,0.908479,0.024525,-0.842219,-0.490234,0.089740,-0.073792,0.727478,1.215637,-0.185624,-0.798466,0.873113,-0.546460,0.094263,0.286351,0.202212,0.257139,1.041244,0.051206,-0.663238,-0.146773,-0.700868,0.212235,-0.899636,-0.860275,0.206659,-0.090233,0.428950,-0.445425,0.736481,-0.233128,0.046280,0.063832,-0.500105,0.438843,0.078293,-1.001570,-0.356427,0.356302,0.592247,-1.413688,-0.708577,-0.720305,1.234820,-0.904009,-0.920797,1.209445,0.309717,-0.622491,-0.649218,-1.400727,0.372739,-0.864222,-0.612138,-0.869983,-0.304616,-1.051172,-0.366344,0.685831,-1.132432,1.625677,-1.520959,-0.837923,-0.166270,1.402440,-0.746718,0.264934,-0.224849,-1.270860,-1.318869,-0.439593,-0.382674,-1.355617,0.274099,-0.369310,0.223229,0.314427,-0.926561,-0.739600,-0.354946,0.040546,-0.955849,1.348909,-0.019784,-2.208257,2.244482,-0.228526,0.659558,-0.580893,-0.660414,-0.513863,-0.581139,0.291775,0.453255,0.478284,0.196670,0.170652,-1.355485,0.888373,-0.132018,-0.327787,0.084208,-0.499657,-0.555718,0.100185,-1.636744,0.594724,-0.600484,-0.693374,0.242877,-1.150809,-0.067117,0.568556,0.504186,-1.077051,0.343385,-1.031525,-0.786424,0.484443,-0.101926,-0.130258,0.858376,-0.638546,0.479734,-0.778824,-0.525962,0.982941,-0.304821,1.939588,0.791862,-1.091768,0.141781,-1.724098,-0.401051,0.941279,0.701412,-0.571245,-0.256577,-0.342780,0.837048,1.358877,1.581108,1.569313,-0.295512,-0.006756,0.395507,-1.060294,-0.894360,-0.636358,-0.649481,0.555612,0.356170,1.622832,0.673341,-0.387073,0.024839,-0.915457,1.045895,-0.709479,-0.966932,0.532160,0.291561,0.426083,-0.756131,0.907425,-0.051521,-0.014702,-0.278753,-0.350683,0.892835,0.930383,-0.845727,-0.961489,0.960855,-0.423828,-0.161644,-0.342394,0.218690,0.108977,0.693223,0.038843,-0.309003,-1.060946,-1.501375,0.168902,0.100711,-0.327109,-0.334881,-0.583961,1.685187,0.375654,0.216059,1.521304,0.570726,0.055949,-0.020855,0.098630,0.719223,-1.232700,0.752166,-0.443604,-0.106655,0.280946,0.406308,0.138565,-2.083566,-0.374718,-0.628707,-1.276878,0.488956,-1.261894,0.086277,0.527915,0.251887,-0.421089,1.167420,0.258068,-0.322602,-1.225852,-0.751379,1.442826,-0.327695,-0.232125,0.456124,0.557179,0.591787,0.918677,-0.003235,-1.202794,0.276321,-1.255111,0.395728,0.098440,-1.266009,-0.555869,0.290608,-0.631436,-0.349103,1.272150,0.628307,-1.343548,0.288847,0.711722,0.145935,-0.626880,0.445570,0.443029,0.205292,0.119290,0.396466,-1.229345,0.438750,-1.078162,-0.469652,0.886576,0.059882,
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
