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

#include <chrono>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <thread>

//#include "meshcat_urdf_visualizer.h"
#include "../opengl_urdf_visualizer.h"

#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#include "urdf/urdf_cache.hpp"
#include "../tiny_visual_instance_generator.h"

using namespace TINY;
using namespace tds;

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

typedef TinyVector3<double, DoubleUtils> Vector3;
typedef TinyQuaternion<double, DoubleUtils> Quaternion;


int frameskip_gfx_sync =1;  //use 60Hz, no need for skipping

bool do_sim = true;


TinyKeyboardCallback prev_keyboard_callback = 0;

void my_keyboard_callback(int keycode, int state)
{
    if (keycode == 's')
        do_sim = state;
    prev_keyboard_callback(keycode, state);
}

//#define USE_LAIKAGO
//#define USE_ANT
#ifdef USE_ANT

#include "../environments/ant_environment.h"
typedef AntEnv<MyAlgebra> Environment;
typedef AntContactSimulation<MyAlgebra> RobotSim;
#else//USE_ANT
#ifdef USE_LAIKAGO
static MyAlgebra::Vector3 start_pos(0,0,.48);//0.4002847
static MyAlgebra::Quaternion start_orn (0,0,0,1);

static std::string urdf_name = "laikago/laikago_toes_zup.urdf";
static bool is_floating = true;
static double hip_angle = 0.07;//0
static double knee_angle = -0.59;//-0.5;
static double abduction_angle = 0.2;
static std::vector<double> initial_poses = {
    abduction_angle, hip_angle, knee_angle, abduction_angle, hip_angle, knee_angle,
    abduction_angle, hip_angle, knee_angle, abduction_angle, hip_angle, knee_angle,
};
#include "../environments/laikago_environment.h"
typedef LaikagoEnv<MyAlgebra> Environment;
typedef LaikagoContactSimulation<MyAlgebra> RobotSim;
#else
#define CARTPOLE
#ifdef CARTPOLE
    #include "../environments/cartpole_environment2.h"
typedef CartpoleEnv<MyAlgebra> Environment;

typedef CartpoleContactSimulation<MyAlgebra> RobotSim;

#else
    #include "../environments/reacher_environment.h"
typedef ReacherEnv<MyAlgebra> Environment;
typedef ReacherContactSimulation<MyAlgebra> RobotSim;
#endif
#endif//USE_LAIKAGO
#endif//USE_ANT


int main(int argc, char* argv[]) {

  int sync_counter = 0;
  int frame = 0;
  World<MyAlgebra> world;
  UrdfParser<MyAlgebra> parser;

  // create graphics
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  
  
  
  visualizer.delete_all();

  RobotSim tmp;
  
  int input_dim = tmp.input_dim();
  if (0)
  {
      std::vector<int> shape_ids;
      std::string plane_filename;
      FileUtils::find_file("plane100.obj", plane_filename);
      TinyVector3f pos(0, 0, 0);
      TinyQuaternionf orn(0, 0, 0, 1);
      TinyVector3f scaling(1, 1, 1);
      visualizer.load_obj(plane_filename, pos, orn, scaling, shape_ids);
  }

  bool create_instances = false;
  char search_path[TINY_MAX_EXE_PATH_LEN];
  std::string texture_path = "";
  std::string file_and_path;
  tds::FileUtils::find_file(tmp.m_urdf_filename, file_and_path);
  auto urdf_structures = tmp.cache.retrieve(file_and_path);
  FileUtils::extract_path(file_and_path.c_str(), search_path,
      TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;
  
  int num_total_threads = 1;
  std::vector<int> visual_instances;
  std::vector<int> num_instances;
  int num_base_instances;
  
  Environment env;
  auto obs = env.reset();
  visualizer.convert_visuals(urdf_structures, texture_path, env.contact_sim.mb_);
  double total_reward = 0;
  int max_steps = 1000;
  int num_steps = 0;

  int num_params = env.neural_network.num_weights() + env.neural_network.num_biases();
 
  //weights trained using c++ ars_train_policy (without observation filter)
#ifdef USE_ANT
  std::vector<double> x = 
{-0.330694,0.100664,-0.050100,0.010913,-0.076245,-0.096708,-0.052048,-0.022072,-0.071742,-0.141120,-0.001460,-0.091223,-0.080160,-0.143914,-0.049341,0.009960,0.073633,0.094207,-0.104590,0.126970,-0.111545,-0.205443,0.164077,0.366699,0.039011,0.047922,0.181835,-0.041931,0.081036,-0.017474,-0.046704,-0.107542,0.009476,0.275584,0.076891,0.027139,0.077219,-0.030522,-0.036556,0.096170,0.044257,0.065861,-0.069107,0.048704,-0.065532,0.101776,0.057067,-0.010219,0.001186,-0.051902,-0.082907,-0.091851,-0.114649,0.055788,-0.103413,-0.334057,-0.047808,0.088291,-0.258178,0.020027,-0.021699,0.064720,0.114032,-0.088063,0.061418,-0.123903,-0.019698,-0.214701,0.057364,0.052798,0.105946,0.013900,-0.124420,0.036956,-0.088813,-0.280787,0.110632,-0.097571,0.042996,0.138702,0.114211,0.040666,0.152882,-0.038852,0.132004,0.159409,0.248187,0.144521,0.133852,-0.098142,0.014450,0.022485,0.044374,-0.175090,0.039559,0.058260,0.178692,-0.140327,-0.019914,0.088317,0.038902,-0.011281,0.050123,-0.107991,-0.038676,-0.084500,0.050126,0.033527,0.202266,-0.070085,0.046814,0.044315,0.082971,0.028167,-0.127337,-0.118247,0.088257,0.028016,-0.031211,0.057919,0.189745,0.213504,0.011288,0.050855,0.012990,0.018937,0.004396,-0.016475,-0.138815,0.086511,-0.140102,-0.106045,0.135993,-0.135477,-0.019576,0.046984,-0.115611,-0.198333,0.094186,0.137661,0.049339,0.077401,-0.162229,0.022853,0.146213,-0.096798,0.136300,-0.024680,0.144730,0.188925,0.025122,0.032732,-0.021309,-0.042702,-0.026105,-0.127947,-0.072268,0.279763,0.010303,0.042321,0.068728,0.037936,-0.061419,-0.202066,-0.128971,-0.127879,-0.048124,-0.037882,0.187363,-0.014169,0.009729,-0.032163,-0.122680,-0.007935,0.122371,0.195222,-0.031082,-0.142409,-0.047097,-0.179822,-0.194071,0.068477,-0.024994,-0.067355,-0.000895,0.060127,0.160594,0.138485,0.155649,-0.077018,-0.035986,0.098026,0.105008,-0.234295,0.107317,0.082829,0.062446,0.084337,0.090816,-0.055368,0.050477,0.054706,-0.146131,-0.150025,0.024232,-0.084176,-0.045031,-0.001584,-0.080642,-0.162535,-0.219500,-0.148941,0.024525,-0.058704,0.126188,-0.152128,0.075867,-0.104105,0.188959,0.049782,-0.173684,-0.041281,0.060444,-0.255056,-0.030986,-0.055905,0.038094,-0.049954,-0.124161,0.230230,0.202641,-0.018996};
#endif

#ifdef USE_LAIKAGO
  std::vector<double> x = {-0.813725,0.153640,0.620345,-0.762425,-0.290226,-0.207007,-0.024241,-0.505319,-0.962317,0.057327,-0.376934,0.779542,1.051703,0.733322,-0.525843,0.713521,-0.028265,-0.381209,0.394661,-0.249793,-0.974984,0.030375,-0.942774,-0.115925,-0.576964,0.398923,-0.861880,-0.589853,-0.365675,-0.709396,0.919653,-0.176127,0.391715,-0.665977,0.343730,0.354476,0.709409,-0.311141,-0.327950,-0.301298,0.027099,0.200675,-1.043911,-0.169909,0.005494,0.492926,0.122722,-0.350802,-0.479186,0.407411,-0.900445,-0.565946,0.885628,-0.703020,-0.360806,0.262098,0.343010,-0.474963,-0.181439,-0.203667,-0.168397,-0.190174,0.503648,0.595199,0.160639,0.496430,-0.508740,-0.122816,-0.281464,0.050841,0.023448,0.228910,-0.783007,0.795470,-0.899643,-0.391805,-0.382215,0.729703,0.423185,1.432758,0.413287,0.347354,-0.002167,-0.264826,-0.205144,0.392787,-0.368816,0.414569,0.116034,0.335022,-0.269509,-0.883381,0.159323,-0.152752,0.382080,-0.886789,-0.605755,0.600607,-0.105646,-0.222538,0.044129,0.257917,-0.254228,-0.022470,-0.456781,0.306946,0.524339,0.041591,0.096322,0.078637,-0.242363,0.053335,-0.907784,-0.199252,0.999486,-0.384917,0.221773,-0.318896,0.066927,-0.127022,-0.110809,0.959233,-0.398454,-0.353019,0.441749,-0.755026,-0.010556,-0.401004,-0.339016,0.523385,0.895682,0.351508,0.220067,-0.093660,0.709168,-0.692206,-0.549519,-0.018577,0.260414,-0.135189,-0.278187,0.660773,-0.161874,0.112154,0.239513,-0.250705,0.499927,0.577550,0.746060,0.085785,-0.377340,0.390667,-0.367283,-0.207354,-0.253046,-0.124539,0.638536,0.443304,-0.136848,-0.187253,0.535331,-0.093092,0.356820,-0.004701,0.587100,-0.006582,-0.847391,0.326158,-0.205371,0.576662,0.132890,0.849326,-0.805228,0.008724,-1.298032,0.085774,-0.150333,-0.199664,0.791736,-0.358533,0.126289,0.092361,-0.550331,-0.041191,-0.775460,0.591444,-0.276290,0.485064,0.327098,-0.063167,-0.191013,0.991792,-0.388944,0.007153,-0.257342,0.223409,-0.206382,0.017412,-0.026796,0.174157,-0.070219,-0.210235,0.048997,1.057589,-0.547644,-0.104499,0.227302,-0.430892,0.384207,-0.272557,-0.605576,-0.057867,0.221632,-0.266406,0.101537,-0.233992,-0.024956,0.424360,0.425195,-0.608896,-0.584088,0.127175,-0.116302,0.261905,0.496137,-0.316031,0.285309,0.669585,-0.278237,-0.968214,-0.353483,-0.019190,0.364158,0.252649,-0.142199,-0.073634,0.747277,-0.735856,-0.106237,-0.843556,0.158160,0.274503,-0.586950,-0.928796,0.202232,-0.081048,-0.492014,-0.730767,0.049638,-0.022691,-0.750751,-0.004012,0.448692,0.066139,0.836136,0.038657,1.056391,-0.019646,0.405053,-0.510902,0.277062,0.223673,-0.072912,0.306755,-1.191276,0.467798,0.084404,-0.656953,-0.108639,0.171374,-0.394464,-0.302027,-0.174815,0.497606,0.135248,0.008017,0.527815,-0.706005,-0.493687,0.321805,0.140150,0.197086,0.731930,-0.557708,-0.198893,-0.392429,0.095121,0.525168,-0.458424,-0.053601,0.881863,-0.920037,0.827344,-1.085381,-0.016723,-0.386960,-0.287766,0.254854,-0.435951,-1.248529,0.157811,0.400469,-0.676672,0.015505,-0.201277,0.112097,-0.514749,0.105555,-0.684231,0.255833,-0.218307,-0.347373,-0.080596,-0.232471,-0.193975,0.261194,-0.310226,-0.136579,0.194931,0.723271,-0.849442,-0.673646,-0.171552,0.219744,-0.475262,-0.305549,-0.108075,0.109089,-0.081203,-0.058609,-0.168417,-0.266964,0.894712,-0.263505,0.426195,0.014186,-0.510902,0.686299,0.145961,-0.234306,0.048043,0.489235,-0.108235,-0.910748,-0.135217,-0.276106,-0.066060,-0.675884,-0.351254,-0.583840,0.631143,-0.036549,0.063441,0.806558,-0.860634,0.354770,0.338252,-0.108280,-0.298631,-0.103718,0.146253,-0.322464,0.171770,-0.488813,0.682319,0.379097,0.657373,-0.656978,-0.444326,0.847194,-0.364541,-0.168504,0.400713,0.238195,-0.405876,0.262933,0.056835,0.167583,-0.115137,-0.094221,-0.270780,-0.302817,0.439980,0.278107,0.811362,0.215725,0.946341,0.253938,-0.151388,-0.408867,0.128733,-0.341038,-0.041583,0.118221,-0.594881,0.023849,-0.313212,0.237087,0.436373,-0.244443,-0.286157,0.132525,0.001805,0.378492,-0.177620,1.077808,0.100800,-1.022460,-0.582441,-0.883396,-0.320787,-0.021144,-0.664334,-0.142002,0.036548,-0.338788,-0.311965,1.040848,-0.130050,-0.164359,0.436464,-0.494073,0.162583,-0.286102,-0.298982,-0.834764,0.761283,-0.337144,0.372722,0.529962,0.005756,-0.161948,-0.359033,0.790885,-0.483433,0.955678,0.203270,-0.364692,-0.000354,0.665794,0.711877,0.071472,0.536178,0.837003,};

#else
#ifdef CARTPOLE
  std::vector<double> x = {0.105781,1.614242,0.716554,1.464621,-0.001607};//0.085644,1.571287,0.743944,1.497057,-0.006282};
#else
  std::vector<double> x(22);
  for (int i=0;i<x.size();i++)
  {
      x[i] = .35*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
  }

#endif
#endif
  env.init_neural_network(x);

  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    {
          auto action = env.policy(obs);
          //std::cout << "state= [" << obs[0] << ", " << obs[1] << ", " << obs[2] << ", " << obs[3] << "]" << std::endl;
          
          double reward;
          bool  done;
          env.step(action,obs,reward,done);
          total_reward += reward;
          num_steps++;
          int num_contacts = 0;
          for (int c=0;c<env.contact_sim.world.mb_contacts_.size();c++)
          {
              for (int j=0;j<env.contact_sim.world.mb_contacts_[c].size();j++)
              {
                if (env.contact_sim.world.mb_contacts_[c][j].distance<0.01)
                {
                    num_contacts++;
                }
              }
          }

          if(done)// || num_steps>=max_steps)
          {
              printf("total_reward=%f\n",total_reward);
              total_reward=0;
              num_steps = 0;
              obs = env.reset();
          }
      }
  
      sync_counter++;
      frame += 1;
      if (sync_counter > frameskip_gfx_sync) 
      {
          sync_counter = 0;
          if (1) {
              bool manual_sync = true;
              if (manual_sync)
              {
                  visualizer.sync_visual_transforms(env.contact_sim.mb_);
              }
          }
          visualizer.render();
          std::this_thread::sleep_for(std::chrono::duration<double>(frameskip_gfx_sync* env.contact_sim.dt));
      }
  }
 
  printf("finished\n");
  return EXIT_SUCCESS;

}
