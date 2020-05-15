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

#include "tiny_double_utils.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/PhysicsDirectC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "tiny_multi_body.h"
#include "tiny_world.h"
#include "Utils/b3Clock.h"
#include "examples/visualizer_api.h"
#include "base/init_google.h"
#include "devtools/build/runtime/get_runfiles_dir.h"
#include "examples/xarm.h"
#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"
#include "third_party/absl/flags/flag.h"
#include "urdf_import.h"

namespace {

class UrdfTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

typedef TinyMatrixXxX<double, DoubleUtils> TinyMatrixXxX;

void check_mass_matrix(const std::string& urdf_filename,
                       const TinyMatrixXxX& true_mass_matrix,
                       bool is_floating = false,
                       std::vector<double> q = std::vector<double>(),
                       double tolerance = 1e-5) {
  printf("Testing mass matrix for %s...\n", urdf_filename.c_str());
  const std::string search_path = ::devtools_build::GetDataDependencyFilepath(
      "google3/third_party/bullet/examples/pybullet/gym/pybullet_data");

  printf("search_path=%s\n", search_path.c_str());
  VisualizerAPI* sim = new VisualizerAPI();
  sim->connect(eCONNECT_DIRECT);
  sim->setAdditionalSearchPath(search_path.c_str());

  TinyWorld<double, DoubleUtils> world;
  TinyMultiBody<double, DoubleUtils>* mb = world.create_multi_body();

  int pendulumId = sim->loadURDF(urdf_filename);
  // init_double_pendulum<double, DoubleUtils>(*mb, world,5);
  TinyUrdfEditor<double, DoubleUtils> ed;
  // ed.initialize_tiny_multibody(*mb,world,pendulumId, *sim);
  ed.extract_urdf_structs(pendulumId, *sim);

  ed.convert_to_multi_body(world, *mb, pendulumId, *sim);
  mb->m_isFloating = is_floating;
  mb->initialize();

  sim->disconnect();
  delete sim;

  printf("DOF in qd: %i\n", mb->dof_qd());

  true_mass_matrix.print("true_mass_matrix");

  TinyMatrixXxX our_mass_matrix(mb->dof_qd(), mb->dof_qd());
  if (q.empty())
    mb->mass_matrix(&our_mass_matrix);
  else
    mb->mass_matrix(q, &our_mass_matrix);

  our_mass_matrix.print("our_mass_matrix");

  for (int i = 0; i < mb->dof_qd(); ++i) {
    auto diff = (our_mass_matrix[i] - true_mass_matrix[i]).length();
    EXPECT_NEAR(abs(diff), 0., tolerance);
  }
}

TEST_F(UrdfTest, TestMassMatrixPendulum5) {
  TinyMatrixXxX mass_matrix(5, 5);
  mass_matrix[0] = std::vector<double>{29.41964000, 21.60864000, 14.04648000,
                                       7.56432000, 2.70216000};
  mass_matrix[1] = std::vector<double>{21.60864000, 16.20864000, 10.80648000,
                                       5.94432000, 2.16216000};
  mass_matrix[2] = std::vector<double>{14.04648000, 10.80648000, 7.56648000,
                                       4.32432000, 1.62216000};
  mass_matrix[3] = std::vector<double>{7.56432000, 5.94432000, 4.32432000,
                                       2.70432000, 1.08216000};
  mass_matrix[4] = std::vector<double>{2.70216000, 2.16216000, 1.62216000,
                                       1.08216000, 0.54216000};
  check_mass_matrix("pendulum5.urdf", mass_matrix);
}

TEST_F(UrdfTest, TestMassMatrixXarm) {
  std::vector<double> q{-0.16229760, -0.78813327, 0.41991399,
                        -0.41973525, 0.42633960,  0.97467484};
  TinyMatrixXxX mass_matrix(6, 6);
  mass_matrix[0] = std::vector<double>{0.08503492,  -0.01164659, 0.00412507,
                                       -0.01163925, -0.00208419, -0.00014355};
  mass_matrix[1] = std::vector<double>{-0.01164659, 0.18590034, 0.07134522,
                                       -0.00232722, 0.01287967, -0.00001630};
  mass_matrix[2] = std::vector<double>{0.00412507,  0.07134522, 0.38704287,
                                       -0.00324145, 0.03251960, -0.00000151};
  mass_matrix[3] = std::vector<double>{-0.01163925, -0.00232722, -0.00324145,
                                       0.00951568,  -0.00026781, 0.00013498};
  mass_matrix[4] = std::vector<double>{-0.00208419, 0.01287967, 0.03251960,
                                       -0.00026781, 0.00984616, 0.00000344};
  mass_matrix[5] = std::vector<double>{-0.00014355, -0.00001630, -0.00000151,
                                       0.00013498,  0.00000344,  0.00015036};
  check_mass_matrix("xarm/xarm6_robot.urdf", mass_matrix, false, q);
}

TEST_F(UrdfTest, TestMassMatrixLaikago) {
  TinyMatrixXxX mass_matrix(18, 18);
  std::vector<double> q{
      0.01770532264342793, 0.32473051473648806, 0.32717222164114856,
      0.8872400755914167,  1.00000000,          2.00000000,
      3.00000000,          -0.43326809,         -0.89231350,
      0.38135881,          -0.61362423,         -0.07643782,
      -0.22857914,         0.24808082,          0.95794003,
      -0.72352225,         -0.60844636,         0.02982312,
      -0.24605128};
  mass_matrix[0] = std::vector<double>{
      0.60363884,  -0.05306196, -0.01033246, 0.00000000,  1.02938513,
      -0.15282672, -0.08527736, 0.00725559,  0.00298599,  0.07994485,
      0.01151118,  -0.00147746, -0.03626127, -0.02193918, -0.00226089,
      0.07661369,  0.01403523,  -0.00080908};
  mass_matrix[1] = std::vector<double>{
      -0.05306196, 1.48016491, -0.02967957, -1.02938513, 0.00000000,
      0.55879673,  0.01417987, 0.10210120,  0.03506235,  0.00651568,
      0.06322841,  0.02719302, 0.03204896,  0.11170013,  0.02612169,
      -0.01823858, 0.09750151, 0.02381043};
  mass_matrix[2] = std::vector<double>{
      -0.01033246, -0.02967957, 1.49155327, 0.15282672,  -0.55879673,
      0.00000000,  -0.06945003, 0.01533777, 0.00845877,  0.02799591,
      -0.01328886, -0.01000246, 0.02457993, -0.03300678, -0.01311908,
      -0.06643305, -0.03844196, -0.00733693};
  mass_matrix[3] = std::vector<double>{
      0.00000000,  -1.02938513, 0.15282672,  25.56700000, 0.00000000,
      0.00000000,  -0.00000130, -0.22709809, -0.05887463, 0.00000167,
      -0.20283639, -0.06273434, -0.00000068, -0.04655913, -0.06019113,
      0.00000158,  -0.19238075, -0.06358603};
  mass_matrix[4] = std::vector<double>{
      1.02938513, 0.00000000,  -0.55879673, 0.00000000, 25.56700000,
      0.00000000, -0.23596664, 0.02479282,  0.01047876, 0.20923957,
      0.04996608, -0.00713516, -0.02767370, 0.04731482, 0.00530205,
      0.20094316, 0.06097939,  -0.00387505};
  mass_matrix[5] = std::vector<double>{
      -0.15282672, 0.55879673,  0.00000000,  0.00000000,  0.00000000,
      25.56700000, -0.03081087, -0.05359486, -0.02265231, -0.05517040,
      0.07093846,  -0.01013143, 0.08035941,  0.18679319,  0.02093012,
      -0.04810985, 0.08753454,  -0.00556372};
  mass_matrix[6] = std::vector<double>{
      -0.08527736, 0.01417987, -0.06945003, -0.00000130, -0.23596664,
      -0.03081087, 0.08071625, -0.00213197, -0.00082050, 0.00000000,
      0.00000000,  0.00000000, 0.00000000,  0.00000000,  0.00000000,
      0.00000000,  0.00000000, 0.00000000};
  mass_matrix[7] = std::vector<double>{
      0.00725559,  0.10210120, 0.01533777, -0.22709809, 0.02479282, -0.05359486,
      -0.00213197, 0.08895704, 0.03082151, 0.00000000,  0.00000000, 0.00000000,
      0.00000000,  0.00000000, 0.00000000, 0.00000000,  0.00000000, 0.00000000};
  mass_matrix[8] = std::vector<double>{
      0.00298599,  0.03506235, 0.00845877, -0.05887463, 0.01047876, -0.02265231,
      -0.00082050, 0.03082151, 0.01478523, 0.00000000,  0.00000000, 0.00000000,
      0.00000000,  0.00000000, 0.00000000, 0.00000000,  0.00000000, 0.00000000};
  mass_matrix[9] = std::vector<double>{
      0.07994485, 0.00651568, 0.02799591, 0.00000167, 0.20923957, -0.05517040,
      0.00000000, 0.00000000, 0.00000000, 0.07817582, 0.00421480, -0.00043575,
      0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000};
  mass_matrix[10] = std::vector<double>{
      0.01151118, 0.06322841, -0.01328886, -0.20283639, 0.04996608, 0.07093846,
      0.00000000, 0.00000000, 0.00000000,  0.00421480,  0.08119023, 0.02693793,
      0.00000000, 0.00000000, 0.00000000,  0.00000000,  0.00000000, 0.00000000};
  mass_matrix[11] = std::vector<double>{
      -0.00147746, 0.02719302, -0.01000246, -0.06273434, -0.00713516,
      -0.01013143, 0.00000000, 0.00000000,  0.00000000,  -0.00043575,
      0.02693793,  0.01478523, 0.00000000,  0.00000000,  0.00000000,
      0.00000000,  0.00000000, 0.00000000};
  mass_matrix[12] = std::vector<double>{
      -0.03626127, 0.03204896, 0.02457993, -0.00000068, -0.02767370,
      0.08035941,  0.00000000, 0.00000000, 0.00000000,  0.00000000,
      0.00000000,  0.00000000, 0.02886450, 0.00809544,  0.00070982,
      0.00000000,  0.00000000, 0.00000000};
  mass_matrix[13] = std::vector<double>{
      -0.02193918, 0.11170013, -0.03300678, -0.04655913, 0.04731482,
      0.18679319,  0.00000000, 0.00000000,  0.00000000,  0.00000000,
      0.00000000,  0.00000000, 0.00809544,  0.06819846,  0.02044221,
      0.00000000,  0.00000000, 0.00000000};
  mass_matrix[14] = std::vector<double>{
      -0.00226089, 0.02612169, -0.01311908, -0.06019113, 0.00530205,
      0.02093012,  0.00000000, 0.00000000,  0.00000000,  0.00000000,
      0.00000000,  0.00000000, 0.00070982,  0.02044221,  0.01478523,
      0.00000000,  0.00000000, 0.00000000};
  mass_matrix[15] = std::vector<double>{
      0.07661369,  -0.01823858, -0.06643305, 0.00000158, 0.20094316,
      -0.04810985, 0.00000000,  0.00000000,  0.00000000, 0.00000000,
      0.00000000,  0.00000000,  0.00000000,  0.00000000, 0.00000000,
      0.07451659,  0.00505231,  -0.00023842};
  mass_matrix[16] = std::vector<double>{
      0.01403523, 0.09750151, -0.03844196, -0.19238075, 0.06097939, 0.08753454,
      0.00000000, 0.00000000, 0.00000000,  0.00000000,  0.00000000, 0.00000000,
      0.00000000, 0.00000000, 0.00000000,  0.00505231,  0.08081594, 0.02675078};
  mass_matrix[17] = std::vector<double>{
      -0.00080908, 0.02381043, -0.00733693, -0.06358603, -0.00387505,
      -0.00556372, 0.00000000, 0.00000000,  0.00000000,  0.00000000,
      0.00000000,  0.00000000, 0.00000000,  0.00000000,  0.00000000,
      -0.00023842, 0.02675078, 0.01478523};

  check_mass_matrix("laikago/laikago_toes_zup.urdf", mass_matrix, true, q);
}

TEST_F(UrdfTest, TestMassMatrixSphere) {
  TinyMatrixXxX mass_matrix(6, 6);
  mass_matrix[0] = std::vector<double>{1.00000000, 0.00000000, 0.00000000,
                                       0.00000000, 0.00000000, 0.00000000};
  mass_matrix[1] = std::vector<double>{0.00000000, 1.00000000, 0.00000000,
                                       0.00000000, 0.00000000, 0.00000000};
  mass_matrix[2] = std::vector<double>{0.00000000, 0.00000000, 1.00000000,
                                       0.00000000, 0.00000000, 0.00000000};
  mass_matrix[3] = std::vector<double>{0.00000000,  0.00000000, 0.00000000,
                                       10.00000000, 0.00000000, 0.00000000};
  mass_matrix[4] = std::vector<double>{0.00000000, 0.00000000,  0.00000000,
                                       0.00000000, 10.00000000, 0.00000000};
  mass_matrix[5] = std::vector<double>{0.00000000, 0.00000000, 0.00000000,
                                       0.00000000, 0.00000000, 10.00000000};

  check_mass_matrix("sphere2.urdf", mass_matrix, true);
}

TEST_F(UrdfTest, TestMassMatrixCube) {
  TinyMatrixXxX mass_matrix(6, 6);
  std::vector<double> q{-0.24799087, 0.33660943, -0.88225726, -0.21637176,
                        -0.11685516, 0.08062473, 1.52019784};

  mass_matrix[0] = std::vector<double>{1.67334000, 0.00000000, 0.00000000,
                                       0.00000000, 0.00000000, 0.00000000};
  mass_matrix[1] = std::vector<double>{0.00000000, 1.67334000, 0.00000000,
                                       0.00000000, 0.00000000, 0.00000000};
  mass_matrix[2] = std::vector<double>{0.00000000, 0.00000000, 1.67334000,
                                       0.00000000, 0.00000000, 0.00000000};
  mass_matrix[3] = std::vector<double>{0.00000000,  0.00000000, 0.00000000,
                                       10.00000000, 0.00000000, 0.00000000};
  mass_matrix[4] = std::vector<double>{0.00000000, 0.00000000,  0.00000000,
                                       0.00000000, 10.00000000, 0.00000000};
  mass_matrix[5] = std::vector<double>{0.00000000, 0.00000000, 0.00000000,
                                       0.00000000, 0.00000000, 10.00000000};

  check_mass_matrix("sphere8cube.urdf", mass_matrix, true, q);
}

}  // namespace
