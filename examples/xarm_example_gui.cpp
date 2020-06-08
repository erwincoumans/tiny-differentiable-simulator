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

#include <stdio.h>
#include "xarm.h"

#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3Common/b3Quaternion.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btQuaternion.h"
#include "SharedMemory/PhysicsClientC_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "assert.h"
#define _USE_MATH_DEFINES 1
#include "math.h"

#include "fix64_scalar.h"
#include "tiny_matrix3x3.h"
#include "tiny_multi_body.h"
#include "tiny_pose.h"
#include "tiny_quaternion.h"
#include "tiny_vector3.h"

struct DoubleUtils {
  static double zero() { return 0.; }
  static double one() { return 1.; }
  static double two() { return 2.; }
  static double half() { return 0.5; }
  static double pi() { return M_PI; }
  static double half_pi() { return M_PI / 2.; }
  static double cos1(double v) { return ::cos(v); }
  static double sin1(double v) { return ::sin(v); }
  template <class T>
  static T sqrt1(T v) {
    return sqrt(v);
  }

  template <class T>
  static double getDouble(T v) {
    return (double)v;
  }

  template <class T>
  static double convert(T) = delete;  // C++11

  static double convert(int value) { return double(value); }

  template <class T>
  static double fraction(T, T) = delete;  // C++11

  static double fraction(int num, int denom) {
    return double(num) / double(denom);
  }

  static void FullAssert(bool a) {
    if (!a) {
      printf("!");
      assert(0);
      exit(0);
    }
  }
};

typedef TinyVector3<double, DoubleUtils> dvec3;
typedef TinyVector3<Fix64Scalar, Fix64Scalar> fvec;
typedef TinyMatrix3x3<double, DoubleUtils> dmat3;

template <typename TinyScalar, typename TinyConstants>
void xarm6_fk(class b3RobotSimulatorClientAPI_NoDirect& visualizer,
              const std::vector<int>& visual_links,
              const std::vector<int>& param_uids,
              TinyMultiBody<TinyScalar, TinyConstants>& mb) {
  std::vector<TinyScalar> q;
  std::vector<TinyScalar> qd;
  std::vector<TinyScalar> tau;
  std::vector<TinyScalar> qdd;

  for (int i = 0; i < mb.m_links.size(); i++) {
    q.push_back(TinyConstants::fraction(1, 10) +
                TinyConstants::fraction(1, 10) * TinyConstants::fraction(i, 1));
    qd.push_back(TinyConstants::fraction(3, 10) +
                 TinyConstants::fraction(1, 10) *
                     TinyConstants::fraction(i, 1));
    tau.push_back(TinyConstants::zero());
    qdd.push_back(TinyConstants::zero());
  }
  while (1) {
    if (visualizer.canSubmitCommand()) {
      for (int i = 0; i < param_uids.size() - 1; i++) {
        double v = visualizer.readUserDebugParameter(param_uids[i + 1]);
        q[i] = TinyConstants::fraction((int)(v * 10.), 10);
      }
    }

    mb.forward_kinematics(q, qd);

    if (visualizer.canSubmitCommand()) {
      if (visual_links.size() == mb.m_links.size() + 1) {
        for (int i = 0; i < mb.m_links.size(); i++) {
          btVector3 pos(TinyConstants::getDouble(
                            mb.m_links[i].m_X_world.m_translation.x()),
                        TinyConstants::getDouble(
                            mb.m_links[i].m_X_world.m_translation.y()),
                        TinyConstants::getDouble(
                            mb.m_links[i].m_X_world.m_translation.z()));

          TinyQuaternion<TinyScalar, TinyConstants> q1;
          mb.m_links[i].m_X_world.m_rotation.getRotation(q1);

          btQuaternion orn(TinyConstants::getDouble(q1.x()),
                           TinyConstants::getDouble(q1.y()),
                           TinyConstants::getDouble(q1.z()),
                           TinyConstants::getDouble(q1.w()));

          visualizer.resetBasePositionAndOrientation(visual_links[i + 1], pos,
                                                     orn);
        }
      }
    }
  }
}

int main(int argc, char* argv[]) {
  b3RobotSimulatorClientAPI_NoDirect visualizer;
  b3PhysicsClientHandle sm = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
  b3RobotSimulatorClientAPI_InternalData data;
  data.m_physicsClientHandle = sm;
  data.m_guiHelper = 0;
  visualizer.setInternalData(&data);

  std::vector<int> linkUids;
  std::vector<int> paramUids;
  if (visualizer.canSubmitCommand()) {
    visualizer.resetSimulation();
    visualizer.loadURDF("plane.urdf");
    int baseId = visualizer.loadURDF("xarm/base.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("base", -4, 4, 0));
    int link1 = visualizer.loadURDF("xarm/link1.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("link1", -4, 4, 0));
    int link2 = visualizer.loadURDF("xarm/link2.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("link2", -4, 4, 0));
    int link3 = visualizer.loadURDF("xarm/link3.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("link3", -4, 4, 0));
    int link4 = visualizer.loadURDF("xarm/link4.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("link4", -4, 4, 0));
    int link5 = visualizer.loadURDF("xarm/link5.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("link5", -4, 4, 0));
    int link6 = visualizer.loadURDF("xarm/link6.urdf");
    paramUids.push_back(visualizer.addUserDebugParameter("link6!", -4, 4, 0));
    linkUids.push_back(baseId);
    linkUids.push_back(link1);
    linkUids.push_back(link2);
    linkUids.push_back(link3);
    linkUids.push_back(link4);
    linkUids.push_back(link5);
    linkUids.push_back(link6);
    b3RobotSimulatorLoadFileResults results;
    b3RobotSimulatorLoadSdfFileArgs sdfArgs;
  }

  {
    btMatrix3x3 m;
    m.setEulerZYX(0, 0, 0.33 * M_PI);
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        double v = DoubleUtils::getDouble(m.getRow(r)[c]);
        printf("v=%f, ", v);
      }
      printf("\n");
    }
  }
  {
    dvec3 a(1, 1, 1);

    dvec3 b(2, 2, 2);
    dvec3 c = a + b;
    dvec3 d;
    d.set_zero();
    double len = c.length();
    printf("len=%f\n", len);

    TinyQuaternion<double, DoubleUtils> q;
    double angle = 0.33 * M_PI;
    q.setRotation(a, angle);
    TinyMatrix3x3<double, DoubleUtils> m(q);
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        double v = DoubleUtils::getDouble(m[r][c]);
        printf("v=%f, ", v);
      }
      printf("\n");
    }
    printf("len = %f!\n", DoubleUtils::getDouble(len));
  }

  { TinyQuaternion<double, DoubleUtils> q; }

  {
    fvec a(Fix64Scalar::one(), Fix64Scalar::one(), Fix64Scalar::one());
    fvec b(Fix64Scalar::two(), Fix64Scalar::two(), Fix64Scalar::two());
    fvec c = a + b;
    Fix64Scalar len = c.length();
    fvec d;
    d.set_zero();
    printf("len=%f\n", Fix64Scalar::getDouble(len));
    TinyQuaternion<Fix64Scalar, Fix64Scalar> q;
  }

  {
    dmat3 m;
    dvec3 v, res;
    res = m * v;
  }

  {
    TinyPose<double, DoubleUtils> pose(
        TinyVector3<double, DoubleUtils>(4.0, 5.0, 6.0),
        TinyQuaternion<double, DoubleUtils>(0, 0, 0, 1));
    TinyVector3<double, DoubleUtils> pos(1, 2, 3);
    pos = pose.transform(pos);
    printf("!\n");
  }

  {
    TinyMultiBody<double, DoubleUtils> mb;
    init_xarm6<double, DoubleUtils>(mb);
    xarm6_fk<double, DoubleUtils>(visualizer, linkUids, paramUids, mb);
  }

  {
    TinyMultiBody<Fix64Scalar, Fix64Scalar> mb;
    init_xarm6<Fix64Scalar, Fix64Scalar>(mb);
    xarm6_fk<Fix64Scalar, Fix64Scalar>(visualizer, linkUids, paramUids, mb);
  }

  return 0;
}
