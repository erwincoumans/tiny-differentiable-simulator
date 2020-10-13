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

#include "visualizer/pybullet/pybullet_visualizer_api.h"

#include <Bullet3Common/b3Logging.h>
#include <SharedMemory/PhysicsDirectC_API.h>
#include <SharedMemory/SharedMemoryInProcessPhysicsC_API.h>
#include <SharedMemory/SharedMemoryPublic.h>
#include <SharedMemory/b3RobotSimulatorClientAPI_InternalData.h>

#ifdef BT_ENABLE_ENET
#include <SharedMemory/PhysicsClientUDP_C_API.h>
#endif  // PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include <SharedMemory/PhysicsClientTCP_C_API.h>
#endif  // PHYSICS_TCP

PyBulletVisualizerAPI::PyBulletVisualizerAPI() {}

PyBulletVisualizerAPI::~PyBulletVisualizerAPI() {}

bool PyBulletVisualizerAPI::connect(int mode, const std::string &hostName,
                                    int portOrKey) {
  if (m_data->m_physicsClientHandle) {
    b3Warning("Already connected, disconnect first.");
    return false;
  }
  b3PhysicsClientHandle sm = nullptr;

  int udpPort = 1234;
  int tcpPort = 6667;
  int key = SHARED_MEMORY_KEY;

  if (portOrKey >= 0) {
    key = portOrKey;
  }
  switch (mode) {
    case eCONNECT_SHARED_MEMORY: {
      sm = b3ConnectSharedMemory(key);
      break;
    }

    case eCONNECT_GUI: {
      int argc = 0;
      char *argv[1] = {0};
#ifdef __APPLE__
      sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
      sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
      break;
    }
    case eCONNECT_DIRECT: {
      sm = b3ConnectPhysicsDirect();
      break;
    }
  }

  if (sm) {
    m_data->m_physicsClientHandle = sm;
    if (!b3CanSubmitCommand(m_data->m_physicsClientHandle)) {
      disconnect();
      return false;
    }
    return true;
  }
  return false;
}
