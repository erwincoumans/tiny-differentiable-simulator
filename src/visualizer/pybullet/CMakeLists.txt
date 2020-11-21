if(Bullet_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  include_directories(${BULLET_INCLUDE_DIRS})
  add_library(pybullet_visualizer_api STATIC pybullet_visualizer_api.cpp)
  target_link_libraries(
    pybullet_visualizer_api BulletRoboticsGUI BulletExampleBrowserLib
    BulletRobotics BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics
    BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath
    OpenGLWindow gwen BussIK Bullet3Common 
  )
  target_include_directories(pybullet_visualizer_api PUBLIC ${DIFF_PHYSICS_SOURCE_DIR}/src ${BULLET_INCLUDE_DIR})
  target_compile_definitions(pybullet_visualizer_api PUBLIC ${BULLET_DEFINITIONS})
  target_link_directories(pybullet_visualizer_api PUBLIC ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})

  if(WIN32)
    target_link_libraries(pybullet_visualizer_api Ws2_32 Opengl32)
  else()
    target_link_libraries(pybullet_visualizer_api pthread ${CMAKE_DL_LIBS})
  endif()
endif(Bullet_FOUND)
