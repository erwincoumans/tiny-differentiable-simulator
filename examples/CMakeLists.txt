SUBDIRS(ars)
# first the examples that have no dependencies 
# then examples with only included third_party dependencies
# at the end, optional complex examples with complicated dependencies (PyTorch, STAN, BOOST etc)

add_executable(pendulum_example_tiny_gui pendulum_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(pendulum_example_tiny_gui opengl_window ${CMAKE_DL_LIBS})
target_compile_definitions(pendulum_example_tiny_gui PRIVATE USE_TINY )
target_include_directories(pendulum_example_tiny_gui PRIVATE ../src )

add_executable(pendulum_example_tiny pendulum_example.cpp  ${TDS_HDRS})
target_link_libraries(pendulum_example_tiny)
target_compile_definitions(pendulum_example_tiny PRIVATE USE_TINY )
target_include_directories(pendulum_example_tiny PRIVATE ../src )



if (Eigen3_FOUND)
add_executable(pendulum_example_eigen_gui pendulum_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(pendulum_example_eigen_gui opengl_window Eigen3::Eigen ${CMAKE_DL_LIBS})
target_include_directories(pendulum_example_eigen_gui PRIVATE ../src)

add_executable(spherical_joint_test_eigen spherical_joint_test.cpp  ${TDS_HDRS})
target_link_libraries(spherical_joint_test_eigen opengl_window tinyxml2  Eigen3::Eigen ${CMAKE_DL_LIBS})
target_include_directories(spherical_joint_test_eigen PRIVATE ../src)

add_executable(laikago_opengl_eigen_example laikago_opengl_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS})
target_link_libraries(laikago_opengl_eigen_example  Eigen3::Eigen tinyxml2 opengl_window ${CMAKE_DL_LIBS})
target_include_directories(laikago_opengl_eigen_example PRIVATE ../third_party/tinyobjloader)
target_include_directories(laikago_opengl_eigen_example PRIVATE ../third_party ../src)

add_executable(laikago_opengl_tiny_example laikago_opengl_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS})
target_link_libraries(laikago_opengl_tiny_example  Eigen3::Eigen tinyxml2 opengl_window ${CMAKE_DL_LIBS})
target_include_directories(laikago_opengl_tiny_example PRIVATE ../third_party/tinyobjloader)
target_include_directories(laikago_opengl_tiny_example PRIVATE ../third_party ../src)
target_compile_definitions(laikago_opengl_tiny_example PRIVATE USE_TINY )

endif(Eigen3_FOUND)


add_executable(spherical_joint_test_tiny spherical_joint_test.cpp  ${TDS_HDRS})
target_link_libraries(spherical_joint_test_tiny opengl_window tinyxml2  ${CMAKE_DL_LIBS})
target_compile_definitions(spherical_joint_test_tiny PRIVATE USE_TINY )
target_include_directories(spherical_joint_test_tiny PRIVATE ../src)

add_executable(billiard_opt_example_gui billiard_opt_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(billiard_opt_example_gui opengl_window ${CMAKE_DL_LIBS})
target_include_directories(billiard_opt_example_gui PRIVATE ../src)

add_executable(soft_contact_example soft_contact_example.cpp  ${TDS_HDRS})
target_link_libraries(soft_contact_example opengl_window Eigen3::Eigen  ${CMAKE_DL_LIBS})
target_compile_definitions(soft_contact_example PRIVATE USE_TINY )
target_include_directories(soft_contact_example PRIVATE ../src)



add_executable(tiny_urdf_parser_opengl_example tiny_urdf_parser_opengl_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS}) 
target_link_libraries(tiny_urdf_parser_opengl_example  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party/tinyobjloader)
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party ../src)



add_executable(opengl_imgui_test opengl_imgui_test.cpp ../third_party/dear_imgui/opengl3/imgui_impl_opengl3.cpp  ${TDS_HDRS})
target_link_libraries(opengl_imgui_test opengl_window dear_imgui  ${CMAKE_DL_LIBS})
target_include_directories(opengl_imgui_test PRIVATE ../third_party/glad ../third_party/dear_imgui ../src)

IF(USE_CPPAD)
  add_executable(cuda_codegen cuda_codegen.cpp ${TDS_HDRS})
  target_compile_definitions(cuda_codegen PRIVATE -DUSE_CPPAD)
  target_link_libraries(
    cuda_codegen
    PRIVATE
      ${CMAKE_DL_LIBS}
      opengl_window
      tinyxml2
      Eigen3::Eigen
      differentiation
  )
  if (USE_STAN)
    target_link_libraries(cuda_codegen PRIVATE ${STAN_LIBRARIES})
  endif(USE_STAN)

  add_executable(cuda_laikago_codegen cuda_laikago_codegen.cpp ${TDS_HDRS})
  target_compile_definitions(cuda_laikago_codegen PRIVATE -DUSE_CPPAD)
  target_link_libraries(
    cuda_laikago_codegen
    PRIVATE
      ${CMAKE_DL_LIBS}
      opengl_window
      tinyxml2
      Eigen3::Eigen
      differentiation
  )
  add_executable(cuda_ant_codegen cuda_ant_codegen.cpp ${TDS_HDRS})
  target_compile_definitions(cuda_ant_codegen PRIVATE -DUSE_CPPAD)
  target_link_libraries(
    cuda_ant_codegen
    PRIVATE
      ${CMAKE_DL_LIBS}
      opengl_window
      tinyxml2
      Eigen3::Eigen
      differentiation
  )
  if (USE_STAN)
    target_link_libraries(cuda_laikago_codegen PRIVATE ${STAN_LIBRARIES})
    target_link_libraries(cuda_ant_codegen PRIVATE ${STAN_LIBRARIES})
  endif(USE_STAN)
ENDIF(USE_CPPAD)

IF(USE_CERES)
message("Ceres libs:" ${CERES_LIBRARIES})

add_executable(ceres_estimation_real ceres_estimation_real.cpp ${TDS_HDRS})
target_link_libraries(ceres_estimation_real pybullet_visualizer_api  glog::glog Eigen3::Eigen ${CMAKE_DL_LIBS}  )

# We want to link against ceres-debug.lib in Debug builds and Release/ceres.lib in optimized/Release builds.
if (WIN32)
    target_link_libraries(ceres_estimation_real  Opengl32)
    target_link_libraries( ceres_estimation_real 
      debug ${PROJECT_SOURCE_DIR}/third_party/ceres-solver/build_cmake/local_install/lib/Debug/ceres-debug.lib 
      optimized  ${PROJECT_SOURCE_DIR}/third_party/ceres-solver/build_cmake/local_install/lib/Release/ceres.lib)
    target_include_directories( ceres_estimation_real PRIVATE  ../third_party/glog/build_cmake/local_install/include ../third_party/ceres-solver/build_cmake/local_install/include )

else(WIN32)
    target_link_libraries(ceres_estimation_real pybullet_visualizer_api Ceres::ceres)
endif (WIN32)
target_include_directories(ceres_estimation_real PRIVATE ../data/ibm-double-pendulum)

IF(APPLE)
  FIND_PACKAGE(OpenGL)
  target_link_libraries(ceres_estimation_real ${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF()

ENDIF()

add_executable(laikago_footstep_ik laikago_footstep_ik.cpp ${TDS_HDRS})
target_include_directories(laikago_footstep_ik PRIVATE ../src )
target_link_libraries(laikago_footstep_ik opengl_window tinyxml2 Eigen3::Eigen ${CMAKE_DL_LIBS})

if (WIN32)
    target_link_libraries(laikago_footstep_ik  Opengl32)
endif (WIN32)

IF(APPLE)
  FIND_PACKAGE(OpenGL)
  target_link_libraries(laikago_footstep_ik ${COCOA} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF()

add_executable(opengl_test opengl_test.cpp  ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS})
target_include_directories(opengl_test PRIVATE ../third_party/tinyobjloader)
target_include_directories(opengl_test PRIVATE ../third_party ../src)
target_link_libraries(opengl_test opengl_window ${CMAKE_DL_LIBS})

add_executable(tiny_urdf_parser_example tiny_urdf_parser_example.cpp ${TDS_HDRS})
target_link_libraries(tiny_urdf_parser_example tinyxml2)
target_include_directories(tiny_urdf_parser_example PRIVATE ../third_party ../src)


set(MESHCAT_LIBRARIES crossguid cpp_base64 zmq )
IF(WIN32)
    set(MESHCAT_LIBRARIES ${MESHCAT_LIBRARIES} ws2_32 Winmm iphlpapi)
ENDIF()

add_executable(tiny_urdf_parser_meshcat_example tiny_urdf_parser_meshcat_example.cpp ${TDS_HDRS}) 
target_link_libraries(tiny_urdf_parser_meshcat_example  tinyxml2 ${MESHCAT_LIBRARIES}) 
target_include_directories(tiny_urdf_parser_meshcat_example PRIVATE ../third_party ../third_party/nlohmann/include ../src)

add_executable(meshcat_zmq_example meshcat_zmq_example.cpp ${TDS_HDRS})
target_link_libraries(meshcat_zmq_example  ${MESHCAT_LIBRARIES})
target_include_directories(meshcat_zmq_example PRIVATE ../third_party ../third_party/nlohmann/include ../src)

add_executable(eval_laikago_env ars/eval_laikago_env.cpp  ${TDS_HDRS}) 
target_link_libraries(eval_laikago_env  tinyxml2   ${MESHCAT_LIBRARIES}) 
target_include_directories(eval_laikago_env PRIVATE ../third_party ../third_party/nlohmann/include ../src)


add_executable(sdf_to_mesh_example sdf_to_mesh_example.cpp)
target_include_directories(sdf_to_mesh_example PRIVATE ../third_party ../src)
target_link_libraries(sdf_to_mesh_example opengl_window ${CMAKE_DL_LIBS})

IF(USE_OSQP)
  add_executable(laikago_tds_mpc whole_body_control/laikago_tds_mpc.cpp)
  target_link_libraries(laikago_tds_mpc tinyxml2 ${MESHCAT_LIBRARIES}
          Eigen3::Eigen)
  target_include_directories(laikago_tds_mpc PRIVATE
      ../third_party ../third_party/nlohmann/include ../src whole_body_control)
  
  target_link_libraries(laikago_tds_mpc osqp::osqp)
  target_include_directories(laikago_tds_mpc PRIVATE ../third_party/osqp/build_cmake/local_install/include)
  target_compile_definitions(laikago_tds_mpc PRIVATE -DTDS_ENABLE_OSQP)
  
  IF(USE_QPOASES)
      target_link_libraries(laikago_tds_mpc qpOASES)
      target_include_directories(laikago_tds_mpc PRIVATE ../third_party/qpOASES/include)
      target_compile_definitions(laikago_tds_mpc PRIVATE -DTDS_ENABLE_QPOASES)
  ENDIF(USE_QPOASES)
ENDIF(USE_OSQP)


add_executable(environment_eval environment_eval.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS}) 
target_link_libraries(environment_eval  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_include_directories(environment_eval PRIVATE ../third_party/tinyobjloader)
target_include_directories(environment_eval PRIVATE ../third_party ../src)




IF(USE_PAGMO2)
  add_executable(pagmo2_example pagmo2_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS} )
  target_link_libraries(pagmo2_example tinyxml2 Eigen3::Eigen Pagmo::pagmo ${CMAKE_DL_LIBS})
  target_include_directories(pagmo2_example PRIVATE ../src ../third_party/tinyobjloader ../third_party)
IF (WIN32)
  target_compile_definitions(pagmo2_example PRIVATE -D_CPPUNWIND)
ENDIF()

  add_executable(environment_train_pagmo environment_train.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS} )
  target_link_libraries(environment_train_pagmo tinyxml2 Eigen3::Eigen Pagmo::pagmo ${CMAKE_DL_LIBS})
  target_include_directories(environment_train_pagmo PRIVATE ../src ../third_party/tinyobjloader ../third_party)
IF (WIN32)
  target_compile_definitions(environment_train_pagmo PRIVATE -D_CPPUNWIND)
ENDIF()
ENDIF()
