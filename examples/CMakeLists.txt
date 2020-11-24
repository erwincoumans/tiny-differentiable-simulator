# first the examples that have no dependencies 
# then examples with only included third_party dependencies
# at the end, optional complex examples with complicated dependencies (PyTorch, STAN, BOOST etc)

add_executable(pendulum_example_tiny_gui pendulum_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(pendulum_example_tiny_gui opengl_window ${CMAKE_DL_LIBS})
target_compile_definitions(pendulum_example_tiny_gui PRIVATE USE_TINY )
target_include_directories(pendulum_example_tiny_gui PRIVATE ../src )

add_executable(pendulum_example_eigen_gui pendulum_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(pendulum_example_eigen_gui opengl_window ${CMAKE_DL_LIBS})
target_include_directories(pendulum_example_eigen_gui PRIVATE ../src  ${EIGEN3_INCLUDE_DIRS})


add_executable(billiard_opt_example_gui billiard_opt_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(billiard_opt_example_gui opengl_window ${CMAKE_DL_LIBS})
target_include_directories(billiard_opt_example_gui PRIVATE ../src)


add_executable(tiny_urdf_parser_opengl_example tiny_urdf_parser_opengl_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS}) 
target_link_libraries(tiny_urdf_parser_opengl_example  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party/tinyobjloader)
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party ../src)

add_executable(opengl_imgui_test opengl_imgui_test.cpp ../third_party/dear_imgui/opengl3/imgui_impl_opengl3.cpp  ${TDS_HDRS})
target_link_libraries(opengl_imgui_test opengl_window dear_imgui TinyAudio ${CMAKE_DL_LIBS})
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
      CppADCodeGen
  )
  if (USE_STAN)
    target_link_libraries(cuda_codegen PRIVATE ${STAN_LIBRARIES})
  endif(USE_STAN)
ENDIF(USE_CPPAD)

IF(Bullet_FOUND)
  add_executable(laikago_footstep_ik laikago_footstep_ik.cpp ${TDS_HDRS})
  target_include_directories(laikago_footstep_ik PRIVATE ../src ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(laikago_footstep_ik pybullet_visualizer_api)
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

