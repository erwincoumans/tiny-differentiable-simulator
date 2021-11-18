add_executable(ars_train_policy ars_train_policy.cpp  ${TDS_HDRS})
target_link_libraries(ars_train_policy  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_compile_definitions(ars_train_policy PRIVATE USE_TINY )
target_include_directories(ars_train_policy PRIVATE ../../third_party ../../src )

add_executable(ars_train_policy_cuda ars_train_policy_cuda.cpp  ${TDS_HDRS})
target_link_libraries(ars_train_policy_cuda  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_compile_definitions(ars_train_policy_cuda PRIVATE USE_TINY )
target_include_directories(ars_train_policy_cuda PRIVATE ../../third_party ../../src )

find_package(OpenMP)


add_executable(ars_train_policy_omp_ant ars_train_policy_omp.cpp ${TDS_HDRS})

if(OpenMP_CXX_FOUND)
    message("OpenMP_CXX_FOUND")
    target_link_libraries(ars_train_policy_omp_ant PUBLIC OpenMP::OpenMP_CXX tinyxml2 opengl_window ${CMAKE_DL_LIBS})
else()
    target_link_libraries(ars_train_policy_omp_ant  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
endif()
target_compile_definitions(ars_train_policy_omp_ant PRIVATE USE_TINY TRAIN_ANT)
target_include_directories(ars_train_policy_omp_ant PRIVATE ../../third_party ../../src )



add_executable(ars_train_policy_omp_laikago ars_train_policy_omp.cpp ${TDS_HDRS})

if(OpenMP_CXX_FOUND)
    message("OpenMP_CXX_FOUND")
    target_link_libraries(ars_train_policy_omp_laikago PUBLIC OpenMP::OpenMP_CXX tinyxml2 opengl_window ${CMAKE_DL_LIBS})
else()
    target_link_libraries(ars_train_policy_omp_laikago  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
endif()
target_compile_definitions(ars_train_policy_omp_laikago PRIVATE USE_TINY )
target_include_directories(ars_train_policy_omp_laikago PRIVATE ../../third_party ../../src )


add_executable(ars_run_policy ars_run_policy.cpp ../../third_party/stb_image/stb_image.cpp ../../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS})
target_link_libraries(ars_run_policy  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_compile_definitions(ars_run_policy PRIVATE USE_TINY )
target_include_directories(ars_run_policy PRIVATE ../../third_party ../../src )



IF(USE_CPPAD)
   add_executable(cuda_laikago_codegen cuda_codegen.cpp ${TDS_HDRS})
  target_compile_definitions(cuda_laikago_codegen PRIVATE -DUSE_CPPAD -DUSE_CPPAD_CODEGEN)
  target_link_libraries(
    cuda_laikago_codegen
    PRIVATE
      ${CMAKE_DL_LIBS}
      opengl_window
      tinyxml2
      Eigen3::Eigen
      differentiation
  )
  
  add_executable(cuda_ant_codegen cuda_codegen.cpp ${TDS_HDRS})
  target_compile_definitions(cuda_ant_codegen PRIVATE -DUSE_ANT -DUSE_CPPAD -DUSE_CPPAD_CODEGEN)
  target_link_libraries(
    cuda_ant_codegen
    PRIVATE
      ${CMAKE_DL_LIBS}
      opengl_window
      tinyxml2
      Eigen3::Eigen
      differentiation
  )
ENDIF(USE_CPPAD)