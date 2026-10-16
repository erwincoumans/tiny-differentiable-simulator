# Batched CPU and CUDA kernels generated with CppADCodeGen, see
# cmake/TdsKernels.cmake and src/utils/kernel_codegen.hpp.

if(Eigen3_FOUND)

tds_add_kernels(g1
  GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/g1/generated
  GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/g1/g1_codegen.cpp)

add_executable(g1_kernel_benchmark g1/g1_kernel_benchmark.cpp ${TDS_HDRS})
target_link_libraries(g1_kernel_benchmark g1_kernels tinyxml2 Eigen3::Eigen ${CMAKE_DL_LIBS})
target_include_directories(g1_kernel_benchmark PRIVATE g1 ../../src ../../third_party)
if(USE_TESTING)
  add_test(NAME g1_kernels
    COMMAND g1_kernel_benchmark --worlds 64 --no-benchmark
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/data)
endif()

# the laikago environment of examples/environments/laikago_environment2.h,
# which also includes the generated kernel directly
tds_add_kernels(laikago
  GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/laikago/generated
  GENERATOR ${CMAKE_CURRENT_SOURCE_DIR}/laikago/laikago_codegen.cpp
  GENERATOR_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/examples)

endif(Eigen3_FOUND)
