
# Copyright 2020 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Tiny Differentiable Simulator (TDS)

cmake_minimum_required(VERSION 3.12)

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # For LLVM tooling
# set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()



if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0017 NEW)
	cmake_policy(SET CMP0042 NEW)
	cmake_policy(SET CMP0057 NEW)
endif(COMMAND cmake_policy)
    
#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(DIFF_PHYSICS)

IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    link_libraries(stdc++fs)
ENDIF()


#The core library consists of those headers
FILE(GLOB_RECURSE TDS_HDRS "src/*.h" "src/*.hpp")


include(ExternalProject)


#find_package(Eigen3 QUIET)
#if (Eigen3_FOUND)
#    message("FOUND EIGEN")
#
#    message("Eigen include directories: ${EIGEN3_INCLUDE_DIRS}")
#    add_definitions(-DUSE_EIGEN)
#else(Eigen3_FOUND)
    OPTION(USE_EIGEN_FALLBACK "Use third_party/eigen3" ON)
      if (USE_EIGEN_FALLBACK)
      message("Using third_party/eigen3")
      add_library (eigen INTERFACE)
      add_library (Eigen3::Eigen ALIAS eigen)
      #target_compile_definitions (eigen INTERFACE ${EIGEN_DEFINITIONS})
      target_include_directories (eigen INTERFACE
          ${CMAKE_CURRENT_LIST_DIR}/third_party/eigen3
      )
      set(Eigen3_FOUND TRUE)
      add_definitions(-DUSE_EIGEN)
    endif(USE_EIGEN_FALLBACK)
#endif (Eigen3_FOUND)

add_definitions(-DTDS_HOME="${CMAKE_CURRENT_LIST_DIR}")
message("Setting TDS_HOME=${CMAKE_CURRENT_LIST_DIR}")


OPTION(TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS "Use left-associative multiplication and row-major (transposed) matrices in RBDL/Featherstone style" OFF)
IF(TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS)
	add_definitions(-DTDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS)
ENDIF()


if(DEFINED Bullet_DIR)
  message("Finding local Bullet")
  find_package(Bullet CONFIG)
  set(BULLET_INCLUDE_DIR ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
  message("BULLET_INCLUDE_DIR=" , ${BULLET_INCLUDE_DIR} )
  set(BLA ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
else()
  OPTION(USE_SYSTEM_BULLET "Try finding system installed version of Bullet, also set SYSTEM_BULLET_IS_DOUBLE_PRECISION!" OFF)
  IF(USE_SYSTEM_BULLET)
  message("Finding system installed Bullet")
  find_package(Bullet)
  OPTION(SYSTEM_BULLET_IS_DOUBLE_PRECISION "System version of Bullet is double precision" OFF)
  if (SYSTEM_BULLET_IS_DOUBLE_PRECISION)
    add_definitions(-DBT_USE_DOUBLE_PRECISION)
  endif()
  ENDIF()
  
endif()



IF(Bullet_FOUND)
  #on Windows, we need to match the default Bullet build settings
  OPTION(USE_MSVC_INCREMENTAL_LINKING "Use MSVC Incremental Linking" OFF)
  OPTION(USE_MSVC_RUNTIME_LIBRARY_DLL "Use MSVC Runtime Library DLL (/MD or /MDd)" OFF)
  IF(MSVC)
  	IF (NOT USE_MSVC_INCREMENTAL_LINKING)
  		#MESSAGE("MSVC_INCREMENTAL_DEFAULT"+${MSVC_INCREMENTAL_DEFAULT})
  		SET( MSVC_INCREMENTAL_YES_FLAG "/INCREMENTAL:NO")
  		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags ${CMAKE_EXE_LINKER_FLAGS_DEBUG})
  		SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "/INCREMENTAL:NO ${replacementFlags}" )
  		MESSAGE("CMAKE_EXE_LINKER_FLAGS_DEBUG=${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
  		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags2 ${CMAKE_EXE_LINKER_FLAGS})
  		SET(CMAKE_EXE_LINKER_FLAGS ${replacementFlag2})
  		STRING(REPLACE "INCREMENTAL:YES" "" replacementFlags3 "${CMAKE_EXTRA_LINK_FLAGS}")
  		SET(CMAKE_EXTRA_LINK_FLAGS ${replacementFlag3})
  		STRING(REPLACE "INCREMENTAL:YES" "INCREMENTAL:NO" replacementFlags3 "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO}")
  		SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO ${replacementFlags3})
  		SET(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "/INCREMENTAL:NO ${replacementFlags3}" )
  	ENDIF (NOT USE_MSVC_INCREMENTAL_LINKING)
  	OPTION(USE_MSVC_RUNTIME_LIBRARY_DLL "Use MSVC Runtime Library DLL (/MD or /MDd)" OFF)
  	IF (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)
  		#We statically link to reduce dependencies
  		FOREACH(flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO )
  			IF(${flag_var} MATCHES "/MD")
  				STRING(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
  			ENDIF(${flag_var} MATCHES "/MD")
  			IF(${flag_var} MATCHES "/MDd")
  				STRING(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
  			ENDIF(${flag_var} MATCHES "/MDd")
  		ENDFOREACH(flag_var)
  	ENDIF (NOT USE_MSVC_RUNTIME_LIBRARY_DLL)

  	IF (CMAKE_CL_64)
  	  ADD_DEFINITIONS(-D_WIN64)
  	ELSE()
  	  OPTION(USE_MSVC_SSE "Use MSVC /arch:sse option"	OFF)
  	  option(USE_MSVC_SSE2 "Compile your program with SSE2 instructions" ON)
  	  IF (USE_MSVC_SSE)
  		  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE")
  	  ENDIF()
  	  IF (USE_MSVC_SSE2)
  		  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
  	  ENDIF()
  	ENDIF()
  	option(USE_MSVC_AVX "Compile your program with AVX instructions"  OFF)
  	IF(USE_MSVC_AVX)
  		add_definitions(/arch:AVX)
  	ENDIF()
  	OPTION(USE_MSVC_FAST_FLOATINGPOINT "Use MSVC /fp:fast option"	ON)
  	IF (USE_MSVC_FAST_FLOATINGPOINT)
  		  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast")
  	ENDIF()
  	OPTION(USE_MSVC_STRING_POOLING "Use MSVC /GF string pooling option"	ON)
  	IF (USE_MSVC_STRING_POOLING)
  		SET(CMAKE_C_FLAGS "/GF ${CMAKE_C_FLAGS}")
  		SET(CMAKE_CXX_FLAGS "/GF ${CMAKE_CXX_FLAGS}")
  	ENDIF()
  	OPTION(USE_MSVC_FUNCTION_LEVEL_LINKING "Use MSVC /Gy function level linking option"	ON)
  	IF(USE_MSVC_FUNCTION_LEVEL_LINKING)
  		SET(CMAKE_C_FLAGS "/Gy ${CMAKE_C_FLAGS}")
  		SET(CMAKE_CXX_FLAGS "/Gy ${CMAKE_CXX_FLAGS}")
  		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /OPT:REF")
  		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /OPT:REF")
  	ENDIF(USE_MSVC_FUNCTION_LEVEL_LINKING)
  	OPTION(USE_MSVC_EXEPTIONS "Use MSVC C++ exceptions option"	OFF)
  	OPTION(USE_MSVC_COMDAT_FOLDING "Use MSVC /OPT:ICF COMDAT folding option"	ON)
  	IF(USE_MSVC_COMDAT_FOLDING)
  		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /OPT:ICF")
  		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /OPT:ICF")
  	ENDIF()
  	OPTION(USE_MSVC_DISABLE_RTTI "Use MSVC /GR- disabled RTTI flags option"	OFF)
  	IF(USE_MSVC_DISABLE_RTTI)
  	  STRING(REGEX REPLACE "/GR" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}) # Disable RTTI
  		SET(CMAKE_C_FLAGS "/GR- ${CMAKE_C_FLAGS}")
  		SET(CMAKE_CXX_FLAGS "/GR- ${CMAKE_CXX_FLAGS}")
  	ENDIF(USE_MSVC_DISABLE_RTTI)
  	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4244 /wd4267")
  ENDIF(MSVC)
ENDIF(Bullet_FOUND)

if (Eigen3_FOUND)
OPTION (USE_RBDL "Compare against RBDL" ON)
if(USE_RBDL)
  add_definitions(-DUSE_RBDL)
  SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/rbdl)
endif(USE_RBDL)
endif(Eigen3_FOUND)

OPTION (USE_CGAL "Include CGAL libraries for mesh generation" OFF)
if (USE_CGAL)
  message("IN CGAL") 
  find_package(Boost)
  find_library(gmp NAMES gmp)
  find_library(mpfr NAMES mpfr)
  if(Boost_FOUND AND gmp AND mpfr)
    add_definitions(-DUSE_CGAL)
    set(CGAL_FOUND TRUE)
    # set(CGAL_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/third_party/CGAL-5.2/include)
    # SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/CGAL)
    find_package(CGAL REQUIRED PATHS ./third_party/CGAL)
  else()
    message(STATUS "Missing required components for CGAL. Required libraries: Boost, GMP, MPFR")
  endif()
endif(USE_CGAL)

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /std:c++17")
endif(MSVC)

message ("USING third_party/tinyxml2")
set(tinyxml2_FOUND TRUE)
set(tinyxml2_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/third_party/tinyxml2/include)
SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/tinyxml2)

IF(WIN32)
	SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Shared Libs" FORCE)
ELSE(WIN32)
	SET(BUILD_SHARED_LIBS ON CACHE BOOL "Shared Libs" FORCE)
ENDIF(WIN32)


ADD_DEFINITIONS(-DSTATIC_LINK_SPD_PLUGIN)

OPTION(USE_STAN "Use Stan Math" OFF)
if(USE_STAN)
	add_definitions(-pthread -D_REENTRANT -DBOOST_DISABLE_ASSERTS -DUSE_STAN)
	set(STAN_LIBRARIES ${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/tbb/libtbb.so.2)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math)
	set(Eigen3_FOUND true)
	# use Eigen distribution from Stan Math
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/boost_1.72.0)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/eigen_3.3.7)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/cpplint_1.4.5)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/opencl_2.2.0)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/sundials_5.2.0/include)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/stan_math/lib/tbb_2019_U8/include)
endif(USE_STAN)

OPTION(USE_ENOKI "Use Enoki" OFF)
IF(USE_ENOKI)
  add_definitions(-DUSE_ENOKI)
  set(ENOKI_AUTODIFF ON CACHE BOOL "Use enoki with autodiff" FORCE)
  set(ENOKI_CUDA ON CACHE BOOL "Use enoki with CUDA" FORCE)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/enoki)
  enoki_set_compile_flags()
  enoki_set_native_flags()
  get_directory_property(ENOKI_COMPILE_OPTIONS     COMPILE_OPTIONS)
  get_directory_property(ENOKI_COMPILE_DEFINITIONS COMPILE_DEFINITIONS)
  set_property(DIRECTORY . PROPERTY COMPILE_OPTIONS     ${ENOKI_COMPILE_OPTIONS})
  set_property(DIRECTORY . PROPERTY COMPILE_DEFINITIONS ${ENOKI_COMPILE_DEFINITIONS})
  set(ENOKI_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/third_party/enoki/include)
  include_directories(${ENOKI_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD_LIBRARIES ${CMAKE_CXX_STANDARD_LIBRARIES})
  # set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})
  # set(CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS})
ENDIF(USE_ENOKI)


OPTION(USE_OPTIM "Use OptimLib" OFF)
if(USE_OPTIM)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/optim)
	add_definitions(-DUSE_OPTIM)
	add_definitions(-DOPTIM_ENABLE_EIGEN_WRAPPERS)
endif(USE_OPTIM)

OPTION(USE_CPPAD "Use CppAD and CppADCodeGen" OFF)
IF(USE_CPPAD)
  # Add CppAD as an external project and an imported target.  
  ExternalProject_Add(CppAD-Project
    PREFIX CppAD
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/third_party/CppAD
    CMAKE_ARGS
      -Dcppad_prefix=${CMAKE_CURRENT_BINARY_DIR}/CppAD
  )
  add_library(CppAD INTERFACE IMPORTED)
  add_dependencies(CppAD CppAD-Project)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/CppAD/include)
  target_include_directories(CppAD INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/CppAD/include)
  target_compile_definitions(CppAD INTERFACE -DUSE_CPPAD)
  
  # Same for CppADCodeGen
  ExternalProject_Add(CppADCodeGen-Project
    PREFIX CppADCodeGen
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/third_party/CppADCodeGen
    CMAKE_ARGS
      -DCPPAD_INCLUDE_DIR=${CMAKE_CURRENT_BINARY_DIR}/CppAD/include
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/CppADCodeGen
      -DGOOGLETEST_GIT=ON
  )
  add_dependencies(CppADCodeGen-Project CppAD)
  add_library(CppADCodeGen INTERFACE IMPORTED)
  add_dependencies(CppADCodeGen CppAD-Project CppADCodeGen-Project)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/CppAD/include)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/CppADCodeGen/include)
  target_include_directories(CppADCodeGen
    INTERFACE
      ${CMAKE_CURRENT_BINARY_DIR}/CppAD/include
      ${CMAKE_CURRENT_BINARY_DIR}/CppADCodeGen/include
  )
ENDIF(USE_CPPAD)

set(BENCHMARK_ENABLE_GTEST_TESTS OFF)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/benchmark)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/HighFive)

OPTION(USE_PAGMO2 "Use Pagmo2" OFF)
IF(USE_PAGMO2)
    find_package(Boost QUIET REQUIRED)
    if (Boost_FOUND)
    message("found boost!")
    endif()

    find_package(TBB REQUIRED)
    IF(TBB_FOUND)
     message("TBB found")
    ENDIF()

    find_package(Pagmo CONFIG)
    if (Pagmo_FOUND)
      message("FOUND PAGMO2!")
      message("${PAGMO_INCLUDE_DIRS}")
      add_definitions(-DUSE_PAGMO)
    endif()
ENDIF()

OPTION(USE_CERES "Use Ceres solver" OFF)
IF(USE_CERES)
    find_package(glog)
    find_package(gflags)
    #find_package(Ceres)
    find_package(Ceres CONFIG REQUIRED)
    IF(ceres_FOUND)
	    add_definitions(-DUSE_CERES)
	    message("FOUND CERES!")
    ELSE()
#      ExternalProject_Add(glog-Project
#          PREFIX glog
#          SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/third_party/glog
#          CMAKE_ARGS
#              -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/glog
#      )
#      add_library(glog INTERFACE IMPORTED)
#      add_dependencies(glog glog-Project)
#      include_directories(${CMAKE_CURRENT_BINARY_DIR}/glog/include)
#  
#      ExternalProject_Add(ceres-solver-Project
#          PREFIX ceres-solver
#          SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/third_party/ceres-solver
#          CMAKE_ARGS
#            -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
#            -DBUILD_EXAMPLES=OFF
#        )
#      add_library(ceres-solver INTERFACE IMPORTED)
#      add_dependencies(ceres-solver ceres-solver-Project glog eigen3)
#      include_directories(${CMAKE_CURRENT_BINARY_DIR}/ceres-solver/include)
    ENDIF()
ENDIF(USE_CERES)

#IF(USE_CERES)
#  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/ceres-solver)
#  SET(CERES_LIBRARIES Ceres::ceres)
#ENDIF()


# set(MINIGLOG ON)
# set(BUILD_TESTING OFF)
# set(BUILD_EXAMPLES OFF)

set(CXXOPTS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CXXOPTS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(CXXOPTS_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/cxxopts)
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/third_party/fmt)

subdirs(${CMAKE_CURRENT_LIST_DIR}/src/visualizer/opengl)
subdirs(${CMAKE_CURRENT_LIST_DIR}/src/visualizer/pybullet)

OPTION(USE_TINY_RENDERER "Use TinyRenderer" OFF)

SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/cpp_base64)
#SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/cloud_seed)
#SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/tiny_audio)
SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/dear_imgui)
SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/crossguid)
SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/zeromq)
#SUBDIRS(third_party/nlohmann)

add_library(differentiation INTERFACE)
target_link_directories(differentiation INTERFACE Eigen3::Eigen)
target_include_directories(differentiation INTERFACE )

#if(USE_CERES)
#    target_link_libraries(differentiation INTERFACE ceres-solver)
#endif(USE_CERES)

if(USE_CPPAD)
    target_link_libraries(differentiation INTERFACE CppADCodeGen)
    target_compile_definitions(differentiation INTERFACE -DUSE_CPPAD_CODEGEN)
    
endif(USE_CPPAD)


OPTION(USE_TESTING "Run unit tests" ON)
IF(USE_TESTING)
    ENABLE_TESTING()
    SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/gtest)
    include_directories(${CMAKE_CURRENT_LIST_DIR}/third_party/gtest/include)
    SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/test)
ENDIF()
SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/examples)

OPTION(BUILD_PYTHON_PLUGIN "Set when you want to build pydiffphys (Python bindings for tiny-differentiable-simulator)" ON)
IF(BUILD_PYTHON_PLUGIN)
  if (Eigen3_FOUND)
  SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/python)
  endif()
ENDIF()

OPTION(USE_OSQP "Use OSQP solver" OFF)
IF(USE_OSQP)
    find_package(osqp)
    #SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/osqp)
ENDIF()

OPTION(USE_QPOASES "Use qpOASES solver" OFF)
IF(USE_QPOASES)
    SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/qpOASES)
ENDIF()