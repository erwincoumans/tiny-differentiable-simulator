#
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

#Tiny Differentiable Simulator or TDS

cmake_minimum_required(VERSION 2.4.3)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # For LLVM tooling

cmake_policy(SET CMP0042 NEW)

cmake_policy(SET CMP0017 NEW)
 if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)
    
#this line has to appear before 'PROJECT' in order to be able to disable incremental linking
SET(MSVC_INCREMENTAL_DEFAULT ON)

PROJECT(DIFF_PHYSICS)

#The core library consists of those headers
FILE(GLOB_RECURSE TDS_HDRS "src/*.h" "src/*.hpp")

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3)
if (Eigen3_FOUND)
    message("FOUND EIGEN")
    add_definitions(-DUSE_EIGEN)
    #include_directories("third_party/eigen3")
else(Eigen3_FOUND)
    set(Eigen3_FOUND TRUE)
    message("Using third_party/eigen3")
    add_definitions(-DUSE_EIGEN)
    set(EIGEN3_INCLUDE_DIRS  ${DIFF_PHYSICS_SOURCE_DIR}/third_party/eigen3)
    message("Eigen=")
    message(${EIGEN3_INCLUDE_DIRS})
    #include_directories(${EIGEN3_INCLUDE_DIRS})
endif (Eigen3_FOUND)


OPTION(TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS "Use left-associative multiplication and row-major (transposed) matrices in RBDL/Featherstone style" OFF)
IF(TDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS)
	add_definitions(-DTDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS)
ENDIF()


if(DEFINED Bullet_DIR)
  message("Finding local Bullet")
  find_package(Bullet CONFIG)
  set(BULLET_INCLUDE_DIR ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_INCLUDE_DIR})
  message("BULLET_INCLUDE_DIR=",${BULLET_INCLUDE_DIR})
  set(BLA ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
else()
  message("Finding system installed Bullet")
  find_package(Bullet)
  OPTION(SYSTEM_BULLET_IS_DOUBLE_PRECISION "System version of Bullet is double precision" OFF)
  if (SYSTEM_BULLET_IS_DOUBLE_PRECISION)
    add_definitions(-DBT_USE_DOUBLE_PRECISION)
  endif()
  
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
  
  SUBDIRS(src/visualizer/pybullet)

  OPTION (USE_RBDL "Compare against RBDL" ON)
  if(USE_RBDL)
    add_definitions(-DUSE_RBDL)
    SUBDIRS(${CMAKE_CURRENT_LIST_DIR}/third_party/rbdl)
  endif(USE_RBDL)

ENDIF(Bullet_FOUND)

find_package(tinyxml2)
if (tinyxml2_FOUND)
    message("USING system tinyxml2")
    include_directories(${tinyxml2_INCLUDE_DIRS})
else (tinyxml2_FOUND)
    message ("USING third_party/tinyxml2")
    set(tinyxml2_FOUND TRUE)
    set(tinyxml2_INCLUDE_DIRS ../third_party/tinyxml2/include)
    SUBDIRS(../third_party/tinyxml2)
endif (tinyxml2_FOUND)

IF(WIN32)
	SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Shared Libs" FORCE)
ELSE(WIN32)
	SET(BUILD_SHARED_LIBS ON CACHE BOOL "Shared Libs" FORCE)
ENDIF(WIN32)




ADD_DEFINITIONS(-DSTATIC_LINK_SPD_PLUGIN)

SUBDIRS(third_party/gtest)
SUBDIRS(test)
SUBDIRS(src/visualizer/opengl)

SUBDIRS(third_party/cpp_base64)
SUBDIRS(third_party/cloud_seed)
SUBDIRS(third_party/tiny_audio)
SUBDIRS(third_party/dear_imgui)
SUBDIRS(third_party/crossguid)
SUBDIRS(third_party/zeromq)
#SUBDIRS(third_party/nlohmann)


SUBDIRS(examples)

OPTION(BUILD_PYTHON_PLUGIN "Set when you want to build pydiffphys (Python bindings for tiny-differentiable-simulator)" ON)
IF(BUILD_PYTHON_PLUGIN)
  SUBDIRS(python)
ENDIF()
