#!/usr/bin/python
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



SET(PYTHON_VERSION_DIFF_PHYSICS "" CACHE STRING "Python version pytinydiffsim will use.")
SET(Python_ADDITIONAL_VERSIONS 3 3.9 3.8 3.7 3.6 3.5 3.4 3.3 3.2 3.1 3.0 2.7 2.7.12 2.7.10 2.7.3 )
SET_PROPERTY(CACHE PYTHON_VERSION_DIFF_PHYSICS PROPERTY STRINGS ${Python_ADDITIONAL_VERSIONS})
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build3/cmake ${CMAKE_MODULE_PATH})
OPTION(EXACT_PYTHON_VERSION "Require Python and match PYTHON_VERSION_DIFF_PHYSICS exactly, e.g. 2.7.12" OFF)
IF(EXACT_PYTHON_VERSION)
    set(EXACT_PYTHON_VERSION_FLAG EXACT REQUIRED)
ENDIF(EXACT_PYTHON_VERSION)
# first find the python interpreter
FIND_PACKAGE(PythonInterp ${PYTHON_VERSION_DIFF_PHYSICS} ${EXACT_PYTHON_VERSION_FLAG})
# python library should exactly match that of the interpreter
# the following can result in fatal error if you don't have the right python configuration
FIND_PACKAGE(PythonLibs ${PYTHON_VERSION_STRING} EXACT)


FIND_PACKAGE(PythonLibs)





IF(WIN32)
	SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Shared Libs" FORCE)
ELSE(WIN32)
	SET(BUILD_SHARED_LIBS ON CACHE BOOL "Shared Libs" FORCE)
ENDIF(WIN32)

IF(APPLE)
	OPTION(BUILD_DIFF_PHYSICS_MAC_USE_PYTHON_FRAMEWORK "Set when you want to use the Python Framework on Mac" ON)
	IF(NOT BUILD_DIFF_PHYSICS_MAC_USE_PYTHON_FRAMEWORK)
		add_definitions(-DB3_NO_PYTHON_FRAMEWORK)
	ENDIF(NOT BUILD_DIFF_PHYSICS_MAC_USE_PYTHON_FRAMEWORK)
	OPTION(BUILD_DIFF_PHYSICS_SHOW_PY_VERSION "Set when you want to show the PY_MAJOR_VERSION and PY_MAJOR_VERSION using #pragme message." OFF)
	IF(BUILD_DIFF_PHYSICS_SHOW_PY_VERSION)
		add_definitions(-DB3_DUMP_PYTHON_VERSION)
	ENDIF()
ENDIF(APPLE)


INCLUDE_DIRECTORIES( 
    ${DIFF_PHYSICS_SOURCE_DIR}
		${DIFF_PHYSICS_SOURCE_DIR}/python
        ${DIFF_PHYSICS_SOURCE_DIR}/third_party/pybind11/include
		${PYTHON_INCLUDE_DIRS}
    ${PYTHON_NUMPY_INCLUDE_DIR}
	../src
)


ADD_DEFINITIONS(-DSTATIC_LINK_SPD_PLUGIN)





ADD_LIBRARY(pytinydiffsim SHARED pytinydiffsim.cc ${TDS_HDRS})
TARGET_COMPILE_DEFINITIONS(pytinydiffsim PUBLIC -DENABLE_TEST_ENVS -DNOMINMAX )


ADD_LIBRARY(pytinydiffsim_dual SHARED pytinydiffsim_dual.cc ${TDS_HDRS})
IF(USE_CPPAD)
ADD_LIBRARY(pytinydiffsim_ad SHARED pytinydiffsim_ad.cc ${TDS_HDRS})
ENDIF(USE_CPPAD)


#SET_TARGET_PROPERTIES(pytinydiffsim PROPERTIES PREFIX "")
#SET_TARGET_PROPERTIES(pytinydiffsim PROPERTIES POSTFIX "")

#SET_TARGET_PROPERTIES(pytinydiffsim PROPERTIES VERSION ${DIFFPHYS_VERSION})
#SET_TARGET_PROPERTIES(pytinydiffsim PROPERTIES SOVERSION ${DIFFPHYS_VERSION})
#SET_TARGET_PROPERTIES(pytinydiffsim PROPERTIES DEBUG_POSTFIX "_d")


TARGET_LINK_LIBRARIES(pytinydiffsim tinyxml2 Eigen3::Eigen )
TARGET_LINK_LIBRARIES(pytinydiffsim_dual tinyxml2 Eigen3::Eigen )
IF(USE_CPPAD)
TARGET_LINK_LIBRARIES(pytinydiffsim_ad tinyxml2 Eigen3::Eigen CppADCodeGen)
IF (WIN32)
	TARGET_COMPILE_DEFINITIONS(pytinydiffsim_ad PUBLIC -D_USE_MATH_DEFINES)
ENDIF(WIN32)
ENDIF(USE_CPPAD)

IF (WIN32)
  TARGET_COMPILE_OPTIONS(pytinydiffsim PUBLIC -bigobj)
  MESSAGE("PYTHON_LIBRARIES")
  MESSAGE(${PYTHON_LIBRARIES})
  GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIR ${PYTHON_LIBRARIES} DIRECTORY)# CACHE)
  MESSAGE("PYTHON_LIBRARY_DIR ")
  MESSAGE(${PYTHON_LIBRARY_DIR})
  target_link_directories (pytinydiffsim PUBLIC "${PYTHON_LIBRARY_DIR}")
  TARGET_LINK_LIBRARIES(pytinydiffsim ${PYTHON_LIBRARIES})
ELSEIF (APPLE)
  SET_TARGET_PROPERTIES(pytinydiffsim PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
ENDIF ()

IF (WIN32)
  TARGET_COMPILE_OPTIONS(pytinydiffsim_dual PUBLIC -bigobj)
  
  GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIR ${PYTHON_LIBRARIES} DIRECTORY)# CACHE)
  
  target_link_directories (pytinydiffsim_dual PUBLIC "${PYTHON_LIBRARY_DIR}")
  TARGET_LINK_LIBRARIES(pytinydiffsim_dual ${PYTHON_LIBRARIES})
ELSEIF (APPLE)
  SET_TARGET_PROPERTIES(pytinydiffsim_dual PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
ENDIF ()


IF(USE_CPPAD)
    IF (WIN32)
      TARGET_COMPILE_OPTIONS(pytinydiffsim_ad PUBLIC -bigobj)

      GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIR ${PYTHON_LIBRARIES} DIRECTORY)# CACHE)

      target_link_directories (pytinydiffsim_ad PUBLIC "${PYTHON_LIBRARY_DIR}")
      TARGET_LINK_LIBRARIES(pytinydiffsim_ad ${PYTHON_LIBRARIES})
    ELSEIF (APPLE)
          SET_TARGET_PROPERTIES(pytinydiffsim_ad PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
    ENDIF ()
ENDIF(USE_CPPAD)



INCLUDE_DIRECTORIES( 
    ${DIFF_PHYSICS_SOURCE_DIR}
		${DIFF_PHYSICS_SOURCE_DIR}/python
		${PYTHON_INCLUDE_DIRS}
    ${PYTHON_NUMPY_INCLUDE_DIR}
	../src
)

##################################################################

SET(pytinyopengl3_SRCS
	pytinyopengl3.cc
)

ADD_LIBRARY(pytinyopengl3 SHARED ${pytinyopengl3_SRCS} ${TDS_HDRS})
TARGET_LINK_LIBRARIES(pytinyopengl3 tinyxml2 opengl_window)
target_include_directories(pytinyopengl3 PRIVATE ../src)


IF (WIN32)
  TARGET_COMPILE_OPTIONS(pytinyopengl3 PUBLIC -bigobj)
  MESSAGE("PYTHON_LIBRARIES")
  MESSAGE(${PYTHON_LIBRARIES})
  GET_FILENAME_COMPONENT(PYTHON_LIBRARY_DIR ${PYTHON_LIBRARIES} DIRECTORY)# CACHE)
  MESSAGE("PYTHON_LIBRARY_DIR ")
  MESSAGE(${PYTHON_LIBRARY_DIR})
  target_link_directories (pytinyopengl3 PUBLIC "${PYTHON_LIBRARY_DIR}")
  TARGET_LINK_LIBRARIES(pytinyopengl3 ${PYTHON_LIBRARIES} Opengl32)
  
ELSEIF (APPLE)
  SET_TARGET_PROPERTIES(pytinyopengl3 PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
ENDIF ()


