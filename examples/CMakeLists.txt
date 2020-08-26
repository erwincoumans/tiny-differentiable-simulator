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


include_directories(. ..)

set(DL ${CMAKE_DL_LIBS})

find_package(ct_core)
find_package(ct_optcon)
find_package(Ceres)
find_package(PkgConfig)

find_package(Torch)

add_definitions(-DBT_USE_DOUBLE_PRECISION)
find_package(Bullet)

#find_package(tinyxml2)
if (tinyxml2_FOUND)
    message("USING system tinyxml2")
    include_directories(${tinyxml2_INCLUDE_DIRS})
else (tinyxml2_FOUND)
    message ("USING third_party/tinyxml2")

    set(tinyxml2_FOUND TRUE)
    set(tinyxml2_INCLUDE_DIRS ../third_party/tinyxml2/include)
    SUBDIRS(../third_party/tinyxml2)
endif (tinyxml2_FOUND)


find_package(crossguid)
IF (crossguid_FOUND)
ELSE()
    SUBDIRS(../third_party/crossguid)
    set(crossguid_INCLUDE_DIRS ../third_party/crossguid)
    include_directories(${crossguid_INCLUDE_DIRS})
ENDIF()

#find_package(zeromq)
#find_package(cppzmq)

IF(cppzmq_FOUND)
    message("USING system zmq")
ELSE()
    message("USING third_party/zmq")
    SET(cppzmq_FOUND TRUE)
    SUBDIRS(../third_party/zeromq)
    set(cppzmq_INCLUDE_DIRS ../third_party/zeromq/include )
    include_directories(${cppzmq_INCLUDE_DIRS})
    add_definitions(-DZMQ_STATIC)
ENDIF()

find_package(nlohmann_json)
IF ( nlohmann_json_FOUND)
ELSE()
    SET(nlohmann_json_FOUND TRUE)
    set(nlohmann_json_INCLUDE_DIRS ../third_party/nlohmann/include)
    include_directories(${nlohmann_json_INCLUDE_DIRS})
ENDIF()

#find_library(zmq_location NAMES libzmq.a)
#message(STATUS ${zmq_location})
#add_library(zmq STATIC IMPORTED)
#set_target_properties(zmq PROPERTIES IMPORTED_LOCATION ${zmq_location})

# show all cmake variables, for debugging
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()



set(MESHCAT_LIBRARIES crossguid cpp_base64 zmq )
IF(WIN32)
    set(MESHCAT_LIBRARIES ${MESHCAT_LIBRARIES} ws2_32 Winmm iphlpapi)
ENDIF()
set(HAS_MESHCAT TRUE)


if (Bullet_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    include_directories(${BULLET_INCLUDE_DIRS})
    add_library(pybullet_visualizer_api STATIC pybullet_visualizer_api.cpp)
    target_link_libraries(pybullet_visualizer_api BulletRoboticsGUI BulletExampleBrowserLib
            BulletRobotics BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics
            BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath
            OpenGLWindow  gwen BussIK Bullet3Common pthread ${DL})

    if (Ceres_FOUND)
        FIND_PACKAGE(PythonLibs REQUIRED)
        INCLUDE_DIRECTORIES( 
            ${PYTHON_INCLUDE_DIRS}
            ${PYTHON_NUMPY_INCLUDE_DIR}
        )

        add_executable(billiard_opt_gui billiard_opt_gui.cpp ${TDS_HDRS})
        target_link_libraries(billiard_opt_gui pybullet_visualizer_api ${DL}
                ${CERES_LIBRARIES})
        target_compile_definitions(billiard_opt_gui PRIVATE STAN_MATH)

        add_executable(cartpole_opt cartpole_opt.cpp ${TDS_HDRS})
        target_link_libraries(cartpole_opt pybullet_visualizer_api ${DL}
                ${CERES_LIBRARIES})
# broken targets
#        add_executable(ceres_estimation ceres_estimation.cpp ${TDS_HDRS})
#        target_link_libraries(ceres_estimation pybullet_visualizer_api ${DL}
#                ${CERES_LIBRARIES})

#        # set(DUSE_MATPLOTLIB true)
#        add_executable(ceres_estimation_real ceres_estimation_real.cpp ${TDS_HDRS})
#        target_link_libraries(ceres_estimation_real pybullet_visualizer_api ${DL}
#                ${CERES_LIBRARIES} python2.7)

        add_executable(neural_pendulum neural_pendulum.cpp ${TDS_HDRS})
        target_link_libraries(neural_pendulum pybullet_visualizer_api ${DL}
                ${CERES_LIBRARIES})

        add_executable(policy_optimization policy_optimization.cpp ${TDS_HDRS})
        target_link_libraries(policy_optimization pybullet_visualizer_api ${DL}
                ${CERES_LIBRARIES})

        add_executable(contact_discovery contact_discovery.cpp ${TDS_HDRS})
        target_link_libraries(contact_discovery pybullet_visualizer_api ${DL}
                ${CERES_LIBRARIES})
    endif (Ceres_FOUND)

    if (ct_core_FOUND AND ct_optcon_FOUND AND Eigen3_FOUND)
        # Activate CppAD (with CodeGen) and HPIPM for Control Toolbox
        add_definitions(-DCPPAD -DCPPADCG -DHPIPM)
    
        include_directories(${ct_core_INCLUDE_DIRS})
        include_directories(${ct_optcon_INCLUDE_DIRS})
        include_directories(${EIGEN3_INCLUDE_DIRS})

        add_executable(ct_cartpole ct_cartpole.cpp ${TDS_HDRS})
        target_link_libraries(ct_cartpole pybullet_visualizer_api ${DL}
                -lpthread hpipm blasfeo)

        add_executable(ct_contact_trajopt ct_contact_trajopt.cpp ${TDS_HDRS})
        target_link_libraries(ct_contact_trajopt pybullet_visualizer_api ${DL}
                -lpthread hpipm blasfeo)

        add_executable(ct_laikago_tracking ct_laikago_tracking.cpp ${TDS_HDRS})
        target_link_libraries(ct_laikago_tracking pybullet_visualizer_api ${DL}
                -lpthread hpipm blasfeo)
    endif (ct_core_FOUND AND ct_optcon_FOUND AND Eigen3_FOUND)

    add_executable(laikago_example_gui laikago_example_gui.cpp ${TDS_HDRS})
    target_link_libraries(laikago_example_gui pybullet_visualizer_api)

    if (Eigen3_FOUND)
      add_executable(laikago_footstep_ik laikago_footstep_ik.cpp ${TDS_HDRS})
      target_link_libraries(laikago_footstep_ik pybullet_visualizer_api)

      add_executable(pendulum_ik_gui pendulum_ik_gui.cpp)
      target_link_libraries(pendulum_ik_gui pybullet_visualizer_api)
    endif()

    
    add_executable(generic_urdf_collision generic_urdf_collision.cpp ${TDS_HDRS})
    target_link_libraries(generic_urdf_collision pybullet_visualizer_api)

    add_executable(gravity_compensation gravity_compensation.cpp ${TDS_HDRS})
    target_link_libraries(gravity_compensation pybullet_visualizer_api)

if (HAS_MESHCAT)
    add_executable(laikago_meshcat_example laikago_meshcat_example.cpp ${TDS_HDRS})
    target_link_libraries(laikago_meshcat_example pybullet_visualizer_api ${MESHCAT_LIBRARIES})
endif (HAS_MESHCAT)

if (Torch_FOUND)
    include_directories(${TORCH_INCLUDE_DIRS})
    message("Torch libraries: ${TORCH_LIBRARIES}")
    add_executable(torch_example torch_example.cpp ${TDS_HDRS})
    target_link_libraries(torch_example pybullet_visualizer_api ${TORCH_LIBRARIES})
endif (Torch_FOUND)

endif (Bullet_FOUND)


if (HAS_MESHCAT AND tinyxml2_FOUND)    

    add_executable(tiny_urdf_parser_meshcat_example tiny_urdf_parser_meshcat_example.cpp ${TDS_HDRS}) 
    target_link_libraries(tiny_urdf_parser_meshcat_example  tinyxml2 ${MESHCAT_LIBRARIES}) 

    add_executable(meshcat_zmq_example meshcat_zmq_example.cpp ${TDS_HDRS})
    target_link_libraries(meshcat_zmq_example  ${MESHCAT_LIBRARIES})

endif (HAS_MESHCAT AND tinyxml2_FOUND)

if (tinyxml2_FOUND)
    add_executable(tiny_urdf_parser_example tiny_urdf_parser_example.cpp ${TDS_HDRS})
    target_link_libraries(tiny_urdf_parser_example tinyxml2)

if (Ceres_FOUND)
    add_executable(neural_contact neural_contact.cpp ${TDS_HDRS})
    target_link_libraries(neural_contact opengl_window ${DL}
            ${CERES_LIBRARIES} ${MESHCAT_LIBRARIES} python2.7 tinyxml2)
endif (Ceres_FOUND)
endif (tinyxml2_FOUND)

add_executable(world_example world_example.cpp ${TDS_HDRS})
add_executable(test_xarm test_xarm.cpp ${TDS_HDRS})

subdirs(opengl_window)

add_executable(pendulum_example_gui pendulum_example_gui.cpp utils/tiny_clock.cpp utils/tiny_chrome_trace_util.cpp utils/tiny_logging.cpp ${TDS_HDRS})
target_link_libraries(pendulum_example_gui opengl_window)

add_executable(opengl_test opengl_test.cpp  ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc utils/tiny_clock.cpp utils/tiny_chrome_trace_util.cpp utils/tiny_logging.cpp ${TDS_HDRS})
target_include_directories(opengl_test PRIVATE ../third_party/tinyobjloader)
target_include_directories(opengl_test PRIVATE ../third_party)



add_executable(tiny_urdf_parser_opengl_example tiny_urdf_parser_opengl_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc utils/tiny_clock.cpp utils/tiny_chrome_trace_util.cpp utils/tiny_logging.cpp ${TDS_HDRS}) 
target_link_libraries(tiny_urdf_parser_opengl_example  tinyxml2 opengl_window) 
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party/tinyobjloader)
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party)



target_link_libraries(opengl_test opengl_window)

subdirs(../third_party/dear_imgui)

add_executable(opengl_imgui_test opengl_imgui_test.cpp ../third_party/dear_imgui/opengl3/imgui_impl_opengl3.cpp utils/tiny_clock.cpp utils/tiny_chrome_trace_util.cpp utils/tiny_logging.cpp ${TDS_HDRS})
target_link_libraries(opengl_imgui_test opengl_window dear_imgui)
target_include_directories(opengl_imgui_test PRIVATE ../third_party/glad ../third_party/dear_imgui )


IF(WIN32)
    target_link_libraries(pendulum_example_gui Opengl32)
    target_link_libraries(tiny_urdf_parser_opengl_example Opengl32)
    target_link_libraries(opengl_test  Opengl32)
    target_link_libraries(opengl_imgui_test Opengl32)
ENDIF()