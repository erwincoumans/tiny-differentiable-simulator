#first the examples that have no dependencies

add_executable(pendulum_example_gui pendulum_example_gui.cpp  ${TDS_HDRS})
target_link_libraries(pendulum_example_gui opengl_window)
target_include_directories(pendulum_example_gui PRIVATE ../src)

add_executable(tiny_urdf_parser_opengl_example tiny_urdf_parser_opengl_example.cpp ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS}) 
target_link_libraries(tiny_urdf_parser_opengl_example  tinyxml2 opengl_window) 
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party/tinyobjloader)
target_include_directories(tiny_urdf_parser_opengl_example PRIVATE ../third_party ../src)

add_executable(opengl_imgui_test opengl_imgui_test.cpp ../third_party/dear_imgui/opengl3/imgui_impl_opengl3.cpp  ${TDS_HDRS})
target_link_libraries(opengl_imgui_test opengl_window dear_imgui TinyAudio)
target_include_directories(opengl_imgui_test PRIVATE ../third_party/glad ../third_party/dear_imgui ../src)

add_executable(opengl_test opengl_test.cpp  ../third_party/stb_image/stb_image.cpp ../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS})
target_include_directories(opengl_test PRIVATE ../third_party/tinyobjloader)
target_include_directories(opengl_test PRIVATE ../third_party ../src)
target_link_libraries(opengl_test opengl_window)

IF(WIN32)
    target_link_libraries(pendulum_example_gui Opengl32)
    target_link_libraries(tiny_urdf_parser_opengl_example Opengl32)
    target_link_libraries(opengl_test  Opengl32)
    target_link_libraries(opengl_imgui_test Opengl32)
ENDIF(WIN32)


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

