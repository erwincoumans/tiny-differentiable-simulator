add_executable(ars_train_policy ars_train_policy  ${TDS_HDRS})
target_link_libraries(ars_train_policy  tinyxml2 ${CMAKE_DL_LIBS}) 
target_compile_definitions(ars_train_policy PRIVATE USE_TINY )
target_include_directories(ars_train_policy PRIVATE ../../third_party ../../src )

add_executable(ars_run_policy ars_run_policy.cpp ../../third_party/stb_image/stb_image.cpp ../../third_party/tinyobjloader/tiny_obj_loader.cc  ${TDS_HDRS})
target_link_libraries(ars_run_policy  tinyxml2 opengl_window ${CMAKE_DL_LIBS}) 
target_compile_definitions(ars_run_policy PRIVATE USE_TINY )
target_include_directories(ars_run_policy PRIVATE ../../third_party ../../src )


