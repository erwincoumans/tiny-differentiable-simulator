
IF(USE_RBDL)

  if (Bullet_FOUND)
    add_executable(test_rbdl_bt test_rbdl.cpp)
    set_target_properties(test_rbdl_bt PROPERTIES FOLDER autodiff)
    target_link_libraries(test_rbdl_bt PUBLIC pybullet_visualizer_api)
    target_link_libraries(test_rbdl_bt PRIVATE tinyxml2 ${PYTHON_LIBRARIES})
    target_link_libraries(test_rbdl_bt PRIVATE gtest rbdl-static Eigen3::Eigen)
    target_include_directories(test_rbdl_bt PUBLIC ${BULLET_INCLUDE_DIR} ../src . ../third_party/gtest/include )
    target_compile_definitions(test_rbdl_bt PUBLIC ${BULLET_DEFINITIONS} -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL -DUSE_BULLET_URDF_PARSER)
    target_link_directories(test_rbdl_bt PUBLIC ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
    target_link_libraries(test_rbdl_bt PUBLIC BulletDynamics BulletCollision LinearMath)
    ADD_TEST(test_rbdl_bt_PASS test_rbdl_bt) 
  endif(Bullet_FOUND)
  
  IF(True)
    add_executable(test_rbdl_tds test_rbdl.cpp)
    set_target_properties(test_rbdl_tds PROPERTIES FOLDER autodiff)
    target_link_libraries(test_rbdl_tds PRIVATE tinyxml2 Eigen3::Eigen ${PYTHON_LIBRARIES})
    target_link_libraries(test_rbdl_tds PRIVATE gtest rbdl-static -lpthread)
    target_include_directories(test_rbdl_tds PUBLIC  ../src . ../third_party/gtest/include )
    target_compile_definitions(test_rbdl_tds PUBLIC -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL )
    ADD_TEST(test_rbdl_tds_PASS test_rbdl_tds)  

    add_executable(test_rbdl_tds_left_associative test_rbdl.cpp)
    set_target_properties(test_rbdl_tds_left_associative PROPERTIES FOLDER autodiff)
    target_link_libraries(test_rbdl_tds_left_associative PRIVATE tinyxml2 Eigen3::Eigen ${PYTHON_LIBRARIES})
    target_link_libraries(test_rbdl_tds_left_associative PRIVATE gtest rbdl-static -lpthread)
    target_include_directories(test_rbdl_tds_left_associative PUBLIC  ../src . ../third_party/gtest/include )
    target_compile_definitions(test_rbdl_tds_left_associative PUBLIC -DTDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL )
    ADD_TEST(test_rbdl_tds_left_associative_PASS test_rbdl_tds)  
  ENDIF(False)
  


  add_executable(test_spatial test_spatial.cpp)
  target_link_libraries(test_spatial PRIVATE gtest gtestmain Eigen3::Eigen tinyxml2 rbdl-static)
  target_include_directories(test_spatial PRIVATE ../src)
  target_compile_definitions(test_spatial PUBLIC -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL )
  add_test(test_spatial_PASS test_spatial)

  add_executable(test_spatial_left_associative test_spatial.cpp)
  target_link_libraries(test_spatial_left_associative PRIVATE gtest gtestmain Eigen3::Eigen tinyxml2 rbdl-static)
  target_include_directories(test_spatial_left_associative PRIVATE ../src)
  target_compile_definitions(test_spatial_left_associative PUBLIC -DTDS_USE_LEFT_ASSOCIATIVE_TRANSFORMS -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL )
  add_test(test_spatial_left_associative_PASS test_spatial)



ENDIF(USE_RBDL)

IF (USE_CPPAD)
    add_executable(test_gradients test_gradients.cpp)
    target_link_libraries(test_gradients PRIVATE differentiation gtest gtestmain Eigen3::Eigen)
    target_include_directories(test_gradients PRIVATE ../src)
    add_test(GradientTests test_gradients)

    #build is often broken
    #if (NOT WIN32)
    #    add_executable(test_codegen test_codegen.cpp)
    #    target_link_libraries(test_codegen PRIVATE differentiation gtest gtestmain tinyxml2 Eigen3::Eigen ${CMAKE_DL_LIBS})
    #    target_include_directories(test_codegen PRIVATE ../src )
    #    add_test(CodegenTests test_codegen)
    #ENDIF()
endif (USE_CPPAD)
