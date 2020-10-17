
IF(USE_RBDL AND Bullet_FOUND)

  if (Bullet_FOUND)
    add_executable(test_rbdl_bt test_rbdl.cpp)
    set_target_properties(test_rbdl_bt PROPERTIES FOLDER autodiff)
    target_link_libraries(test_rbdl_bt PUBLIC pybullet_visualizer_api)
    target_link_libraries(test_rbdl_bt PRIVATE tinyxml2 ${PYTHON_LIBRARIES})
    target_link_libraries(test_rbdl_bt PRIVATE gtest rbdl-static)
    target_include_directories(test_rbdl_bt PUBLIC ${BULLET_INCLUDE_DIR} ../src . ../third_party/gtest/include ../third_party/eigen3)
    target_compile_definitions(test_rbdl_bt PUBLIC ${BULLET_DEFINITIONS} -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL -DUSE_BULLET_URDF_PARSER)
    target_link_directories(test_rbdl_bt PUBLIC ${Bullet_DIR}/${BULLET_ROOT_DIR}/${BULLET_LIBRARY_DIRS})
    target_link_libraries(test_rbdl_bt PUBLIC BulletDynamics BulletCollision LinearMath)
  endif()
  
  add_executable(test_rbdl_tds test_rbdl.cpp)
  set_target_properties(test_rbdl_tds PROPERTIES FOLDER autodiff)
  target_link_libraries(test_rbdl_tds PRIVATE tinyxml2 ${PYTHON_LIBRARIES})
  target_link_libraries(test_rbdl_tds PRIVATE gtest rbdl-static)
  target_include_directories(test_rbdl_tds PUBLIC  ../src . ../third_party/gtest/include ../third_party/eigen3)
  target_compile_definitions(test_rbdl_tds PUBLIC -D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 -DUSE_RBDL )
  
ENDIF()


IF(USE_RBDL)
  
  
ENDIF()

