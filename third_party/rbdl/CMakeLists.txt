
INCLUDE_DIRECTORIES (
  include
)

INCLUDE_DIRECTORIES( include )

IF(WIN32)
INCLUDE_DIRECTORIES( include/Windows )
ENDIF()

ADD_LIBRARY(rbdl-static STATIC 
 src/rbdl_version.cc
  src/rbdl_mathutils.cc
  src/rbdl_utils.cc
  src/Constraints.cc
  src/Dynamics.cc
  src/Logging.cc
  src/Joint.cc
  src/Model.cc
  src/Kinematics.cc
)

target_include_directories( rbdl-static PUBLIC include )
target_link_libraries( rbdl-static PUBLIC Eigen3::Eigen)


IF(WIN32)
target_compile_definitions( rbdl-static PUBLIC -DNOMINMAX PRIVATE -DNOMINMAX )
TARGET_COMPILE_OPTIONS(rbdl-static PUBLIC -bigobj)
target_include_directories( rbdl-static PUBLIC include/Windows )
ELSE(WIN32)
IF(APPLE)
target_include_directories( rbdl-static PUBLIC include/Mac )
ELSE(APPLE)
target_include_directories( rbdl-static PUBLIC include/Linux )
ENDIF(APPLE)
ENDIF(WIN32)



ADD_LIBRARY(rbdl-urdfreader STATIC 
		addons/urdfreader/urdfreader.cc
	    addons/urdfreader/urdf/urdfdom/urdf_parser/src/check_urdf.cpp	
		addons/urdfreader/urdf/urdfdom/urdf_parser/src/pose.cpp
		addons/urdfreader/urdf/urdfdom/urdf_parser/src/model.cpp
		addons/urdfreader/urdf/urdfdom/urdf_parser/src/link.cpp
		addons/urdfreader/urdf/urdfdom/urdf_parser/src/joint.cpp
		addons/urdfreader/urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h
		addons/urdfreader/urdf/urdfdom_headers/urdf_exception/include/urdf_exception/exception.h	
		addons/urdfreader/urdf/urdfdom_headers/urdf_model/include/urdf_model/pose.h
		addons/urdfreader/urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h
		addons/urdfreader/urdf/urdfdom_headers/urdf_model/include/urdf_model/link.h
		addons/urdfreader/urdf/urdfdom_headers/urdf_model/include/urdf_model/joint.h
		addons/urdfreader/tinyxml/tinystr.cpp
		addons/urdfreader/tinyxml/tinyxml.cpp
		addons/urdfreader/tinyxml/tinyxmlerror.cpp
		addons/urdfreader/tinyxml/tinyxmlparser.cpp
		addons/urdfreader/urdf/boost_replacement/lexical_cast.h
		addons/urdfreader/urdf/boost_replacement/shared_ptr.h
		addons/urdfreader/urdf/boost_replacement/printf_console.cpp
		addons/urdfreader/urdf/boost_replacement/printf_console.h
		addons/urdfreader/urdf/boost_replacement/string_split.cpp
		addons/urdfreader/urdf/boost_replacement/string_split.h
		)


target_include_directories( rbdl-urdfreader PUBLIC include addons/urdfreader )
target_link_libraries( rbdl-urdfreader PUBLIC Eigen3::Eigen)

IF(WIN32)
  target_compile_definitions( rbdl-urdfreader PUBLIC -DNOMINMAX PRIVATE -DNOMINMAX )
  TARGET_COMPILE_OPTIONS(rbdl-urdfreader PUBLIC -bigobj)
  target_include_directories( rbdl-urdfreader PUBLIC include/Windows )
ELSE(WIN32)
IF(APPLE)
  target_include_directories( rbdl-urdfreader PUBLIC include/Mac )
ELSE(APPLE)
  target_include_directories( rbdl-urdfreader PUBLIC include/Linux )
ENDIF(APPLE)
ENDIF(WIN32)
