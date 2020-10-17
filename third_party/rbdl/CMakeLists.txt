
INCLUDE_DIRECTORIES (
  include
)

INCLUDE_DIRECTORIES( include )

IF(WIN32)
INCLUDE_DIRECTORIES( include/Windows )
ENDIF()

ADD_LIBRARY(rbdl-static 
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

target_include_directories( rbdl-static PUBLIC include ${EIGEN3_INCLUDE_DIRS})

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

