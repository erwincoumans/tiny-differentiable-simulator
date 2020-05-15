
INCLUDE_DIRECTORIES( include )
#ADD_DEFINITIONS( -DFD_SETSIZE=16384  )

ADD_LIBRARY(tinyxml2 tinyxml2.cpp include/tinyxml2.h)

target_include_directories(tinyxml2 PUBLIC include )




