
ADD_LIBRARY(crossguid guid.cpp)

IF(WIN32)
	target_compile_definitions(crossguid PUBLIC GUID_WINDOWS PRIVATE GUID_WINDOWS)
ENDIF()

target_include_directories(crossguid PUBLIC . PRIVATE .)


