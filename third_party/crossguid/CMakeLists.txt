
ADD_LIBRARY(crossguid guid.cpp)

IF(WIN32)
	target_compile_definitions(crossguid PUBLIC GUID_WINDOWS PRIVATE GUID_WINDOWS)
ELSE()
IF(APPLE)
        find_library(CoreFoundation CoreFoundation)
	target_compile_definitions(crossguid PUBLIC GUID_CFUUID PRIVATE GGUID_CFUUID )
	target_link_libraries(crossguid PUBLIC ${CoreFoundation} PRIVATE ${CoreFoundation})
ELSE()
        target_compile_definitions(crossguid PUBLIC GUID_LIBUUID PRIVATE GUID_LIBUUID)
        target_link_libraries(crossguid PUBLIC uuid PRIVATE uuid)

ENDIF()
ENDIF()

target_include_directories(crossguid PUBLIC . PRIVATE .)


