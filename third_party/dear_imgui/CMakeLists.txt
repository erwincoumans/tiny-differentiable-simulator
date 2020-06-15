
ADD_LIBRARY(dear_imgui imgui.cpp imgui_demo.cpp imgui_draw.cpp imgui_widgets.cpp)

IF(WIN32)
	target_compile_definitions(dear_imgui PUBLIC GUID_WINDOWS PRIVATE GUID_WINDOWS)
ELSE()
IF(APPLE)
	target_compile_definitions(dear_imgui PUBLIC GUID_LIBUUID PRIVATE GUID_LIBUUID)
ENDIF()
ENDIF()

target_include_directories(dear_imgui PUBLIC . PRIVATE .)


