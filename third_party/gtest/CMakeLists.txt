
INCLUDE_DIRECTORIES(
	.
	include
)

SET(gtest-1.7.0_SRCS
	src/gtest-all.cc
)

ADD_DEFINITIONS(-D_VARIADIC_MAX=10, -DGTEST_LANG_CXX11 )

FILE(GLOB_RECURSE gtest-1.7.0_HDRS "*.h")

ADD_LIBRARY(gtest ${gtest-1.7.0_SRCS} ${gtest-1.7.0_HDRS})

