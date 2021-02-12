#!/bin/sh
ROOT=$(pwd)
echo $ROOT
install_name_tool -change @rpath/libboost_serialization.dylib $ROOT/third_party/boost/stage/lib/libboost_serialization.dylib $ROOT/build_cmake/examples/pagmo2_example
install_name_tool -change @rpath/libpagmo.6.dylib $ROOT/third_party/pagmo2/build_cmake/local_install/lib/libpagmo.6.0.dylib $ROOT/build_cmake/examples/pagmo2_example
install_name_tool -change @rpath/libtbb.12.dylib $ROOT/third_party/oneTBB/build_cmake/local_install/lib/libtbb.12.2.dylib $ROOT/build_cmake/examples/pagmo2_example
install_name_tool -change @rpath/libtbb.12.dylib $ROOT/third_party/oneTBB/build_cmake/local_install/lib/libtbb.12.2.dylib $ROOT/third_party/pagmo2/build_cmake/local_install/lib/libpagmo.6.0.dylib

