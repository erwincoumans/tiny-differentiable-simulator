@set ROOT=%cd%
@echo root= %ROOT%

git submodule update --init --recursive


pushd third_party\glog
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%\third_party\glog\build_cmake\local_install ..
cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  lib\Release local_install\lib\Release
copy  lib\Debug local_install\lib\Debug

popd
cd %ROOT%


pushd third_party\gflags
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%/third_party/gflags/build_cmake/local_install ..
cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  lib\Release local_install\lib\Release
copy  lib\Debug local_install\lib\Debug

popd
cd %ROOT%



pushd third_party\eigen
mkdir build_cmake
cd build_cmake
cmake -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install   ..
cmake  --build .  --target INSTALL  --config Release
popd
cd %ROOT%

pushd third_party\bullet3
mkdir build_cmake
cd build_cmake

cmake -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DCMAKE_CXX_FLAGS="/MP" -DUSE_DOUBLE_PRECISION=ON -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..

cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  lib\Release local_install\lib\Release
copy  lib\Debug local_install\lib\Debug

popd
cd %ROOT%


pushd third_party\ceres-solver
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TESTING=OFF -Dgflags_DIR=%ROOT%/third_party/gflags/build_cmake -Dglog_DIR=%ROOT%/third_party/glog/build_cmake -DEigen3_DIR=%ROOT%/third_party/eigen/build_cmake  -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%/third_party/ceres-solver/build_cmake/local_install ..
cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  lib\Release local_install\lib\Release
copy  lib\Debug local_install\lib\Debug
popd
cd %ROOT%

rem del third_party\ceres-solver\build_cmake\local_install\lib\*.lib
del third_party\bullet3\build_cmake\local_install\lib\*.lib
del third_party\glog\build_cmake\local_install\lib\*.lib
del third_party\gflags\build_cmake\local_install\lib\*.lib


mkdir build_cmake
cd build_cmake

cmake  -DCMAKE_CXX_FLAGS="/MP" -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DBullet_DIR=%ROOT%\third_party\bullet3\build_cmake -Dgflags_DIR=%ROOT%\third_party\gflags\build_cmake -Dglog_DIR=%ROOT%\third_party\glog\build_cmake -DEigen3_DIR=%ROOT%\third_party\eigen\build_cmake -DCeres_DIR=%ROOT%\third_party\ceres-solver\build_cmake\local_install\cmake -DUSE_CPPAD=ON ..

cmake  --build .  --target INSTALL  --config Release
start DIFF_PHYSICS.sln