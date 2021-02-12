@set ROOT=%cd%
@echo root= %ROOT%

git submodule update --init --recursive

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



pushd third_party\glog
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_CXX_FLAGS="/MP" -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -Dgflags_DIR=%ROOT%/third_party/gflags/build_cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%\third_party\glog\build_cmake\local_install ..
cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL  --config Release
mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  Release local_install\lib\Release
copy  Debug local_install\lib\Debug

popd
cd %ROOT%


pushd third_party\cppadcodegen
git apply ..\patches\CppADCodeGen.diff
popd
cd %ROOT%




pushd third_party\eigen3
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
cmake  -DCMAKE_CXX_FLAGS="/MP" -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_TESTING=OFF -Dgflags_DIR=%ROOT%/third_party/gflags/build_cmake -Dglog_DIR=%ROOT%/third_party/glog/build_cmake -DEigen3_DIR=%ROOT%/third_party/eigen3/build_cmake  -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=%ROOT%/third_party/ceres-solver/build_cmake/local_install ..
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



pushd third_party\boost
call bootstrap.bat
b2 --with-serialization
popd
cd %ROOT%


pushd third_party\oneTBB
mkdir build_cmake
cd build_cmake
cmake -DCMAKE_INSTALL_PREFIX:PATH=local_install -DTBB_TEST=OFF -DBUILD_SHARED_LIBS=OFF ..
cmake  --build .  --target INSTALL  --config Debug
cmake  --build .  --target INSTALL  --config Release
popd
cd %ROOT%


pushd third_party\pagmo2
git apply ..\pagmo2_msvc.diff
mkdir build_cmake
cd build_cmake
cmake -DCMAKE_INSTALL_PREFIX:PATH=local_install -DPAGMO_WITH_EIGEN3=ON -DBoost_DIR:PATH=%ROOT%/third_party/boost/stage/lib/cmake/Boost-1.75.0 -DEigen3_DIR:PATH=%ROOT%/third_party/eigen/build_cmake -DTBB_VERSION=2021.1.0 -DPAGMO_BUILD_STATIC_LIBRARY=ON -DTBB_ROOT=%ROOT%/third_party/oneTBB/build_cmake/local_install ..
cmake  --build .  --target INSTALL  --config Debug
cmake  --build .  --target INSTALL  --config Release
popd
cd %ROOT%




mkdir build_cmake
cd build_cmake

cmake  -DCMAKE_CXX_FLAGS="/MP" -DUSE_CERES=ON -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DBullet_DIR=%ROOT%\third_party\bullet3\build_cmake -Dgflags_DIR=%ROOT%\third_party\gflags\build_cmake -Dglog_DIR=%ROOT%\third_party\glog\build_cmake -DEigen3_DIR=%ROOT%\third_party\eigen3\build_cmake -DCeres_DIR=%ROOT%\third_party\ceres-solver\build_cmake\local_install\cmake -DUSE_CPPAD=ON -DPAGMO_WITH_EIGEN3=ON -DBoost_DIR:PATH=%ROOT%/third_party/boost/stage/lib/cmake/Boost-1.75.0 -DTBB_VERSION=2021.1.0 -DPagmo_DIR:PATH=%ROOT%/third_party/pagmo2/build_cmake/local_install/lib/cmake/pagmo -DTBB_ROOT=%ROOT%/third_party/oneTBB/build_cmake/local_install ..

rem cmake  --build .  --target INSTALL  --config Release
rem start DIFF_PHYSICS.sln
