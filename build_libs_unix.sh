@set ROOT=%cd%
@echo root= %ROOT%
ROOT=$(pwd)
echo $ROOT

git submodule update --init --recursive

pushd third_party/eigen
mkdir build_cmake
cd build_cmake
cmake -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..
make -j
make install
popd
cd $ROOT

pushd third_party/bullet3
mkdir build_cmake
cd build_cmake

cmake  -DUSE_DOUBLE_PRECISION=ON -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..
make -j
make install

popd
cd $ROOT

pushd third_party/glog
mkdir build_cmake
cd build_cmake
cmake  -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=$ROOT/third_party/glog/build_cmake/local_install ..
make -j
make install
popd
cd $ROOT


pushd third_party/gflags
mkdir build_cmake
cd build_cmake
cmake   -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=$ROOT/third_party/gflags/build_cmake/local_install ..
make -j
make install
popd
cd $ROOT


pushd third_party/ceres-solver
mkdir build_cmake
cd build_cmake
cmake  -DBUILD_TESTING=OFF -Dgflags_DIR=$ROOT/third_party/gflags/build_cmake -Dglog_DIR=$ROOT/third_party/glog/build_cmake -DEigen3_DIR=$ROOT/third_party/eigen/build_cmake  -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX:PATH=$ROOT/third_party/ceres-solver/build_cmake/local_install ..
make -j
make install
popd
cd $ROOT

mkdir build_cmake
cd build_cmake

cmake  -DBullet_DIR=$ROOT/third_party/bullet3/build_cmake -Dgflags_DIR=$ROOT/third_party/gflags/build_cmake -Dglog_DIR=$ROOT/third_party/glog/build_cmake -DEigen3_DIR=$ROOT/third_party/eigen/build_cmake -DCeres_DIR=$ROOT/third_party/ceres-solver/build_cmake/local_install/lib/cmake/Ceres ..
make -j

