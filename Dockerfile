FROM ubuntu:18.04

ENV LANG C.UTF-8

RUN \
  apt-get -y -q update && \
  # Prevents debconf from prompting for user input
  # See https://github.com/phusion/baseimage-docker/issues/58
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    gcc \
    g++ \
    build-essential \
    cmake \
    wget \
    curl \
    unzip \
    git \
    git-lfs \
    python-dev \
    python-pip \
    python3-dev \
    python3-pip \
    libeigen3-dev \
    libceres-dev \
    libtinyxml2-dev \
    libicu-dev \
    libbz2-dev \
    freeglut3-dev \
    ffmpeg \
    libhdf5-dev \
    nano \
    htop \
    tmux \
    libcrossguid-dev \
    libgtest-dev \
    libnlopt-dev \
    libnlopt0 \
    libtbb-dev \
    libboost-all-dev


# Install llvm requirements
RUN \
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
  llvm-10 \
  llvm-10-dev \
  libllvm10 \
  libllvm-10-ocaml-dev \
  libclang-common-10-dev \
  libclang-10-dev \
  libclang1-10 \
  clang-format-10 \
  python3-clang-10 \
  clang-10 \
  clang-tools-10 \
  lld-10 \
  libssl-dev

# apt install python-pip2
# apt install python-pip
# apt install cmake
# apt install uuid-dev
# apt-get install libhdf5-dev
# apt search crossguid
# apt install libcrossguid-dev
# apt search boost
# apt install libboost-dev
# apt install libboost-system
# apt search boost-system
# apt install libboost-system-dev
# apt install libboost-filesystem-dev
# apt install libboost-serialization-dev
# apt search gcc
# apt install g++
# apt install gcc-9
# apt install gcc-8
# apt-get install manpages-dev
# apt install software-properties-common
# add-apt-repository ppa:ubuntu-toolchain-r/test
# apt install gcc-7 g++-7 gcc-8 g++-8 gcc-9 g++-9
# apt install clang-9
# apt search numpy
# apt install python-numpy
# apt install -y cgdb
# apt install -y vim-nox
# apt install valgrind

# Install latest version of CMake (not available from APT repo)
RUN mkdir -p /root/cmake_install
WORKDIR /root/cmake_install
RUN wget https://github.com/Kitware/CMake/releases/download/v3.18.3/cmake-3.18.3.tar.gz
RUN tar -zxf cmake-3.18.3.tar.gz
WORKDIR /root/cmake_install/cmake-3.18.3
RUN ./bootstrap && make && make install


RUN mkdir -p /root/code

# Copy tiny-differentiable-simulator folder
COPY . /root/code/tiny-differentiable-simulator

# Get python dependencies
WORKDIR /root/code/tiny-differentiable-simulator

# Trying to resolve dependency issues...
RUN pip3 install numpy==1.16.0

# Probably need to point LLVM_CONFIG to the right path
ENV LLVM_CONFIG=/usr/bin/llvm-config-10

RUN pip3 install -r python/requirements.txt

# Check out git submodules
# WORKDIR /root/code/tiny-differentiable-simulator
# RUN git submodule update --init --recursive

# #  Build TDS
# WORKDIR /root/code/tiny-differentiable-simulator
# RUN rm -rf build && cmake -Bbuild . && make -C build experiment_neural_swimmer

# Set up git lfs
RUN git lfs install

# Install all dependencies
RUN mkdir /root/deps

# Ceres
WORKDIR /root/deps
RUN git clone https://github.com/ceres-solver/ceres-solver.git
WORKDIR /root/deps/ceres-solver
RUN git checkout 1.14.x
RUN mkdir build 
WORKDIR /root/deps/ceres-solver/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j && make install

# Bullet
WORKDIR /root/deps
RUN git clone https://github.com/bulletphysics/bullet3.git
WORKDIR /root/deps/bullet3
RUN ./build_cmake_pybullet_double.sh 
WORKDIR /root/deps/bullet3/build_cmake
RUN make install 


WORKDIR /root/deps 
RUN wget -q https://github.com/stevengj/nlopt/archive/v2.6.2.tar.gz 
RUN tar -zxf v2.6.2.tar.gz 
WORKDIR /root/deps/nlopt-2.6.2
RUN mkdir build 
WORKDIR /root/deps/nlopt-2.6.2/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make && make install 

# PAGMO
WORKDIR /root/deps 
RUN git clone https://github.com/esa/pagmo2.git 
WORKDIR /root/deps/pagmo2 
RUN mkdir build 
WORKDIR /root/deps/pagmo2/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. -DPAGMO_WITH_NLOPT=ON \
      # -DNLopt_DIR="/usr/lib/x86_64-linux-gnu/cmake/nlopt/" \ 
      && cmake -DCMAKE_BUILD_TYPE=Release --build . && cmake --build . --target install


WORKDIR /usr/src/googletest/googletest
RUN mkdir build 
WORKDIR /usr/src/googletest/googletest/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make && make install

# Setup repo
WORKDIR /root/code/tiny-differentiable-simulator

# In case stan-math is not built 
WORKDIR /root/code/tiny-differentiable-simulator/third_party/stan_math
RUN make -f ./make/standalone math-libs 

# Run estimation
WORKDIR /root/code/tiny-differentiable-simulator
RUN mkdir build
WORKDIR /root/code/tiny-differentiable-simulator/build 
RUN cmake -DCMAKE_BUILD_TYPE=Release .. 
RUN make test_pagmo_estimation

CMD [ "./test/test_pagmo_estimation" ]
