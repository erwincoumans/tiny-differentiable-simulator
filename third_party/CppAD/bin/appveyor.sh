#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ $0 != "bin/appveyor.sh" ]
then
    echo 'bin/appveyor.sh: must be executed from its parent directory'
    exit 1
fi
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
echo_eval mkdir build
echo_eval cd build
echo_eval cmake \
    -G '"Unix Makefiles"' \
    -D CMAKE_C_COMPILER=gcc \
    -D CMAKE_CXX_COMPILER=g++ \
    ..
echo_eval make check
echo_eval make install
echo_eval make uninstall
# -----------------------------------------------------------------------------
echo 'bin/appveyor.sh: OK'
exit 0
