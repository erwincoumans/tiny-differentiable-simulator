#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# $begin get_cppadcg.sh$$ $newlinech #$$
# $spell
#   gz
#   CppAD
#   cppadcg
#   Eigen
# $$
#
# $section Download and Install CppADCodeGen in Build Directory$$
#
# $head Syntax$$
# $code bin/get_cppadcg.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $href%https://github.com/joaoleal/CppADCodeGen%cppadcg%$$
# in the CppAD $code build$$ directory.
#
# $head Requirements$$
# You must first use $cref get_eigen.sh$$ to download and install Eigen.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The Cppadcg source code is downloaded into the sub-directory
# $code build/external/cppadcg.git$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional$$ is used this install.
#
# $head Git Hash$$
# This will install the commit of Cppadcg with the following git hash
# $srccode%sh%
git_hash='38d4f3b'
# %$$
# The date corresponding to this commit was 20200113.
#
# $head Configuration$$
# If the file
# $codei%
#   build/external/cppadcg-%git_hash%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
# $end
# -----------------------------------------------------------------------------
package='cppadcg'
if [ $0 != "bin/get_$package.sh" ]
then
    echo "bin/get_$package.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
web_page='https://github.com/joaoleal/CppADCodeGen.git'
cppad_dir=`pwd`
# -----------------------------------------------------------------------------
# prefix
eval `grep '^prefix=' bin/get_optional.sh`
if [[ "$prefix" =~ ^[^/] ]]
then
    prefix="$cppad_dir/$prefix"
fi
echo "prefix=$prefix"
# -----------------------------------------------------------------------------
configured_flag="build/external/$package-${git_hash}.configured"
echo "Executing get_$package.sh"
if [ -e "$configured_flag" ]
then
    echo "Skipping configuration because $configured_flag exits"
    echo_eval cd build/external/$package.git/build
    echo_eval make install
    echo "get_$package.sh: OK"
    exit 0
fi
# -----------------------------------------------------------------------------
if [ ! -d build/external ]
then
    echo_eval mkdir -p build/external
fi
echo_eval cd build/external
# -----------------------------------------------------------------------------
if [ ! -e $package.git ]
then
    echo_eval git clone $web_page $package.git
fi
# -----------------------------------------------------------------------------
echo_eval cd $package.git
echo_eval git checkout --quiet $git_hash
if [ ! -e build ]
then
    echo_eval mkdir build
fi
echo_eval cd build
echo_eval cmake \
    -D CPPAD_INCLUDE_DIRS='../include' \
    -D CMAKE_INSTALL_PREFIX=$prefix \
    -D EIGNE_INCLUDE_DIR=$prefix/include \
    -D GOOGLETEST_GIT=ON \
    ..
echo_eval make install
# -----------------------------------------------------------------------------
echo_eval touch $cppad_dir/$configured_flag
echo "get_$package.sh: OK"
