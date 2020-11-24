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
# $begin get_colpack.sh$$ $newlinech #$$
# $spell
#   colpack
#   gz
#   Col
#   CppAD
# $$
#
# $section Download and Install ColPack in Build Directory$$
#
# $head Syntax$$
# $code bin/get_colpack.sh$$
#
# $head Purpose$$
# If you are using Unix, this command will download and install
# $cref/Colpack/colpack_prefix/Colpack Home Page/$$
# in the CppAD $code build$$ directory.
#
# $head Distribution Directory$$
# This command must be executed in the
# $cref/distribution directory/download/Distribution Directory/$$.
#
# $head Source Directory$$
# The ColPack source code is downloaded into the sub-directory
# $code build/external/colpack.git$$ below the distribution directory.
#
# $head Prefix$$
# The $cref/prefix/get_optional.sh/prefix/$$
# in the file $code bin/get_optional$$ is used this install.
#
# $head Version$$
# This will install the following version of ColPack
# $srccode%sh%
version='1.0.10'
# %$$
#
# $head Configuration$$
# If the file
# $codei%
#   build/external/colpack-%version%.configured
# %$$
# exists, the configuration will be skipped.
# Delete this file if you want to re-run the configuration.
#
# $end
# -----------------------------------------------------------------------------
package='colpack'
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
web_page='https://github.com/CSCsw/ColPack.git'
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
configured_flag="build/external/$package-${version}.configured"
echo "Executing get_$package.sh"
if [ -e "$configured_flag" ]
then
    echo "Skipping configuration because $configured_flag exits"
    echo_eval cd build/external/$package.git
    echo_eval make install
    echo "get_$package.sh: OK"
    exit 0
fi
# --------------------------------------------------------------------------
if [ -e /usr/lib64 ]
then
    libdir='lib64'
else
    libdir='lib'
fi
# -----------------------------------------------------------------------------
if [ ! -d build/external ]
then
    echo_eval mkdir -p build/external
fi
echo_eval cd build/external
# -----------------------------------------------------------------------------
if [ ! -e "$package.git" ]
then
    echo_eval git clone $web_page $package.git
fi
# -----------------------------------------------------------------------------
echo_eval cd $package.git
echo_eval git checkout --quiet v$version
# -----------------------------------------------------------------------------
if which autoconf >& /dev/null
then
    echo_eval libtoolize
    echo_eval autoreconf --install --force
fi
# -----------------------------------------------------------------------------
system_name=`uname | sed -e 's|\(......\).*|\1|'`
if [ "$system_name" == 'CYGWIN' ]
then
    lib_type='--enable-static --disable-shared'
    echo_eval ls -l ./configure
    echo_eval chmod +x ./configure
else
    lib_type='--enable-static --enable-shared'
fi
echo_eval ./configure \
    --prefix=$prefix \
    --libdir=$prefix/$libdir \
    $lib_type
#
echo_eval touch $cppad_dir/$configured_flag
echo_eval make install
# -----------------------------------------------------------------------------
echo "get_$package: OK"
