#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ $0 != "bin/travis.sh" ]
then
    echo 'bin/travis.sh: must be executed from its parent directory'
    exit 1
fi
if [ "$1" == '' ]
then
    echo 'usage: bin/travis.sh test_name'
    exit 1
fi
test_name="$1"
if [ ! -e $test_name ]
then
    if [ "$test_name" != 'all' ]
    then
cat << EOF
usage: bin/travis.sh test_name

Where test_name can be 'all', a directory that contains tests,
or a file that contains one test. A file that contains one test must
have the .cpp extension. Note that travis does not yet support test
that require other packages; e.g., adolc.
EOF
    fi
fi
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
if [ -e 'build' ]
then
    echo_eval rm -r build
fi
echo_eval mkdir build
echo_eval cd build
echo_eval cmake -D cppad_prefix=`pwd`/prefix ..
#
if [ "$test_name" == 'all' ]
then
    echo_eval make check
elif [ -d "$test_name" ]
then
    check=`echo $test_name | sed -e 's|/$||' -e 's|/|_|g' -e 's|^|check_|'`
    echo_eval make "$check"
else
    ext=`echo $test_name | sed -e 's|.*/||'`
    if [ "$ext" != '.cpp' ]
    then
        echo "travis.sh: $test_name is not 'all', a directory, or *.cpp file"
        exit 1
    fi
    cd ..
    echo_eval bin/test_one.sh $test_name
fi
echo_eval make install
echo_eval make uninstall
# -----------------------------------------------------------------------------
echo 'bin/travis.sh: OK'
exit 0
