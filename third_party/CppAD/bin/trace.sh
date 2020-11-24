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
if [ $0 != "bin/trace.sh" ]
then
    echo "bin/trace.sh: must be executed from its parent directory"
    exit 1
fi
name="$1"
option="$2"
file="include/cppad/local/sweep/$name.hpp"
#
ok='yes'
if [ "$option" != '0' ] && [ "$option" != '1' ]
then
    ok='no'
fi
echo "grep '_TRACE [01]' $file"
if ! grep '_TRACE [01]' $file > /dev/null
then
    ok='no'
fi
if [ "$ok" == 'no' ]
then
    echo 'usage: bin/trace.sh name (0|1)'
    echo 'name: include/cppad/local/sweep/name.hpp defined *_TRACE as 0 or 1'
    exit 1
fi
old=`grep '_TRACE [01]' $file`
sed -e "s|TRACE [01]|TRACE $option|" -i $file
new=`grep '_TRACE [01]' $file`
#
echo "old: $old"
echo "new: $new"
#
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
