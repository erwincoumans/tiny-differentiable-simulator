#!/bin/bash
# Builds shareable pytinydiffsim wheels in a manylinux_2_28 container, one per Python version, and runs
# auditwheel repair on them. The result needs glibc >= 2.27; OpenGL, EGL and X11 are loaded at run time.
#
#   ./build_manylinux_wheels.sh [OUT_DIR]     # OUT_DIR defaults to ./wheelhouse
#
# Environment:
#   PYTHONS     CPython versions to build                    default: "cp311 cp312 cp313 cp314"
#   JOBS        how many of them build at once (each needs   default: all of them
#               up to about 4 GB of memory)
#   WORK_DIR    staging and build directory on a local disk  default: a new directory in $TMPDIR or /tmp
#   IMAGE       manylinux image                              default: quay.io/pypa/manylinux_2_28_<machine>
#   CCACHE_DIR  compiler cache, kept between runs            default: tds-manylinux-ccache in $TMPDIR or /tmp
#
# The wheels hold every extension setup.py builds, pytinydiffsim_ad and pytinykernels too, so the CppAD and
# CppADCodeGen submodules must be checked out (git submodule update --init third_party/CppAD
# third_party/CppADCodeGen). The other submodules are not needed.
#
# The Docker daemon runs as root, which an NFS home with root squash does not let read the checkout, so the
# tracked sources (with uncommitted edits, without untracked files) are copied to WORK_DIR first. The container
# sets up ccache as root, then builds as the calling user, so everything it writes stays removable without sudo.
# setuptools builds in the source tree, so each Python version builds in a copy of its own (hard links), and
# the versions build in parallel: each spends most of its time in a few large translation units (about 3
# minutes for all four on 16 cores). The cache mostly serves reruns: the versions share only the objects of
# the files that do not include Python.h (glad, stb, tinyxml2). Follow a build in $WORK_DIR/logs/<cpXY>.log.
set -euo pipefail

if [[ "${1:-}" == --in-container ]]; then
  # Inside the container, as root. The image has no ccache, so ccache 4 is built from source on the first run
  # (half a minute) and kept in the cache directory. It is put in front of gcc and g++ on PATH (masquerading),
  # since setuptools calls them directly. Then the build, as the calling user.
  ccache_version=4.11.3
  ccache_bin=/ccache/bin/ccache-$ccache_version-$(uname -m)
  if [[ ! -x $ccache_bin ]]; then
    echo "building ccache $ccache_version"
    { dnf -y -q install libzstd-devel &&
      curl -fsSL "https://github.com/ccache/ccache/releases/download/v$ccache_version/ccache-$ccache_version.tar.gz" |
        tar -xz -C /tmp &&
      cmake -S "/tmp/ccache-$ccache_version" -B /tmp/ccache-build -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTING=OFF \
        -DENABLE_DOCUMENTATION=OFF -DREDIS_STORAGE_BACKEND=OFF -DHTTP_STORAGE_BACKEND=OFF &&
      cmake --build /tmp/ccache-build -j"$(nproc)" &&
      install -D -m 755 /tmp/ccache-build/ccache "$ccache_bin" &&
      chown -R "$HOST_UID:$HOST_GID" /ccache/bin; } > /tmp/ccache-build.log 2>&1 ||
      { tail -20 /tmp/ccache-build.log; echo "warning: could not build ccache, building without it" >&2; }
  fi
  if [[ -x $ccache_bin ]]; then
    install -m 755 "$ccache_bin" /usr/local/bin/ccache
    mkdir -p /usr/local/ccache-bin
    for compiler in cc c++ gcc g++; do ln -sf /usr/local/bin/ccache "/usr/local/ccache-bin/$compiler"; done
  fi
  exec setpriv --reuid="$HOST_UID" --regid="$HOST_GID" --clear-groups "$0" --build
fi

if [[ "${1:-}" == --build-one ]]; then
  # One Python version, in /work/build/<cpXY>, a copy of the staged sources: setuptools writes build/ and the
  # egg-info next to setup.py.
  py=$2
  rm -rf "/work/build/$py"
  mkdir -p /work/build
  cp -al /work/src "/work/build/$py"
  cd "/work/build/$py"
  # The image's Pythons were built with -g, which setuptools passes on: the extensions would hold 300 MB of
  # debug information. setuptools >= 72 replaces the interpreter's CFLAGS with $CFLAGS for C and with $CXXFLAGS
  # for C++, older ones append $CFLAGS to them, so both are set to the interpreter's flags, -g0 last.
  flags="$("/opt/python/$py-$py/bin/python" -c 'import sysconfig; print(sysconfig.get_config_var("CFLAGS"))') -g0"
  export CFLAGS=$flags CXXFLAGS=$flags
  if "/opt/python/$py-$py/bin/pip" wheel -v --no-deps -w /work/dist . > "/work/logs/$py.log" 2>&1; then
    echo "=== $py built $(date +%T)"
  else
    echo "=== $py FAILED $(date +%T), the end of logs/$py.log:"
    tail -40 "/work/logs/$py.log"
    exit 1
  fi
  exit 0
fi

if [[ "${1:-}" == --build ]]; then
  # /work holds src/ (the staged sources), and gets build/, logs/, dist/ and wheelhouse/; /ccache is CCACHE_DIR.
  export HOME=/work/home PIP_CACHE_DIR=/work/pip-cache
  if command -v ccache >/dev/null; then
    # BASEDIR keeps each version's build tree (/work/build/<cpXY>) out of the hashes.
    export PATH=/usr/local/ccache-bin:$PATH CCACHE_DIR=/ccache CCACHE_BASEDIR=/work CCACHE_MAXSIZE=5G
    ccache -z >/dev/null
  fi
  rm -rf /work/dist /work/wheelhouse /work/logs
  mkdir -p "$HOME" /work/dist /work/wheelhouse /work/logs
  echo "=== $PYTHONS, $JOBS at a time, from $(date +%T)"
  printf '%s\n' $PYTHONS | xargs -P "$JOBS" -I{} "$0" --build-one {}
  for whl in /work/dist/*.whl; do
    auditwheel repair "$whl" --plat "$PLAT" -w /work/wheelhouse
  done
  if command -v ccache >/dev/null; then ccache -s; fi
  exit 0
fi

SCRIPT=$(realpath "${BASH_SOURCE[0]}")
ROOT=$(git -C "$(dirname "$SCRIPT")" rev-parse --show-toplevel)
OUT=$(realpath -m "${1:-wheelhouse}")
MACHINE=$(uname -m)
PLAT=manylinux_2_28_$MACHINE
IMAGE=${IMAGE:-quay.io/pypa/$PLAT}
PYTHONS=${PYTHONS:-cp311 cp312 cp313 cp314}
JOBS=${JOBS:-$(wc -w <<< "$PYTHONS")}
WORK=${WORK_DIR:-$(mktemp -d "${TMPDIR:-/tmp}/tds-manylinux.XXXXXX")}
CCACHE=${CCACHE_DIR:-${TMPDIR:-/tmp}/tds-manylinux-ccache}
SUBMODULES=(third_party/CppAD third_party/CppADCodeGen)
for submodule in "${SUBMODULES[@]}"; do
  if [[ ! -d "$ROOT/$submodule/include" ]]; then
    echo "$submodule is not checked out: git submodule update --init ${SUBMODULES[*]}" >&2
    exit 1
  fi
done
mkdir -p "$WORK/src" "$CCACHE"

echo "staging sources from $ROOT in $WORK/src"
# The superproject's tracked files, but not its submodules (git ls-files names each as a directory, which tar
# would copy whole), then the files of the two submodules the build needs.
{
  git -C "$ROOT" ls-files -z |
    grep -zvxF -f <(git -C "$ROOT" config -f .gitmodules --get-regexp '\.path$' | cut -d' ' -f2)
  for submodule in "${SUBMODULES[@]}"; do
    git -C "$ROOT/$submodule" ls-files -z | sed -z "s|^|$submodule/|"
  done
} | (cd "$ROOT" && tar --null -T - -cf -) | tar -xf - -C "$WORK/src"
cp "$SCRIPT" "$WORK/build_manylinux_wheels.sh"

echo "building $PYTHONS for $PLAT"
docker run --rm -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -e PYTHONS="$PYTHONS" -e JOBS="$JOBS" -e PLAT="$PLAT" \
  -v "$WORK:/work" -v "$CCACHE:/ccache" \
  "$IMAGE" /work/build_manylinux_wheels.sh --in-container

mkdir -p "$OUT"
cp -v "$WORK"/wheelhouse/*.whl "$OUT/"
echo "wheels in $OUT; staging and build directories left in $WORK (rm -rf it when done)"
