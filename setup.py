1#
# Copyright 2020 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages
from sys import platform as _platform
import sys
import glob
import os

from distutils.core import setup
from distutils.extension import Extension
from distutils.util import get_platform
from glob import glob

# monkey-patch for parallel compilation
import multiprocessing
import multiprocessing.pool


def parallelCCompile(self,
                     sources,
                     output_dir=None,
                     macros=None,
                     include_dirs=None,
                     debug=0,
                     extra_preargs=None,
                     extra_postargs=None,
                     depends=None):
    # those lines are copied from distutils.ccompiler.CCompiler directly
    macros, objects, extra_postargs, pp_opts, build = self._setup_compile(
        output_dir, macros, include_dirs, sources, depends, extra_postargs)
    cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)
    # parallel code
    N = 2 * multiprocessing.cpu_count()  # number of parallel compilations
    try:
        # On Unix-like platforms attempt to obtain the total memory in the
        # machine and limit the number of parallel jobs to the number of Gbs
        # of RAM (to avoid killing smaller platforms like the Pi)
        mem = os.sysconf('SC_PHYS_PAGES') * os.sysconf('SC_PAGE_SIZE')  # bytes
    except (AttributeError, ValueError):
        # Couldn't query RAM; don't limit parallelism (it's probably a well
        # equipped Windows / Mac OS X box)
        pass
    else:
        mem = max(1, int(round(mem / 1024 ** 3)))  # convert to Gb
        N = min(mem, N)

    def _single_compile(obj):
        try:
            src, ext = build[obj]
        except KeyError:
            return
        newcc_args = cc_args
        if _platform == "darwin":
            if src.endswith('.cpp') or src.endswith('.cc'):
                newcc_args = cc_args + ["-mmacosx-version-min=10.15", "-std=c++17", "-stdlib=libc++"]
        self._compile(obj, src, ext, newcc_args, extra_postargs, pp_opts)

    # convert to list, imap is evaluated on-demand
    pool = multiprocessing.pool.ThreadPool(N)
    list(pool.imap(_single_compile, objects))
    return objects


import distutils.ccompiler

distutils.ccompiler.CCompiler.compile = parallelCCompile

# see http://stackoverflow.com/a/8719066/295157
import os

platform = get_platform()
print(platform)

CXX_FLAGS = ''
CXX_FLAGS += '-fpermissive '
CXX_FLAGS += '-D_USE_MATH_DEFINES '
CXX_FLAGS += '-DUSE_EIGEN '



# libraries += [current_python]

libraries = []
include_dirs = ['src', '.','python', 'third_party/eigen3', 'third_party/tinyxml2/include', 'third_party/pybind11/include']

pytinyopengl3_libraries = []
pytinyopengl3_include_dirs = ['src', 'third_party/tinyobjloader', 'third_party/tinyxml2/include']

try:
    import numpy

    NP_DIRS = [numpy.get_include()]
except:
    print("numpy is disabled. getCameraImage maybe slower.")
else:
    print("numpy is enabled.")
    CXX_FLAGS += '-DPYBULLET_USE_NUMPY '
    for d in NP_DIRS:
        print("numpy_include_dirs = %s" % d)
    include_dirs += NP_DIRS

sources = ["third_party/tinyxml2/tinyxml2.cpp"]

pytinyopengl3_sources = ["python/pytinyopengl3.cc",\
"src/visualizer/opengl/tiny_camera.cpp",\
"src/visualizer/opengl/tiny_font_stash.cpp",\
"src/visualizer/opengl/tiny_fonts.cpp",\
"src/visualizer/opengl/tiny_gl_instancing_renderer.cpp",\
"src/visualizer/opengl/tiny_gl_primitive_renderer.cpp",\
"src/visualizer/opengl/tiny_gl_render_to_texture.cpp",\
"src/visualizer/opengl/tiny_glfw_opengl_window.cpp",\
"src/visualizer/opengl/tiny_load_shader.cpp",\
"src/visualizer/opengl/tiny_open_sans.cpp",\
"src/visualizer/opengl/tiny_opengl_fontstashcallbacks.cpp",\
"src/visualizer/opengl/tiny_opengl3_app.cpp",\
"src/visualizer/opengl/utils/tiny_logging.cpp",\
"third_party/stb_image/stb_image_write.cpp",\
"third_party/glad/gl.c",\
"third_party/stb_image/stb_image.cpp",\
"third_party/tinyobjloader/tiny_obj_loader.cc",\
"third_party/tinyxml2/tinyxml2.cpp",\
]



if _platform == "linux" or _platform == "linux2":
    print("linux")
    libraries = ['dl', 'pthread', 'stdc++fs']
    CXX_FLAGS += '-D_LINUX '
    CXX_FLAGS += '-DTINY_USE_EGL '
    CXX_FLAGS += '-DGLEW_STATIC '
    CXX_FLAGS += '-DGLEW_INIT_OPENGL11_FUNCTIONS=1 '
    CXX_FLAGS += '-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 '
    CXX_FLAGS += '-DDYNAMIC_LOAD_X11_FUNCTIONS '

    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once '
    CXX_FLAGS += '-fvisibility=hidden '
    CXX_FLAGS += '-fvisibility-inlines-hidden '
    CXX_FLAGS += '-std=c++1z '
    CXX_FLAGS += '-Wno-sign-compare '
    CXX_FLAGS += '-Wno-reorder '
    CXX_FLAGS += '-Wno-unused-local-typedefs '
    CXX_FLAGS += '-Wno-unused-variable '
    CXX_FLAGS += '-Wno-unused-but-set-variable '
    pytinyopengl3_libraries += ['dl','pthread']
    pytinyopengl3_sources += ["src/visualizer/opengl/tiny_x11_opengl_window.cpp",\
      "src/visualizer/opengl/tiny_egl_opengl_window.cpp",\
      "third_party/glad/egl.c",\
      "third_party/glad/glx.c"]


elif _platform == "win32":
    print("win32!")
    libraries = ['User32', 'kernel32']
    CXX_FLAGS += '-DWIN32 '
    CXX_FLAGS += '-DGLEW_STATIC '
    CXX_FLAGS += '/std:c++17 '
    
    pytinyopengl3_libraries = ['Ws2_32', 'Winmm', 'User32', 'Opengl32', 'kernel32', 'glu32', 'Gdi32', 'Comdlg32']
 
    pytinyopengl3_sources += ["src/visualizer/opengl/tiny_win32_opengl_window.cpp",\
    "src/visualizer/opengl/tiny_win32_window.cpp"]
    
elif _platform == "darwin":
    print("darwin!")
    os.environ['LDFLAGS'] = '-framework Cocoa -mmacosx-version-min=10.15 -stdlib=libc++ -framework OpenGL'
    CXX_FLAGS += '-DB3_NO_PYTHON_FRAMEWORK '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-D_DARWIN '
    CXX_FLAGS += '-mmacosx-version-min=10.15 '
    #    CXX_FLAGS += '-framework Cocoa '
    pytinyopengl3_sources += ["src/visualizer/opengl/tiny_mac_opengl_window.cpp",\
       "src/visualizer/opengl/tiny_mac_opengl_window_objc.m"]
    
else:
    print("bsd!")
    libraries = ['GL', 'GLEW', 'pthread']
    os.environ['LDFLAGS'] = '-L/usr/X11R6/lib'
    CXX_FLAGS += '-D_BSD '
    CXX_FLAGS += '-I/usr/X11R6/include '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-fno-inline-functions-called-once'

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

need_files = []
datadir = "diffphys_data"

hh = setup_py_dir + "/" + datadir

for root, dirs, files in os.walk(hh):
    for fn in files:
        ext = os.path.splitext(fn)[1][1:]
        if ext and ext in 'yaml index meta data-00000-of-00001 png gif jpg urdf sdf obj txt mtl dae off stl STL xml '.split(
        ):
            fn = root + "/" + fn
            need_files.append(fn[1 + len(hh):])

print("found resource files: %i" % len(need_files))
for n in need_files:
    print("-- %s" % n)
print("packages")
print(find_packages('examples/pybullet'))
print("-----")

extensions = []

CXX_FLAGS_TDS = CXX_FLAGS + '-DENABLE_TEST_ENVS ' + '-DNOMINMAX '

pytinydiffsim_ext = Extension(
    "pytinydiffsim",
    sources=sources+["python/pytinydiffsim.cc"],
    libraries=libraries,
    
    extra_compile_args=CXX_FLAGS_TDS.split(),
    include_dirs=include_dirs + ["."])

extensions.append(pytinydiffsim_ext)

pytinydiffsim_dual_ext = Extension(
    "pytinydiffsim_dual",
    sources=sources+["python/pytinydiffsim_dual.cc"],
    libraries=libraries,
    extra_compile_args=CXX_FLAGS.split(),
    include_dirs=include_dirs + ["."])

#extensions.append(pytinydiffsim_dual_ext)

if os.path.exists("third_party/CppAD/include"):
    platform_include_dirs = []
    if _platform == "win32":
      platform_include_dirs=["third_party/patches/CppADCodeGenWindows/include"]
    if _platform == "linux" or _platform == "linux2":
      platform_include_dirs=["third_party/patches/CppADCodeGenLinux/include"]
    if _platform == "darwin":
      platform_include_dirs=["third_party/patches/CppADCodeGenOSXIntel/include"]
    pytinydiffsim_ad_ext = Extension(
        "pytinydiffsim_ad",
        sources=sources+["python/pytinydiffsim_ad.cc"],
        libraries=libraries,
        extra_compile_args=CXX_FLAGS.split(),
        include_dirs=include_dirs + platform_include_dirs + [".",
                  "third_party/CppADCodeGen/include",
                  "third_party/CppAD/include" ])
      
    extensions.append(pytinydiffsim_ad_ext)
else:
    print("Skipping pytinydiffsim_ad extension since CppAD is missing.")

pytinyopengl3_ext = Extension(
    "pytinyopengl3",
    sources=pytinyopengl3_sources,
    libraries=pytinyopengl3_libraries,
    extra_compile_args=CXX_FLAGS.split(),
    include_dirs=pytinyopengl3_include_dirs + [
        ".", "third_party/pybind11/include", 
        "third_party/optionalX11",
        "third_party/glad",
        "third_party",
    ])
extensions.append(pytinyopengl3_ext)


setup(
    name='pytinydiffsim',
    version='0.6.0',
    description=
    'Tiny Differentiable Physics Library for Robotics Simulation and Reinforcement Learning',
    long_description=
    'Tiny Differentiable Physics Library for Robotics Simulation and Reinforcement Learning',
    url='https://github.com/google-research/tiny-differentiable-simulator',
    author='Eric Heiden, David Millard, Erwin Coumans',
    author_email='erwincoumans@google.com',
    license='Apache License 2.0',
    platforms='any',
    keywords=[
        'physics simulation', 'optimal control', 
        'robotics', 'collision detection', 'opengl',
        'reinforcement learning'
    ],
    install_requires=[
        'numpy',
    ],
    ext_modules=extensions,
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'License :: OSI Approved :: zlib/libpng License',
        'Operating System :: Microsoft :: Windows', 'Operating System :: POSIX :: Linux',
        'Operating System :: MacOS', 'Intended Audience :: Science/Research',
        "Programming Language :: Python", 'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.4', 'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6', 'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8', 'Topic :: Games/Entertainment :: Simulation',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Framework :: Robot Framework'
    ],
    package_dir={'': 'python'},
    packages=[x for x in find_packages('python')],
    package_data={'pytinydiffsim_data': need_files})
