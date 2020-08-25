#
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
            if src.endswith('.cpp'):
                newcc_args = cc_args + ["-mmacosx-version-min=10.7", "-stdlib=libc++"]
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


# libraries += [current_python]

libraries = []
include_dirs = ['.','python', 'third_party/tinyxml2/include', 'third_party/pybind11/include']

pytinyopengl3_libraries = []
pytinyopengl3_include_dirs = []

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

sources = ["python/pytinydiffsim.cc", "third_party/tinyxml2/tinyxml2.cpp"]

pytinyopengl3_sources = ["python/pytinyopengl3.cc",\
"examples/opengl_window/tiny_camera.cpp",\
"examples/opengl_window/tiny_font_stash.cpp",\
"examples/opengl_window/tiny_fonts.cpp",\
"examples/opengl_window/tiny_gl_instancing_renderer.cpp",\
"examples/opengl_window/tiny_gl_primitive_renderer.cpp",\
"examples/opengl_window/tiny_gl_render_to_texture.cpp",\
"examples/opengl_window/tiny_glfw_opengl_window.cpp",\
"examples/opengl_window/tiny_load_shader.cpp",\
"examples/opengl_window/tiny_open_sans.cpp",\
"examples/opengl_window/tiny_opengl_fontstashcallbacks.cpp",\
"examples/opengl_window/tiny_opengl3_app.cpp",\
"third_party/stb_image/stb_image_write.cpp",\
"third_party/glad/gl.c",\
]



if _platform == "linux" or _platform == "linux2":
    print("linux")
    libraries = ['dl', 'pthread']
    CXX_FLAGS += '-D_LINUX '
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


elif _platform == "win32":
    print("win32!")
    libraries = ['User32', 'kernel32']
    CXX_FLAGS += '-DWIN32 '
    CXX_FLAGS += '-DGLEW_STATIC '
    CXX_FLAGS += '/std:c++17 '
    
    pytinyopengl3_libraries = ['Ws2_32', 'Winmm', 'User32', 'Opengl32', 'kernel32', 'glu32', 'Gdi32', 'Comdlg32']
 
    pytinyopengl3_sources += ["examples/opengl_window/tiny_win32_opengl_window.cpp",\
    "examples/opengl_window/tiny_win32_window.cpp"]
    
elif _platform == "darwin":
    print("darwin!")
    os.environ['LDFLAGS'] = '-framework Cocoa -mmacosx-version-min=10.7 -stdlib=libc++ -framework OpenGL'
    CXX_FLAGS += '-DB3_NO_PYTHON_FRAMEWORK '
    CXX_FLAGS += '-DHAS_SOCKLEN_T '
    CXX_FLAGS += '-D_DARWIN '
    CXX_FLAGS += '-std=c++17 '
    CXX_FLAGS += '-stdlib=libc++ '
    CXX_FLAGS += '-mmacosx-version-min=10.7 '
    #    CXX_FLAGS += '-framework Cocoa '
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

pytinydiffsim_ext = Extension(
    "pytinydiffsim",
    sources=sources,
    libraries=libraries,
    extra_compile_args=CXX_FLAGS.split(),
    include_dirs=include_dirs + ["."])

extensions.append(pytinydiffsim_ext)

pytinyopengl3_ext = Extension(
    "pytinyopengl3",
    sources=pytinyopengl3_sources,
    libraries=pytinyopengl3_libraries,
    extra_compile_args=CXX_FLAGS.split(),
    include_dirs=pytinyopengl3_include_dirs + [
        ".", "third_party/pybind11/include", 
        "third_party/glad",
        "third_party",
    ])
extensions.append(pytinyopengl3_ext)


setup(
    name='pytinydiffsim',
    version='0.0.1',
    description=
    'Tiny Differentiable Physics Library for Robotics Simulation and Reinforcement Learning',
    long_description=
    'tbd',
    url='https://github.com/bulletphysics/bullet3',
    author='Eric Heiden, Erwin Coumans',
    author_email='erwincoumans@google.com',
    license='zlib',
    platforms='any',
    keywords=[
        'game development', 'virtual reality', 'physics simulation', 'robotics',
        'collision detection', 'opengl'
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
