// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "math/tiny/tiny_dual.h"
#include "math/tiny/tiny_dual_double_utils.h"
#include "math/tiny/tiny_algebra.hpp"
#include "dynamics/mass_matrix.hpp"
#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#define USE_IK_JAC_TRANSPOSE

typedef double InnerScalar;
typedef ::TINY::TinyDualDouble MyScalar;
typedef ::TINY::TinyDualDoubleUtils MyTinyConstants;

typedef TinyAlgebra<MyScalar, MyTinyConstants> MyAlgebra;

#include "pytinydiffsim_includes.h"

using namespace TINY;
using namespace tds;

namespace py = pybind11;


PYBIND11_MODULE(pytinydiffsim_dual, m) {
 
    py::class_<TinyDualDouble>(m, "TinyDualDouble")
        .def(py::init<InnerScalar, InnerScalar>())
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def("real", &TinyDualDouble::real)
        .def("dual", &TinyDualDouble::dual)
        .def(-py::self)
        .def("__repr__",
            [](const TinyDualDouble& a) {
                return "[ real=" + std::to_string(a.real()) + " , dual=" + std::to_string(a.dual()) + "]";
            })
        ;

#include "pytinydiffsim.inl"

}


