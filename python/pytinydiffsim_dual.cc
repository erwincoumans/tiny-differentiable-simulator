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


#include "tiny_dual.h"
#include "tiny_dual_double_utils.h"


typedef double TinyDualScalar;
typedef TinyDualDouble MyScalar;
typedef TinyDualDoubleUtils MyTinyConstants;

#include "pytinydiffsim_includes.h"

namespace py = pybind11;

PYBIND11_MODULE(pytinydiffsim_dual, m) {
 
    py::class_<TinyDualDouble>(m, "TinyDualDouble")
        .def(py::init<TinyDualScalar, TinyDualScalar>())
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

