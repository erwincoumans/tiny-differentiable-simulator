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


#include <cppad/cppad.hpp>

#include "math/tiny/cppad_utils.h"
#include "math/tiny/tiny_algebra.hpp"
#include "math/eigen_algebra.hpp"
#include "dynamics/mass_matrix.hpp"
#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#define USE_IK_JAC_TRANSPOSE

typedef double InnerScalar; // define underlying data type
typedef CppAD::AD<InnerScalar> MyScalar; // wrap in Cpp::AD
typedef CppADUtils<InnerScalar> MyTinyConstants; // Utis struct with functions
typedef TinyAlgebra<MyScalar, MyTinyConstants> MyAlgebra; // Algebra

#include "pytinydiffsim_includes.h"

using namespace TINY;
using namespace tds;

namespace py = pybind11;


PYBIND11_MODULE(pytinydiffsim_ad, m) {
 
    py::class_<MyScalar>(m, "ADDouble")
        .def(py::init<InnerScalar>())
        .def(py::self + py::self)
        .def(py::self + InnerScalar())
        .def(InnerScalar() + py::self)
        .def(py::self - py::self)
        .def(py::self - InnerScalar())
        .def(InnerScalar() - py::self)
        .def(py::self * py::self)
        .def(py::self * InnerScalar())
        .def(InnerScalar() * py::self)
        .def(py::self / py::self)
        .def(py::self / InnerScalar())
        .def(InnerScalar() / py::self)
        .def(py::self += py::self)
        .def(py::self += InnerScalar())
        .def(py::self -= py::self)
        .def(py::self -= InnerScalar())
        .def(py::self *= py::self)
        .def(py::self *= InnerScalar())
        .def(py::self /= py::self)
        .def(py::self /= InnerScalar())
        .def(-py::self)
        .def("__repr__",
            [](const MyScalar& a) {
                return std::to_string(CppAD::Value(a));
            })
        ;
    
    m.def("independent", &TinyAD::independent<InnerScalar>);
    m.def("compute_jacobian", &TinyAD::jacobian<InnerScalar>);

#include "pytinydiffsim.inl"

}


