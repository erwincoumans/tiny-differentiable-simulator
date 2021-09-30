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

#define USE_CPPAD
#include <vector>
#include <cppad/cg/cppadcg.hpp>
#include "math/tiny/cppad_utils.h"
#include "dynamics/mass_matrix.hpp"
#include "dynamics/kinematics.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#define USE_IK_JAC_TRANSPOSE

// Switch between Eigen and Tiny Algebra.
#if 1
#include <cppad/example/cppad_eigen.hpp>
#include "math/eigen_algebra.hpp"

typedef double InnerScalar; // define underlying data type
typedef CppAD::AD<InnerScalar> MyScalar; // wrap in Cpp::AD
typedef CppADUtils<InnerScalar> MyTinyConstants; // Utis struct with functions
typedef CppAD::ADFun<InnerScalar, InnerScalar> ADFun; // CppAD ADFun
typedef std::vector<InnerScalar> BaseVector; // Vector of CppAD base types
typedef tds::EigenAlgebraT<MyScalar> MyAlgebra; 

#else
#include "math/tiny/tiny_algebra.hpp"

typedef double InnerScalar; // define underlying data type
typedef CppAD::AD<InnerScalar> MyScalar; // wrap in Cpp::AD
typedef CppADUtils<InnerScalar> MyTinyConstants; // Utis struct with functions
typedef CppAD::ADFun<InnerScalar, InnerScalar> ADFun; // CppAD ADFun
typedef std::vector<InnerScalar> BaseVector; // Vector of CppAD base types
typedef TinyAlgebra<MyScalar, MyTinyConstants> MyAlgebra;
#endif


#include "pytinydiffsim_includes.h"
#include <pybind11/iostream.h>

using namespace TINY;
using namespace tds;
using std::vector;

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
        .def("value", [](const MyScalar& a) {
                return CppAD::Value(a);
        })
        .def("__repr__",
            [](const MyScalar& a) {
                return std::to_string(CppAD::Value(a));
        })
        ;
    
    /* Convenience CppAD functions */
    m.def("independent", &TinyAD::independent<InnerScalar>);
    m.def("compute_jacobian", &TinyAD::compute_jacobian<InnerScalar>);
    m.def("print_ad", &TinyAD::print_ad<InnerScalar>);
    m.def("hold_memory", &CppAD::thread_alloc::hold_memory);
    
    /* ADFun class */
    py::class_<ADFun>(m, "ADFun")
        .def(py::init([](vector<MyScalar>& x, vector<MyScalar>& y) {
            return new CppAD::ADFun<InnerScalar>(x, y);
        }))
        .def("Forward", [](ADFun& adfun, size_t q, const BaseVector& xq) {
            return adfun.Forward(q, xq);            
        }, py::call_guard<py::scoped_ostream_redirect,
                          py::scoped_estream_redirect>())
        .def("Reverse", &ADFun::Reverse<BaseVector>,
            py::call_guard<py::scoped_ostream_redirect,
                           py::scoped_estream_redirect>())
        .def("Jacobian", &ADFun::Jacobian<BaseVector>)
        ;

    /* Algebra functions */
    m.def("quat_from_euler_rpy", &MyAlgebra::quat_from_euler_rpy);
     
#include "pytinydiffsim.inl"

}
