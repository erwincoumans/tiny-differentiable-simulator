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

#define B3_NO_PROFILE 1
#include "math/tiny/tiny_double_utils.h"


typedef double InnerScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;

#include "math/tiny/tiny_algebra.hpp"


typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

#include "pytinydiffsim_includes.h"

using namespace TINY;
using namespace tds;

namespace py = pybind11;



PYBIND11_MODULE(pytinydiffsim, m) {


#include "pytinydiffsim.inl"

}