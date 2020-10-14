
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>


#include "math/tiny/tiny_vector3.h"
#include "geometry.hpp"
#include "rigid_body.hpp"
//#include "examples/motion_import.h"
#include "urdf/urdf_parser.hpp"

#include "math/tiny/tiny_matrix3x3.h"
#include "math/tiny/tiny_matrix_x.h"
#include "mb_constraint_solver_spring.hpp"
#include "multi_body.hpp"
#include "math/pose.hpp"
#include "math/tiny/tiny_quaternion.h"
#include "math/tiny/tiny_raycast.h"
#include "urdf/urdf_structures.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "world.hpp"



template <typename Algebra>
struct UrdfToMultiBody2 {
    typedef ::tds::UrdfStructures<Algebra> TinyUrdfStructures;

    void convert(TinyUrdfStructures* urdf_structures,
        ::tds::World<Algebra>* world,
        ::tds::MultiBody<Algebra>* mb) {
        ::tds::UrdfToMultiBody<Algebra>::convert_to_multi_body(
            *urdf_structures, *world, *mb);

        mb->initialize();
    }
};

