
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>


#include "tiny_vector3.h"
#include "tiny_geometry.h"
#include "tiny_rigid_body.h"
#include "examples/motion_import.h"
#include "examples/tiny_urdf_parser.h"

#include "tiny_matrix3x3.h"
#include "tiny_matrix_x.h"
#include "tiny_mb_constraint_solver_spring.h"
#include "tiny_multi_body.h"
#include "tiny_pose.h"
#include "tiny_quaternion.h"
#include "tiny_raycast.h"
#include "tiny_urdf_structures.h"
#include "tiny_urdf_to_multi_body.h"
#include "tiny_world.h"



template <typename TinyScalar, typename TinyConstants>
struct UrdfToMultiBody2 {
    typedef ::TinyUrdfStructures<TinyScalar, TinyConstants> TinyUrdfStructures;

    void convert(TinyUrdfStructures* urdf_structures,
        TinyWorld<TinyScalar, TinyConstants>* world,
        TinyMultiBody<MyScalar, MyTinyConstants>* mb) {
        TinyUrdfToMultiBody<MyScalar, MyTinyConstants>::convert_to_multi_body(
            *urdf_structures, *world, *mb);

        mb->initialize();
    }
};

