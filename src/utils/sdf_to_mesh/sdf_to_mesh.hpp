#pragma once

#include <vector>

#include "visualizer/opengl/tiny_opengl3_app.h"
#include "marching_cubes.hpp"

struct RenderShape
{
    std::vector<GfxVertexFormat1> vertices;
    std::vector<int> indices;

    size_t num_triangles;
    size_t num_vertices;
};
