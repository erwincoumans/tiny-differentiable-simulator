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

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdio.h>

#include "math/tiny/tiny_float_utils.h"
#include "math/tiny/tiny_vector3.h"
#include "math/tiny/tiny_pose.h"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/tiny_camera.h"
#include <string>

#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "stb_image/stb_image.h"

using namespace TINY;

std::string file_open_dialog(TinyWindowInterface* window)
{
  std::string file_name="";
  char file_name_buffer[1024]={0};
  if (window)
  {
    if (window->file_open_dialog(file_name_buffer, 1024))
      {
        file_name = file_name_buffer;
      }
  }
  return file_name;
}


std::vector<int> my_load_obj_shapes(TinyOpenGL3App& opengl_app, const std::string& obj_filename, const ::TINY::TinyVector3f& pos, const ::TINY::TinyQuaternionf& orn, const ::TINY::TinyVector3f& scaling)
{
    std::vector<int> shape_ids;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    //test Wavefront obj loading
    std::string warn;
    std::string err;
    char basepath[1024];
    bool triangulate = true;

    ::tds::FileUtils::extract_path(obj_filename.c_str(), basepath, 1024);
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, obj_filename.c_str(),
        basepath, triangulate);

    for (int i = 0; i < shapes.size(); i++)
    {
        std::vector<int> indices;
        std::vector<GfxVertexFormat1> vertices;
        int textureIndex = -1;
        TinyMeshUtils::extract_shape(attrib, shapes[i], materials, indices, vertices, textureIndex);
        textureIndex = -1;
        ::TINY::TinyVector3f color(1, 1, 1);
        if (shapes[i].mesh.material_ids.size())
        {
            int mat_id = shapes[i].mesh.material_ids[0];
            if (mat_id >=0 && mat_id < materials.size())
            {
                const tinyobj::material_t& mat = materials[mat_id];
                color.setValue(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
                if (mat.diffuse_texname.length())
                {
                    std::string texture_file_name = std::string(basepath) + mat.diffuse_texname;
                    std::vector< unsigned char> buffer;
                    int width, height, n;
                    unsigned char* image = stbi_load(texture_file_name.c_str(), &width, &height, &n, 3);

                    textureIndex = opengl_app.m_renderer->register_texture(image, width, height);
                    free(image);
                }
            }
        }
        int shape = opengl_app.m_renderer->register_shape(&vertices[0].x, vertices.size(), &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex);
        shape_ids.push_back(shape);
    }

    opengl_app.m_renderer->write_transforms();
    return shape_ids;
}


namespace py = pybind11;

PYBIND11_MODULE(pytinyopengl3, m) {
  m.doc() = R"pbdoc(
        tiny opengl3 graphics engine python binding
        -----------------------

        .. currentmodule:: pytinyopengl3

        .. autosummary::
           :toctree: _generate

    )pbdoc";


  py::class_<DrawGridData>(m, "DrawGridData")
     .def(py::init<>())
    .def_readwrite("upAxis", &DrawGridData::upAxis)
    .def_readwrite("drawAxis", &DrawGridData::drawAxis)
    .def_readwrite("upOffset", &DrawGridData::upOffset)
    .def_readwrite("gridSize", &DrawGridData::gridSize);
         

  py::class_<TinyOpenGL3App>(m,"TinyOpenGL3App")
    .def(py::init<const char*,int,int, bool, int, int, int, int>(),
      py::arg("title")="pytinyopengl3",
      py::arg("width")=1024,
      py::arg("height")=768,
      py::arg("allowRetina")=1,
      py::arg("windowType")=0,
      py::arg("renderDevice")=-1,
      py::arg("maxNumObjectCapacity")=256 * 1024,
      py::arg("maxShapeCapacityInBytes")= 256 * 1024 * 1024)
      .def("swap_buffer", &TinyOpenGL3App::swap_buffer)
      .def("register_cube_shape", &TinyOpenGL3App::register_cube_shape)
      .def("register_graphics_unit_sphere_shape", &TinyOpenGL3App::register_graphics_unit_sphere_shape)
      .def("register_graphics_capsule_shape", &TinyOpenGL3App::register_graphics_capsule_shape)
      .def("draw_grid", (void (TinyOpenGL3App::*)())&TinyOpenGL3App::draw_grid)
      .def("draw_grid", (void (TinyOpenGL3App::*)(DrawGridData)) & TinyOpenGL3App::draw_grid)
      .def("draw_text_3d", (void (TinyOpenGL3App::*)(const char*, float [3], float[4], float[4], float, int)) &TinyOpenGL3App::draw_text_3d)
      .def("draw_text_3d", (void (TinyOpenGL3App::*)(const char*, float, float, float, float)) &TinyOpenGL3App::draw_text_3d)
      .def_readwrite("renderer",&TinyOpenGL3App::m_renderer)
      .def_readwrite("window",&TinyOpenGL3App::m_window)
      ;
      
  
  py::class_<TinyCamera>(m, "TinyCamera")
    .def(py::init<>())
    .def("update", &TinyCamera::update)
    .def("set_camera_distance", &TinyCamera::set_camera_distance)
    .def("set_camera_pitch", &TinyCamera::set_camera_pitch)
    .def("set_camera_yaw", &TinyCamera::set_camera_yaw)
    .def("set_camera_up_vector",&TinyCamera::set_camera_up_vector)
    .def("set_camera_up_axis", &TinyCamera::set_camera_up_axis)
    .def("set_camera_target_position", &TinyCamera::set_camera_target_position)
      ;
      
  py::class_<TinyGLInstancingRenderer>(m, "TinyGLInstancingRenderer")
    .def("init", &TinyGLInstancingRenderer::init)
    
    .def("update_camera", &TinyGLInstancingRenderer::update_camera)
    
    .def("register_shape", &TinyGLInstancingRenderer::register_shape1)
    //.def("update_shape", &TinyGLInstancingRenderer::update_shape)

    .def("register_texture", &TinyGLInstancingRenderer::register_texture1)
    //.def("update_texture", &TinyGLInstancingRenderer::update_texture)
    .def("remove_texture", &TinyGLInstancingRenderer::remove_texture)

    .def("register_graphics_instance", &TinyGLInstancingRenderer::register_graphics_instance)
    .def("write_single_instance_transform_to_cpu", &TinyGLInstancingRenderer::write_single_instance_transform_to_cpu)
    .def("write_single_instance_color_to_cpu", &TinyGLInstancingRenderer::write_single_instance_color_to_cpu2)
      
    .def("render_scene", &TinyGLInstancingRenderer::render_scene)
    .def("write_transforms", &TinyGLInstancingRenderer::write_transforms)
    .def("remove_all_instances", &TinyGLInstancingRenderer::remove_all_instances)
    .def("remove_graphics_instance", &TinyGLInstancingRenderer::remove_graphics_instance)
      
    //.def("get_active_camera", (TinyCamera* (TinyGLInstancingRenderer::*)()) &TinyGLInstancingRenderer::get_active_camera)
    .def("set_camera", &TinyGLInstancingRenderer::set_active_camera)
      
    .def("draw_line", &TinyGLInstancingRenderer::draw_line)
    .def("draw_lines", &TinyGLInstancingRenderer::draw_lines)
    .def("get_screen_width", &TinyGLInstancingRenderer::get_screen_width)
    .def("get_screen_height", &TinyGLInstancingRenderer::get_screen_height)
    .def("get_total_num_instances", &TinyGLInstancingRenderer::get_total_num_instances)
    .def("set_plane_reflection_shape_index", &TinyGLInstancingRenderer::set_plane_reflection_shape_index)
    ;
    
   py::class_<TinyWindowInterface>(m, "TinyWindowInterface")
  .def("requested_exit", &TinyWindowInterface::requested_exit)
  .def("set_request_exit", &TinyWindowInterface::requested_exit)
  //.def("set_mouse_button_callback", &TinyWindowInterface::set_mouse_button_callback)
  .def("set_window_title", &TinyWindowInterface::set_window_title)
  ;
  
  m.def("file_open_dialog", &file_open_dialog);

  m.def("load_obj_shapes", &my_load_obj_shapes);
      
  
  py::class_<TinyPose<float, FloatUtils>>(m, "TinyPosef")
      .def(py::init<>())
      .def(py::self * py::self)
      .def_readwrite("position", &TinyPose<float, FloatUtils>::m_position)
      .def_readwrite("orientation", &TinyPose<float, FloatUtils>::m_orientation)
      .def("set_identity", &TinyPose<float, FloatUtils>::set_identity);

  py::class_<TinyVector3<float, FloatUtils>>(m, "TinyVector3f")
      .def(py::init<float, float, float>())
      .def("set_zero", &TinyVector3<float, FloatUtils>::set_zero)
      .def_readwrite("x", &TinyVector3<float, FloatUtils>::m_x)
      .def_readwrite("y", &TinyVector3<float, FloatUtils>::m_y)
      .def_readwrite("z", &TinyVector3<float, FloatUtils>::m_z)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(-py::self)
      .def("__repr__",
           [](const TinyVector3<float, FloatUtils> &a) {
             return "[" + std::to_string(a.m_x) + " " + std::to_string(a.m_y) +
                    " " + std::to_string(a.m_z) + "]";
           })
      .def("__getitem__", [](const TinyVector3<float, FloatUtils> &a,
                             int i) { return a[i]; })
      .def("__setitem__", [](TinyVector3<float, FloatUtils> &a, int i,
                             double v) { a[i] = v; });

    py::class_<TinyMatrix3x3<float, FloatUtils>>(m, "TinyMatrix3x3f")
        .def(py::init<float, float, float,
            float, float, float,
            float, float, float>())
        .def(py::init<TinyQuaternion<float, FloatUtils>>())
        .def("get_at", &TinyMatrix3x3<float, FloatUtils>::get_at)
        .def("get_row", &TinyMatrix3x3<float, FloatUtils>::getRow)
        .def("set_identity", &TinyMatrix3x3<float, FloatUtils>::set_identity)
        .def("setRotation", &TinyMatrix3x3<float, FloatUtils>::setRotation)
        .def("getRotation", &TinyMatrix3x3<float, FloatUtils>::getRotation2);

  py::class_<TinyQuaternion<float, FloatUtils>>(m, "TinyQuaternionf")
      .def(py::init<float, float, float, float>())
      .def("set_identity", &TinyQuaternion<float, FloatUtils>::set_identity)
      .def("get_euler_rpy", &TinyQuaternion<float, FloatUtils>::get_euler_rpy)
      .def("get_euler_rpy2",
           &TinyQuaternion<float, FloatUtils>::get_euler_rpy2)
      .def("set_euler_rpy", &TinyQuaternion<float, FloatUtils>::set_euler_rpy)

      .def_readwrite("x", &TinyQuaternion<float, FloatUtils>::m_x)
      .def_readwrite("y", &TinyQuaternion<float, FloatUtils>::m_y)
      .def_readwrite("z", &TinyQuaternion<float, FloatUtils>::m_z)
      .def_readwrite("w", &TinyQuaternion<float, FloatUtils>::m_w)
      .def("__repr__",
           [](const TinyQuaternion<float, FloatUtils> &q) {
             return "[" + std::to_string(q.m_x) + " " + std::to_string(q.m_y) +
                    " " + std::to_string(q.m_z) + " " + std::to_string(q.m_w) +
                    "]";
           })
      .def("__getitem__", [](const TinyQuaternion<float, FloatUtils> &a,
                             int i) { return a[i]; })
      .def("__setitem__", [](TinyQuaternion<float, FloatUtils> &a, int i,
                             float v) { a[i] = v; });
    py::enum_<EnumSphereLevelOfDetail>(m, "EnumSphereLevelOfDetail")
        .value("SPHERE_LOD_POINT_SPRITE", SPHERE_LOD_POINT_SPRITE, "SPHERE_LOD_POINT_SPRITE")
        .value("SPHERE_LOD_LOW", SPHERE_LOD_LOW, "SPHERE_LOD_LOW")
        .value("SPHERE_LOD_MEDIUM", SPHERE_LOD_MEDIUM, "SPHERE_LOD_MEDIUM")
        .value("SPHERE_LOD_HIGH", SPHERE_LOD_HIGH, "SPHERE_LOD_HIGH") 
        .export_values();
        ;

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
 
}
