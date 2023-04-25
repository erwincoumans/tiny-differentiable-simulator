//runner.py --play --file rl_games\configs\tds\ppo_tds_ant.yaml --checkpoint runs/Ant_tds/nn/Ant_tds.pth
// 
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

#include <string>
#include <stdio.h>

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>



#include "math/tiny/tiny_float_utils.h"
#include "math/tiny/tiny_vector3.h"
#include "math/tiny/tiny_pose.h"
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/tiny_camera.h"
#include "examples/opengl_urdf_visualizer.h"
#include "urdf/urdf_parser.hpp"
#include "tiny_obj_loader.h"
#include "utils/file_utils.hpp"
#include "visualizer/opengl/utils/tiny_mesh_utils.h"
#include "math/tiny/tiny_algebra.hpp"
#include "stb_image/stb_image.h"

//to expose OpenGL constants
#include "visualizer/opengl/tiny_opengl_include.h"

using namespace TINY;
typedef ::TINY::FloatUtils MyTinyConstants;
typedef TinyAlgebra<float, MyTinyConstants> MyAlgebra;


extern std::string DYNAMIC_CUDA_PATH;
extern std::string DYNAMIC_CUDART_PATH;

void set_cuda_path(const std::string& cuda_path)
{
	DYNAMIC_CUDA_PATH = cuda_path;
}
void set_cudart_path(const std::string& cudart_path)
{
	DYNAMIC_CUDART_PATH = cudart_path;
}
	

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

extern std::string triangleVertexShaderTextInit;
extern std::string triangleFragmentShaderInit;
extern std::string useShadowMapInstancingVertexShaderInit;
extern std::string useShadowMapInstancingFragmentShaderInit;
extern std::string createShadowMapInstancingVertexShaderInit;
extern std::string createShadowMapInstancingFragmentShaderInit;
extern std::string segmentationMaskInstancingVertexShaderInit;
extern std::string segmentationMaskInstancingFragmentShaderInit;
extern std::string instancingVertexShaderInit;
extern std::string instancingFragmentShaderInit;

std::string get_triangle_vertex_shader()
{
    return triangleVertexShaderTextInit;
}

void set_triangle_vertex_shader(const std::string& vertex_shader)
{
    triangleVertexShaderTextInit = vertex_shader;
}

std::string get_triangle_fragment_shader()
{
    return triangleFragmentShaderInit;
}

void set_triangle_fragment_shader(const std::string& fragment_shader)
{
    triangleFragmentShaderInit = fragment_shader;
}

std::string get_use_shadowmap_instancing_vertex_shader()
{
    return useShadowMapInstancingVertexShaderInit;
}

void set_use_shadowmap_instancing_vertex_shader(const std::string& vertex_shader)
{
    useShadowMapInstancingVertexShaderInit = vertex_shader;
}

std::string get_create_shadowmap_instancing_vertex_shader()
{
    return createShadowMapInstancingVertexShaderInit;
}

void set_create_shadowmap_instancing_vertex_shader(const std::string& vertex_shader)
{
    createShadowMapInstancingVertexShaderInit = vertex_shader;
}

std::string get_use_shadowmap_instancing_fragment_shader()
{
    return useShadowMapInstancingFragmentShaderInit;
}

void set_use_shadowmap_instancing_fragment_shader(const std::string& fragment_shader)
{
    useShadowMapInstancingFragmentShaderInit = fragment_shader;
}

std::string get_create_shadowmap_instancing_fragment_shader()
{
    return createShadowMapInstancingFragmentShaderInit;
}

void set_create_shadowmap_instancing_fragment_shader(const std::string& fragment_shader)
{
    createShadowMapInstancingFragmentShaderInit = fragment_shader;
}


std::string get_segmentation_mask_instancing_vertex_shader()
{
    return segmentationMaskInstancingVertexShaderInit;
}

void set_segmentation_mask_instancing_vertex_shader(const std::string& vertex_shader)
{
    segmentationMaskInstancingVertexShaderInit = vertex_shader;
}

std::string get_segmentation_mask_instancing_fragment_shader()
{
    return segmentationMaskInstancingFragmentShaderInit;
}

void set_segmentation_mask_instancing_fragment_shader(const std::string& fragment_shader)
{
    segmentationMaskInstancingFragmentShaderInit = fragment_shader;
}


std::string get_instancing_vertex_shader()
{
    return instancingVertexShaderInit;
}

void set_instancing_vertex_shader(const std::string& vertex_shader)
{
    instancingVertexShaderInit = vertex_shader;
}

std::string get_instancing_fragment_shader()
{
    return instancingFragmentShaderInit;
}

void set_instancing_fragment_shader(const std::string& fragment_shader)
{
    instancingFragmentShaderInit = fragment_shader;
}



std::string my_extract_path(const std::string& file_name) {
  char full_path[TINY_MAX_EXE_PATH_LEN];
  tds::FileUtils::extract_path(file_name.c_str(), full_path,
                               TINY_MAX_EXE_PATH_LEN);
  return full_path;
}

std::array<float, 16> compute_camera_view_matrix(const ::TINY::TinyVector3f& cam_pos,  const ::TINY::TinyVector3f& cam_target,  const ::TINY::TinyVector3f& cam_up)
{
    const auto& eye = cam_pos;
    const auto& center = cam_target;
    const auto& up = cam_up;
    auto f = (center - eye).normalized();
    auto u = up.normalized();
    auto s = (f.cross(u)).normalized();
    u = s.cross(f);

    std::array<float, 16> viewMatrix;

	viewMatrix[0 * 4 + 0] = s.x();
	viewMatrix[1 * 4 + 0] = s.y();
	viewMatrix[2 * 4 + 0] = s.z();

	viewMatrix[0 * 4 + 1] = u.x();
	viewMatrix[1 * 4 + 1] = u.y();
	viewMatrix[2 * 4 + 1] = u.z();

	viewMatrix[0 * 4 + 2] = -f.x();
	viewMatrix[1 * 4 + 2] = -f.y();
	viewMatrix[2 * 4 + 2] = -f.z();

	viewMatrix[0 * 4 + 3] = 0.f;
	viewMatrix[1 * 4 + 3] = 0.f;
	viewMatrix[2 * 4 + 3] = 0.f;

	viewMatrix[3 * 4 + 0] = -s.dot(eye);
	viewMatrix[3 * 4 + 1] = -u.dot(eye);
	viewMatrix[3 * 4 + 2] = f.dot(eye);
	viewMatrix[3 * 4 + 3] = 1.f;
    return viewMatrix;
}


std::array<float, 16> compute_camera_projection_matrix(float left, float right, float bottom, float top, float nearVal, float farVal)
{
    std::array<float, 16> projectionMatrix;

	projectionMatrix[0 * 4 + 0] = (float(2) * nearVal) / (right - left);
	projectionMatrix[0 * 4 + 1] = float(0);
	projectionMatrix[0 * 4 + 2] = float(0);
	projectionMatrix[0 * 4 + 3] = float(0);

	projectionMatrix[1 * 4 + 0] = float(0);
	projectionMatrix[1 * 4 + 1] = (float(2) * nearVal) / (top - bottom);
	projectionMatrix[1 * 4 + 2] = float(0);
	projectionMatrix[1 * 4 + 3] = float(0);

	projectionMatrix[2 * 4 + 0] = (right + left) / (right - left);
	projectionMatrix[2 * 4 + 1] = (top + bottom) / (top - bottom);
	projectionMatrix[2 * 4 + 2] = -(farVal + nearVal) / (farVal - nearVal);
	projectionMatrix[2 * 4 + 3] = float(-1);

	projectionMatrix[3 * 4 + 0] = float(0);
	projectionMatrix[3 * 4 + 1] = float(0);
	projectionMatrix[3 * 4 + 2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
	projectionMatrix[3 * 4 + 3] = float(0);
    return projectionMatrix;
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


struct ReadPixelBuffer
{

  pybind11::capsule buffer_handle;
  pybind11::array_t<unsigned char> rgba;
  pybind11::array_t<float> depth;

  std::vector<unsigned char> rgba_data;
  std::vector<float> depth_data;
  
  ReadPixelBuffer(TinyOpenGL3App& gl_app)
  :buffer_handle(pybind11::capsule ([](){}))
  {
    gl_app.get_screen_pixels(rgba_data,depth_data);

    unsigned char* rgba_ptr = rgba_data.size()? &rgba_data[0] : 0;
    rgba = pybind11::array_t<unsigned char>(rgba_data.size(), rgba_ptr,
                                            buffer_handle);

    float* depth_ptr = depth_data.size() ? &depth_data[0] : 0;
    depth =
        pybind11::array_t<float>(depth_data.size(), depth_ptr, buffer_handle);
  }

  virtual ~ReadPixelBuffer()
  {
  
  }


};

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

  py::class_< TinyCudaVbo>(m,"TinyCudaVbo")
      .def_readwrite("num_instances",&TinyCudaVbo::num_instances)
      .def_readwrite("positions",&TinyCudaVbo::positions_int)
      .def_readwrite("orientations",&TinyCudaVbo::orientations_int)
      .def_readwrite("colors",&TinyCudaVbo::colors_int)
      .def_readwrite("scalings",&TinyCudaVbo::scalings_int)
      .def_readwrite("vertices",&TinyCudaVbo::vertices_int)
      ;

  py::class_< GfxVertexFormat1>(m,"GfxVertexFormat")
      .def_readwrite("x",&GfxVertexFormat1::x)
      .def_readwrite("y",&GfxVertexFormat1::y)
      .def_readwrite("z",&GfxVertexFormat1::z)
      .def_readwrite("w",&GfxVertexFormat1::w)
      .def_readwrite("nx",&GfxVertexFormat1::nx)
      .def_readwrite("ny",&GfxVertexFormat1::ny)
      .def_readwrite("nz",&GfxVertexFormat1::nz)
      .def_readwrite("u",&GfxVertexFormat1::u)
      .def_readwrite("v",&GfxVertexFormat1::v)
      ;

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
      .def("dump_next_frame_to_png", &TinyOpenGL3App::dump_next_frame_to_png,
           py::arg("filename") = "image.png",
           py::arg("render_to_texture") = 1,
           py::arg("width") = -1,
           py::arg("height") = -1
          )
      .def("enable_render_to_texture",
           &TinyOpenGL3App::enable_render_to_texture)

      .def("cuda_register_texture_image",
           &TinyOpenGL3App::cuda_register_texture_image,
           py::arg("gl_texture_int"),
           py::arg("verbose")=true      
      )

      

      .def("set_up_axis", &TinyOpenGL3App::set_up_axis)
      .def("cuda_copy_texture_image", &TinyOpenGL3App::cuda_copy_texture_image,
            py::arg("cuda_resource_int"),
            py::arg("dest_memory_int"),
            py::arg("num_bytes"), 
            py::arg("gpu_device_destination") = true,
            py::arg("w_offset") = 0,
            py::arg("h_offset") = 0)

      .def("cuda_malloc", &TinyOpenGL3App::cuda_malloc,
                            py::arg("num_bytes"))

      .def("cuda_free", &TinyOpenGL3App::cuda_free,
           py::arg("cuda_ptr"))

      .def("cuda_map_vbo", &TinyOpenGL3App::cuda_map_vbo ,
          py::arg("verbose")=false)
      .def("cuda_unmap_vbo", &TinyOpenGL3App::cuda_unmap_vbo )

      .def("set_background_color", &TinyOpenGL3App::set_background_color)
      .def("dump_frames_to_video", &TinyOpenGL3App::dump_frames_to_video)
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
      
  
  py::class_<ReadPixelBuffer>(m, "ReadPixelBuffer")
  .def(py::init<TinyOpenGL3App&>())
  .def_readonly("rgba", &ReadPixelBuffer::rgba,          R"pbdoc( rgba pixel data)pbdoc")
  .def_readonly("depth", &ReadPixelBuffer::depth,
                    R"pbdoc( depth pixel data)pbdoc")

  ;


  py::class_<TinyViewportTile>(m, "TinyViewportTile")
          .def(py::init<>())
          .def_readwrite("visual_instances",
                     &TinyViewportTile::visual_instances)
           .def_readwrite("projection_matrix",
                    &TinyViewportTile::projection_matrix)
           .def_readwrite("view_matrix",
                     &TinyViewportTile::view_matrix)
           .def_readwrite("viewport_dims",
                     &TinyViewportTile::viewport_dims)

           ;

  

  py::class_<UrdfInstancePair>(m, "UrdfInstancePair")
          .def(py::init<>())
          .def_readwrite("link_index", &UrdfInstancePair::m_link_index)
          .def_readwrite("visual_instance", &UrdfInstancePair::m_visual_instance)
          .def_readwrite("viz_origin_xyz", &UrdfInstancePair::viz_origin_xyz)
          .def_readwrite("viz_origin_rpy", &UrdfInstancePair::viz_origin_rpy);
  
  

  py::class_<TinyCamera>(m, "TinyCamera")
    .def(py::init<>())
    .def("update", &TinyCamera::update)
    .def("set_camera_distance", &TinyCamera::set_camera_distance)
    .def("set_camera_pitch", &TinyCamera::set_camera_pitch)
    .def("set_camera_yaw", &TinyCamera::set_camera_yaw)
    .def("set_camera_up_vector",&TinyCamera::set_camera_up_vector)
    .def("set_camera_up_axis", &TinyCamera::set_camera_up_axis)
    .def("set_camera_target_position", &TinyCamera::set_camera_target_position)
    .def("get_camera_projection_matrix", &TinyCamera::get_camera_projection_matrix2)
    .def("get_camera_view_matrix", &TinyCamera::get_camera_view_matrix2)
      ;
      
  py::class_<TinyGLInstancingRenderer>(m, "TinyGLInstancingRenderer")
    .def("init", &TinyGLInstancingRenderer::init)
    .def("update_camera", &TinyGLInstancingRenderer::update_camera)
    .def("get_active_camera", &TinyGLInstancingRenderer::get_active_camera2)

    .def_property("view_matrix",
        [](const TinyGLInstancingRenderer& self) { 
            std::array<float, 16> viewMatrix;
            self.get_view_matrix(viewMatrix.data());
            return viewMatrix;
        },
        [](TinyGLInstancingRenderer& self, const std::array<float, 16>& viewMatrix) {
            self.set_view_matrix(viewMatrix.data());
        })
    .def_property("projection_matrix",
        [](const TinyGLInstancingRenderer& self) { 
            std::array<float, 16> projectionMatrix;
            self.get_projection_matrix(projectionMatrix.data());
            return projectionMatrix;
        },
        [](TinyGLInstancingRenderer& self, const std::array<float, 16>& projectionMatrix) {
            self.set_projection_matrix(projectionMatrix.data());
        })
    
    .def("register_shape", &TinyGLInstancingRenderer::register_shape1,
      py::arg("vertices"), py::arg("indices"), py::arg("textureIndex")=-1, py::arg("double_sided")=true)


    //.def("update_shape", &TinyGLInstancingRenderer::update_shape)
    .def("get_shape_vertex_count", &TinyGLInstancingRenderer::get_shape_vertex_count)
    .def("get_shape_vertex_offsets", &TinyGLInstancingRenderer::get_shape_vertex_offsets)

      

    .def("register_texture", &TinyGLInstancingRenderer::register_texture1)
    //.def("update_texture", &TinyGLInstancingRenderer::update_texture)
    .def("remove_texture", &TinyGLInstancingRenderer::remove_texture)

    .def("register_graphics_instance", &TinyGLInstancingRenderer::register_graphics_instance)
    .def("register_graphics_instances", &TinyGLInstancingRenderer::register_graphics_instances)

    .def("write_single_instance_transform_to_cpu", &TinyGLInstancingRenderer::write_single_instance_transform_to_cpu)
    .def("write_single_instance_color_to_cpu", &TinyGLInstancingRenderer::write_single_instance_color_to_cpu2)
    .def("write_single_instance_flags_to_cpu", &TinyGLInstancingRenderer::write_single_instance_flags_to_cpu)
      
    .def("render_scene", &TinyGLInstancingRenderer::render_scene)
    .def("render_scene2", &TinyGLInstancingRenderer::render_scene2)
 
    .def("render_scene_tiled", &TinyGLInstancingRenderer::render_scene_internal)
      
    .def("write_transforms", &TinyGLInstancingRenderer::write_transforms)
    .def("remove_all_instances", &TinyGLInstancingRenderer::remove_all_instances)
    .def("remove_graphics_instance", &TinyGLInstancingRenderer::remove_graphics_instance)
      
    //.def("get_active_camera", (TinyCamera* (TinyGLInstancingRenderer::*)()) &TinyGLInstancingRenderer::get_active_camera)
    .def("set_camera", &TinyGLInstancingRenderer::set_camera)
      
    .def("draw_line", &TinyGLInstancingRenderer::draw_line)
    .def("draw_lines", &TinyGLInstancingRenderer::draw_lines)
    .def("get_screen_width", &TinyGLInstancingRenderer::get_screen_width)
    .def("get_screen_height", &TinyGLInstancingRenderer::get_screen_height)
    .def("get_total_num_instances", &TinyGLInstancingRenderer::get_total_num_instances)
    .def("set_plane_reflection_shape_index", &TinyGLInstancingRenderer::set_plane_reflection_shape_index)
    ;
    
   py::class_<TinyWindowInterface>(m, "TinyWindowInterface")
  .def("requested_exit", &TinyWindowInterface::requested_exit)
  .def("set_request_exit", &TinyWindowInterface::set_request_exit2)

  .def("set_window_title", &TinyWindowInterface::set_window_title)
  .def("set_keyboard_callback", &TinyWindowInterface::set_keyboard_callback2)
  .def("set_mouse_move_callback", &TinyWindowInterface::set_mouse_move_callback2)
  .def("set_mouse_button_callback", &TinyWindowInterface::set_mouse_button_callback2)
  .def("set_wheel_callback", &TinyWindowInterface::set_wheel_callback2)
  .def("set_resize_callback", &TinyWindowInterface::set_resize_callback2)
  ;
  
  m.def("file_open_dialog", &file_open_dialog);
  m.def("set_cuda_path", &set_cuda_path);
  m.def("set_cudart_path", &set_cudart_path);
  
  m.def("load_obj_shapes", &my_load_obj_shapes);
  m.def("extract_path", &my_extract_path);
  m.def("compute_camera_view_matrix", &compute_camera_view_matrix);
  m.def("compute_camera_projection_matrix", &compute_camera_projection_matrix);
  
  m.def("get_triangle_vertex_shader", &get_triangle_vertex_shader);
  m.def("set_triangle_vertex_shader", &set_triangle_vertex_shader);
  
  m.def("get_triangle_fragment_shader", &get_triangle_fragment_shader);
  m.def("set_triangle_fragment_shader", &set_triangle_fragment_shader);
    
  m.def("get_use_shadowmap_instancing_vertex_shader",&get_use_shadowmap_instancing_vertex_shader);
  m.def("set_use_shadowmap_instancing_vertex_shader",&set_use_shadowmap_instancing_vertex_shader);
  
  m.def("get_create_shadowmap_instancing_vertex_shader",&get_create_shadowmap_instancing_vertex_shader);
  m.def("set_create_shadowmap_instancing_vertex_shader",&set_create_shadowmap_instancing_vertex_shader);

  m.def("get_use_shadowmap_instancing_fragment_shader",&get_use_shadowmap_instancing_fragment_shader);
  m.def("set_use_shadowmap_instancing_fragment_shader",&set_use_shadowmap_instancing_fragment_shader);
  
  m.def("get_create_shadowmap_instancing_fragment_shader",&get_create_shadowmap_instancing_fragment_shader);
  m.def("set_create_shadowmap_instancing_fragment_shader",&set_create_shadowmap_instancing_fragment_shader);

  m.def("get_segmentation_mask_instancing_vertex_shader",&get_segmentation_mask_instancing_vertex_shader);
  m.def("set_segmentation_mask_instancing_vertex_shader",&set_segmentation_mask_instancing_vertex_shader);

  m.def("get_segmentation_mask_instancing_fragment_shader",&get_segmentation_mask_instancing_fragment_shader);
  m.def("get_segmentation_mask_instancing_fragment_shader",&get_segmentation_mask_instancing_fragment_shader);

  m.def("get_instancing_vertex_shader", &get_instancing_vertex_shader);
  m.def("set_instancing_vertex_shader", &set_instancing_vertex_shader);

  m.def("get_instancing_fragment_shader", &get_instancing_fragment_shader);
  m.def("set_instancing_fragment_shader", &set_instancing_fragment_shader);



  py::class_<::tds::UrdfParser<MyAlgebra>>(m, "UrdfParser")
          .def(py::init<>())
      .def("load_urdf", &::tds::UrdfParser<MyAlgebra>::load_urdf,
          py::arg("file_name"),
          py::arg("verbose")=false
          )
      .def("load_urdf_from_string", &::tds::UrdfParser<MyAlgebra>::load_urdf_from_string)
      ;
  


  
  py::class_<::tds::UrdfStructures<MyAlgebra>>(m,"OpenGLUrdfStructures")
      .def(py::init<>())
      .def_readwrite("robot_name",
                     &::tds::UrdfStructures<MyAlgebra>::robot_name);
  

  py::class_<OpenGLUrdfVisualizer<MyAlgebra>>(m, "OpenGLUrdfVisualizer")
      .def(py::init<int,int, const char*, bool, int, int, int, int>(),
      py::arg("width")=1024,
      py::arg("height")=768,
      py::arg("title")="pytinyopengl3",
      py::arg("allow_retina")=1,
      py::arg("window_type")=0,
      py::arg("render_device")=-1,
      py::arg("max_num_object_capacity")=256 * 1024,
      py::arg("max_shape_capacity_in_bytes")= 128 * 1024 * 1024)

      .def("convert_visuals", &OpenGLUrdfVisualizer<MyAlgebra>::convert_visuals2)
      .def("render", &OpenGLUrdfVisualizer<MyAlgebra>::render,
          py::arg("do_swap_buffer")=true,
          py::arg("render_segmentation_mask")=false,
          py::arg("up_axis")=2,
          py::arg("write_transforms")=true
      )
      .def("render_tiled", &OpenGLUrdfVisualizer<MyAlgebra>::render_tiled,
          py::arg("tiles"),
          py::arg("do_swap_buffer")=true,
          py::arg("render_segmentation_mask")=false,
          py::arg("up_axis")=2,
          py::arg("write_transforms")=true
          )
      .def("swap_buffer", &OpenGLUrdfVisualizer<MyAlgebra>::swap_buffer)

      .def("create_instances",   &OpenGLUrdfVisualizer<MyAlgebra>::create_instances)
      .def("sync_visual_transforms",
           &OpenGLUrdfVisualizer<MyAlgebra>::sync_visual_transforms2,
          py::arg("all_instances"), py::arg("visual_world_transforms_array"),
           py::arg("visual_offset"), py::arg("sim_spacing"),
           py::arg("apply_visual_offset")=false,
          py::arg("link_mapping")=std::vector<int>()
          )

      .def_readwrite("opengl_app", &OpenGLUrdfVisualizer<MyAlgebra>::m_opengl_app)
      
      .def_readwrite("path_prefix", &OpenGLUrdfVisualizer<MyAlgebra>::m_path_prefix)
      ;

    PyModule_AddIntConstant(m.ptr(), "GL_TEXTURE_2D", GL_TEXTURE_2D);
    PyModule_AddIntConstant(m.ptr(), "SEGMENTATION_MASK_RENDERMODE", B3_SEGMENTATION_MASK_RENDERMODE);
    PyModule_AddIntConstant(m.ptr(), "DEFAULT_RENDERMODE", B3_DEFAULT_RENDERMODE);
    

  
  py::class_<TinyPose<float, FloatUtils>>(m, "TinyPosef")
      .def(py::init<>())
      .def(py::init<const TinyVector3<float, FloatUtils>&,const TinyQuaternion<float, FloatUtils>&>())
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
      .def(py::self * float())
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
        .def("get_column", &TinyMatrix3x3<float, FloatUtils>::get_column)
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
