import pytinyopengl3 as p
import math, time
import warp as wp
import numpy as np

import nvtx

wp.init()

device = "cuda" #wp.get_preferred_device()

num_objects = 1000000
#num_objects = 1000
#num_objects = 10

app = p.TinyOpenGL3App("warp_gl_interop", maxNumObjectCapacity=num_objects+10)
app.renderer.init()




cam = p.TinyCamera()
cam.set_camera_distance(2.)
cam.set_camera_pitch(-20)
app.renderer.set_camera(cam)
width = 256
height = 256
pixels = [255] * width * height * 3
colorR = 255
colorG = 255
colorB = 255

for i in range(width):
  for j in range(height):
     a = i < width / 2
     b = j < width / 2
     if (a == b):
        pixels[(i + j * width) * 3 + 0] = 0
        pixels[(i + j * width) * 3 + 1] = 255
        pixels[(i + j * width) * 3 + 2] = 255
     else:
      pixels[(i + j * width) * 3 + 0] = colorR
      pixels[(i + j * width) * 3 + 1] = colorG
      pixels[(i + j * width) * 3 + 2] = colorB


textureIndex = app.renderer.register_texture(pixels, width, height, False)
pos = p.TinyVector3f(0.,0.,1.)
orn = p.TinyQuaternionf(0.,0.,0.,1.)
color = p.TinyVector3f(1.,1.,1.)
scaling = p.TinyVector3f(0.1,0.1,0.1)
opacity = 1
rebuild = True
#shape = app.register_graphics_unit_sphere_shape(p.EnumSphereLevelOfDetail.SPHERE_LOD_HIGH, textureIndex)
shape = app.register_cube_shape(1,1,1, textureIndex,4)


@wp.kernel
def init_vec(a: wp.array(dtype=wp.vec4)):
      tid = wp.tid()
      a[tid] = wp.vec4(0.,0.,float(tid)*1.5, 1.0)
      


vec_pos = [p.TinyVector3f(0.,0.,0.)]*num_objects
vec_orn = [p.TinyQuaternionf(0.,0.,0.,1.)]*num_objects
vec_scaling = [p.TinyVector3f(1.,1.,1.)]*num_objects
vec_color = [p.TinyVector3f(1.,1.,1.)]*num_objects
vec_scaling = [p.TinyVector3f(0.1,0.1,0.1)]*num_objects

sphere = app.renderer.register_graphics_instances(shape, vec_pos, vec_orn, vec_color, vec_scaling, opacity, rebuild)


shape = app.register_cube_shape(1, 1, 0.01, textureIndex, 40)
#print("shape=",shape)


ground_plane=True
if ground_plane:
  pos = p.TinyVector3f(0.,0.,0.)
  scaling = p.TinyVector3f(1.,1.,1.)
  app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity, rebuild)


app.renderer.write_transforms()







velocities = wp.array(
    np.array([0.0, 0.0, 0.1, 0.0]*num_objects),
    device=device,
    dtype=wp.vec4,
    requires_grad=False,
  )

@wp.kernel
def init_kernel(a: wp.array(dtype=wp.vec4), sim_spacing: float, square_id: int):
      tid = wp.tid()
      a[tid] = wp.vec4(sim_spacing * float(tid % square_id) - float(square_id) * sim_spacing / 2.,
                       sim_spacing * float(tid / square_id) - float(square_id) * sim_spacing / 2.,
                       0.,1.)
      



vbo = app.cuda_map_vbo()
sim_spacing = 0.3
positions = wp.array(ptr=vbo.positions,dtype=wp.vec4, shape=(num_objects,), length=num_objects,capacity=num_objects,device=device, owner=False, ndim=1)
wp.launch(init_kernel, device=device, dim=num_objects, inputs=[positions, sim_spacing, int(math.sqrt(num_objects))])
app.cuda_unmap_vbo()


# Example reading/writing OpenGL vertices (in VBO) using CUDA interop
if 1:
    print("shape=",shape)
    vcnt = app.renderer.get_shape_vertex_count()
    total_vertices = 0
    for vc in vcnt:
        total_vertices += vc
        
    print("vcnt2 =",vcnt )
    #vertex offset into VBO for each shape
    voffsets = app.renderer.get_shape_vertex_offsets()
    print("voffsets2=",voffsets)

    vbo = app.cuda_map_vbo()

    num_vertices = vcnt[shape]
    print("num_vertices=",num_vertices)
    offset = voffsets[shape]
    vertex_stride = 9*4 #9 floats [x,y,z,w, nx,ny,nz, u,v]
    print("total_vertices=",total_vertices)

    a = np.array([-1.0, -1.0, -1.0, -1.0]*num_vertices)
     
    vertices_dest = wp.array(
        a,
        dtype=wp.vec4,
        device=device,
        requires_grad=False,
    )

    vertices_src = wp.array(ptr=vbo.vertices,dtype=wp.vec4, shape=(total_vertices,), strides=(vertex_stride,), length=total_vertices,capacity=total_vertices,device=device, owner=False, ndim=1)

    @wp.kernel
    def modify_vertices_kernel(src: wp.array(dtype=wp.vec4), dst: wp.array(dtype=wp.vec4), offset: wp.int32, x_y_dim: float):
          tid = wp.tid()
          v = src[tid+offset]
          src[tid+offset] = wp.vec4(x_y_dim*v[0],x_y_dim*v[1],-0.01,v[3])
          dst[tid] = src[tid+offset]
          
    x_y_dim = sim_spacing*math.sqrt(num_objects)/2.
    wp.launch(modify_vertices_kernel, device=device, dim=num_vertices, inputs=[vertices_src, vertices_dest, offset, x_y_dim])

    app.cuda_unmap_vbo()

    #debug values
    print("vertices_dest=",vertices_dest)


@wp.kernel
def test_kernel(a: wp.array(dtype=wp.vec4),
    b: wp.array(dtype=wp.vec4),
    dt: float):
      
      tid = wp.tid()
      a[tid] = a[tid] + b[tid]*dt
    

vcnt = app.renderer.get_shape_vertex_count()
print("vcnt =",vcnt )
app.renderer.update_camera(2)
dg = p.DrawGridData()
dg.drawAxis = True
app.draw_grid(dg)

#stop_app = False

def my_mouse_move_callback(a,b):
  print("mouse move:",a,b)
  
app.window.set_mouse_move_callback(my_mouse_move_callback)

def my_mouse_button_callback(a,b,c,d):
  print("mouse button:",a,b,c,d)
  
app.window.set_mouse_button_callback(my_mouse_button_callback)
  
def my_resize_callback(a,b):
  print("resize:",a,b)
app.window.set_resize_callback(my_resize_callback)

def my_wheel_callback(a,b):
  print("wheel:",a,b)
app.window.set_wheel_callback(my_wheel_callback)

def my_keyboard_callback(a,b):
  print("key:", a,b)
  if a==27:
    print("requesting exit!")
    app.window.set_request_exit()
app.window.set_keyboard_callback(my_keyboard_callback)

while not app.window.requested_exit():

  with nvtx.annotate("sync_visual_transforms", color="orange"):  
    vbo = app.cuda_map_vbo()
    positions = wp.array(ptr=vbo.positions,dtype=wp.vec4, shape=(num_objects,), length=num_objects,capacity=num_objects,device=device, owner=False, ndim=1)
    wp.launch(test_kernel, device=device, dim=num_objects, inputs=[positions, velocities, 1./240.])
    app.cuda_unmap_vbo()

  
  if 0:
    from_line=p.TinyVector3f(0,0,0)
    to_line=p.TinyVector3f(1,1,1)
    color=p.TinyVector3f(1,0,0)
    width=2
    app.renderer.draw_line(from_line,to_line,color,width)
  
  with nvtx.annotate("render_scene", color="blue"):  
    app.renderer.render_scene()
    
  #app.draw_text_3d("hi",1,1,1,1)
  with nvtx.annotate("swap_buffer", color="green"):  
    app.swap_buffer()


