import pytinyopengl3 as p
import math, time

app = p.TinyOpenGL3App("title")
app.renderer.init()
cam = p.TinyCamera()
cam.set_camera_distance(2.)
cam.set_camera_pitch(-20)
app.renderer.set_camera(cam)

pos = p.TinyVector3f(0.,0.,0.)
orn = p.TinyQuaternionf(0.,0.,0.,1.)
orn.set_euler_rpy(p.TinyVector3f(math.pi/2.0,0.,0.))

color = p.TinyVector3f(1.,1.,1.)

scaling = p.TinyVector3f(.1,0.1,0.1)
opacity = 0.4

app.renderer.write_transforms()

#bunny_file = "../../data/bunny.obj"
bunny_file = "../../data/cube.obj"

#bunny_file = "../../data/laikago/chassis.obj"



shapes = p.load_obj_shapes(app, bunny_file, pos, orn, scaling)
shape = shapes[0]

rebuild = True
cube_size = 0.1
for x in range (20):
  for y in range (20):
    for z in range (20):
      pos = p.TinyVector3f(x*cube_size,y*cube_size,z*cube_size)
      mesh = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity, rebuild)


while not app.window.requested_exit():
  app.renderer.update_camera(2)
  dg = p.DrawGridData()
  dg.drawAxis = True
  app.draw_grid(dg)
  from_line=p.TinyVector3f(0,0,0)
  to_line=p.TinyVector3f(1,1,1)
  color=p.TinyVector3f(1,1,1)
  width=2
  app.renderer.draw_line(from_line,to_line,color,width)
  pos = p.TinyVector3f(0,0,math.sin(time.time())+1)
  app.renderer.write_single_instance_color_to_cpu(color, opacity, mesh)
  app.renderer.write_single_instance_transform_to_cpu(pos, orn,mesh)
  app.renderer.write_transforms()  
  app.renderer.render_scene()
  app.draw_text_3d("hi",1,1,1,1)
  app.swap_buffer()

