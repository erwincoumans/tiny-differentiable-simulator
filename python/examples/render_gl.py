import pytinyopengl3 as p
app = p.TinyOpenGL3App("title")
app.renderer.init()
textureIndex = -1
shape = app.register_cube_shape(10, 10, 0.01, textureIndex, 40)
print("shape=",shape)
pos = p.TinyVector3f(0.,0.,0.)
orn = p.TinyQuaternionf(0.,0.,0.,1.)
color = p.TinyVector3f(1.,0.,0.)
scaling = p.TinyVector3f(1.,1.,1.)
opacity = 0.5
app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity)

while 1:
  app.renderer.update_camera(1)
  app.draw_grid()
  app.renderer.render_scene()
  app.draw_text_3d("hi",1,1,1,1)
  app.swap_buffer()

