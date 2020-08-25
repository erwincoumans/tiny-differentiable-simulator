import pytinyopengl3 as p
app = p.TinyOpenGL3App("title")
app.renderer.init()
#cam = app.renderer.get_active_camera()
#cam.set_camera_distance(2.)

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
        pixels[(i + j * width) * 3 + 1] = 0
        pixels[(i + j * width) * 3 + 2] = 255
     else:
      pixels[(i + j * width) * 3 + 0] = colorR
      pixels[(i + j * width) * 3 + 1] = colorG
      pixels[(i + j * width) * 3 + 2] = colorB


textureIndex = app.renderer.register_texture(pixels, width, height, False)
shape = app.register_cube_shape(1, 1, 0.01, textureIndex, 40)
print("shape=",shape)
pos = p.TinyVector3f(0.,0.,0.)
orn = p.TinyQuaternionf(0.,0.,0.,1.)
color = p.TinyVector3f(1.,1.,1.)
scaling = p.TinyVector3f(1.,1.,1.)
opacity = 1
app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity)
app.renderer.write_transforms()


while not app.window.requested_exit():
  app.renderer.update_camera(1)
  app.draw_grid()
  from_line=p.TinyVector3f(0,0,0)
  to_line=p.TinyVector3f(1,1,1)
  color=p.TinyVector3f(1,0,0)
  width=2
  app.renderer.draw_line(from_line,to_line,color,width)
  app.renderer.render_scene()
  app.draw_text_3d("hi",1,1,1,1)
  app.swap_buffer()

