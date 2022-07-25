import pytinydiffsim as tds
import time

num_actors = 2#128 #1024#4096
auto_reset_when_done = True
tds_robot = tds.VectorizedLaikagoEnv(num_actors, auto_reset_when_done)
tds_robot.reset()
actions = [[0] * tds_robot.action_dim()]*num_actors
res = tds_robot.step(actions)


import pytinyopengl3 as g
viz = g.OpenGLUrdfVisualizer()
urdf = g.OpenGLUrdfStructures()
parser = g.UrdfParser()
file_name = tds_robot.urdf_filename()
urdf = parser.load_urdf(file_name)
print("urdf=",urdf)
texture_path = "laikago_tex.jpg"
viz.path_prefix = g.extract_path(file_name)
print("viz.path_prefix=",viz.path_prefix)
viz.convert_visuals(urdf, texture_path)
print("create_instances")

all_instances_prev = viz.create_instances(urdf, texture_path, num_actors)
all_instances = viz.create_instances(urdf, texture_path, num_actors)

#print("all_instances=",all_instances)
#print("all_instances[0]=",all_instances[0])

for i in all_instances[1]:
  print(i.visual_instance)
  
#sync transforms
#for pairs in all_instances:
#  for pair in pairs:
#    print("pair.link=", pair.link_index, " pair.visual_instance=", pair.visual_instance)
sim_spacing = 0




print("len(all_instances)=",len(all_instances))
print("\nhold CTRL and right/left/middle mouse button to rotate/zoom/move camera")


st = time.time()

if 1:
  width = viz.opengl_app.renderer.get_screen_width()
  height = viz.opengl_app.renderer.get_screen_height()
  nx=30
  ny=30  
  tile_width = int(width/nx)
  tile_height = int(height/ny)
  
  tiles=[]
  
  
  for x in range (nx):
    for y in range (ny):
      tile = g.TinyViewportTile()
      tile.visual_instances = [35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67]
      #tile.visual_instances = [35,37,39,41]^M
      cam = viz.opengl_app.renderer.get_active_camera()
      tile.projection_matrix = cam.get_camera_projection_matrix()
      tile.view_matrix = cam.get_camera_view_matrix()
      tile.viewport_dims=[x*tile_width,y*tile_height,tile_width, tile_height]
      tiles.append(tile)  

while 1:  

  width = viz.opengl_app.renderer.get_screen_width()
  height = viz.opengl_app.renderer.get_screen_height()
  tile_width = int(width/nx)
  tile_height = int(height/ny)

  cam = viz.opengl_app.renderer.get_active_camera()
  tile_index = 0
  for x in range (nx):
    for y in range (ny):
      tile = tiles[tile_index]
      tile_index+=1
      tile.view_matrix = cam.get_camera_view_matrix()
      tile.viewport_dims=[x*tile_width,y*tile_height,tile_width, tile_height]

  res = tds_robot.step(actions)
  viz.sync_visual_transforms(all_instances, res.visual_world_transforms, tds_robot.obs_dim(), sim_spacing)

  #viz.render()
  viz.render2(tiles)

  et = time.time()
  dt = et-st
  st = et
  print(dt)
  print("fps = ", len(tiles)*(1./dt))
  
  #cam = viz.opengl_app.renderer.get_active_camera()
  #print("cam=",cam)
  #print("cam.get_camera_projection_matrix=",cam.get_camera_projection_matrix())
  #print("cam.get_camera_view_matrix=",cam.get_camera_view_matrix())
          
          
          
