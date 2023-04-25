import pytinydiffsim as tds
import time
import sys
import numpy
numpy.set_printoptions(threshold=sys.maxsize)
import numpy as np
import matplotlib.pyplot as plt
plt.ion()
img = np.random.rand(400, 400)
image = plt.imshow(img, interpolation='none')#, cmap='gray', vmin=0.8, vmax=1)
ax = plt.gca()

width = 1024
height = 768

if 1:
    num_actors = 2 #1024#4096
    auto_reset_when_done = True
    tds_robot = tds.VectorizedLaikagoEnv(num_actors, auto_reset_when_done)
    tds_robot.reset()
    actions = [[0] * tds_robot.action_dim()]*num_actors
    res = tds_robot.step(actions)


    import pytinyopengl3 as g
    viz = g.OpenGLUrdfVisualizer(width=width, height=height)
    viz.opengl_app.set_background_color(1,0,0)
    viz.opengl_app.swap_buffer()
    viz.opengl_app.swap_buffer()
    
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
      print("screen_width=",width)
      height = viz.opengl_app.renderer.get_screen_height()
      print("screen_height=",height)
      
      nx=40
      ny=40 
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


    cam = g.TinyCamera()
    cam.set_camera_up_axis(2)
    cam.set_camera_distance(1)
    cam.set_camera_pitch(-30)
    cam.set_camera_target_position(0.,0.,0.)
    viz.opengl_app.renderer.set_camera(cam)


            
    frame = 0
    #for frame in range (100):
    while not viz.opengl_app.window.requested_exit():
      frame=frame+1
    
      width = viz.opengl_app.renderer.get_screen_width()
      height = viz.opengl_app.renderer.get_screen_height()
      tile_width = int(width/nx)
      tile_height = int(height/ny)
    
      ct = time.time()
      
      cam = viz.opengl_app.renderer.get_active_camera()
      tile_index = 0
      for x in range (nx):
        for y in range (ny):
          tile = tiles[tile_index]
          tile_index+=1
          tile.view_matrix = cam.get_camera_view_matrix()
          tile.viewport_dims=[x*tile_width,y*tile_height,tile_width, tile_height]


      et = time.time()
      print("update viewports dt=",et-ct)
      
          
      ct = time.time()
      res = tds_robot.step(actions)
      et = time.time()
      print("tds_robot.step dt=",et-ct)
      
      #viz.sync_visual_transforms(all_instances, res.visual_world_transforms, tds_robot.obs_dim(), sim_spacing)
    
      #viz.render(do_swap_buffer=False, render_segmentation_mask=True)
      #name = "test_"+str(frame)+".png"
      #viz.opengl_app.dump_next_frame_to_png(filename=name, render_to_texture=False, width=19200, height=10800)
      ct = time.time()
      viz.render_tiled(tiles, do_swap_buffer = False, render_segmentation_mask=False)
      et = time.time()
      print("render dt=",et-ct)
      
      #viz.render()

      ct = time.time()
      pixels = g.ReadPixelBuffer(viz.opengl_app)
      et = time.time()
      print("ReadPixelBuffer dt=",et-ct)
      #print('pixels.rgba=', pixels.rgba)

      #np_img_arr = np.reshape(pixels.rgba, (height, width, 4))
      #np_img_arr = np_img_arr * (1. / 255.)
      #np_img_arr = np.flipud(np_img_arr)
    
      #image.set_data(np_img_arr)
      
      #np_depth_arr = np.flipud(np.reshape(pixels.depth, (height, width, 1)))
      #image.set_data(np_depth_arr)
      ct = time.time()
      viz.swap_buffer()
      et = time.time()
      print("swap_buffer dt=",et-ct)
      et = time.time()
      dt = et-st
      st = et
      
      #image.set_data(np_img_arr)
      ax.plot([0])
      #plt.draw()
      #plt.show()
      #plt.pause(0.0001)
        
      print(dt)
      print("fps = ", len(tiles)*(1./dt))
      


      #cam = viz.opengl_app.renderer.get_active_camera()
      #print("cam=",cam)
      #print("cam.get_camera_projection_matrix=",cam.get_camera_projection_matrix())
      #print("cam.get_camera_view_matrix=",cam.get_camera_view_matrix())
              
              
          
