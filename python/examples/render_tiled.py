#for pytinyopengl3: pip install pytinydiffsim, use latest, at least version >= 0.5.0
import pytinyopengl3 as g
import math
from numpngw import write_apng
frames = []

import torch
import numpy as np
use_cv2 = False
if use_cv2:
  import cv2

#from omni.isaac.core.utils.nucleus import get_assets_root_path
#assets_root_path = get_assets_root_path()
#if assets_root_path is None:
#    print("Cannot find Isaac Asset Root Path")                
#else:
#    print("assets_root_path=",assets_root_path)

use_cuda_interop = True



use_image_observation = True

class CartpoleTest:
    def __init__(self, num_envs=256, enable_tiled = False, use_matplot_lib = False):
        self.num_envs = num_envs
        self.use_camera = True
        self.camera_image_stack = 1
        self.camera_type = "rgbd"
        self.enable_tiled = enable_tiled
        self.once = True
        self.use_matplot_lib = use_matplot_lib
        if enable_tiled and self.use_matplot_lib:
          import matplotlib.pyplot as plt
          import matplotlib
          self.plt = plt
          self.plt.ion()
          img = np.random.rand(2000, 2000)
          self.matplotlib_image = self.plt.imshow(img, interpolation='none')#, cmap='gray', vmin=0.8, vmax=1)
          self.ax = self.plt.gca()
        else:
          self.use_matplot_lib = False

        self.counter=0 
        if self.camera_type == "rgb":
            self.camera_channels = 3
        elif self.camera_type == "grey":
            self.camera_channels = 1
        elif self.camera_type == "depth":
            self.camera_channels = 1
        elif self.camera_type == "rgbd":
            self.camera_channels = 4
        else:
            raise NotImplementedError(f"Unsupported camera type {self.camera_type}")

        self.camera_width = 512
        self.camera_height = 512 
        self.device = "cuda"
        if self.use_camera:
      
            self.max_x = math.ceil(math.sqrt(self.num_envs))
            self.tile_width = self.camera_width
            self.tile_height = self.camera_height
            self.width=self.tile_width * self.max_x
            self.height=self.tile_height * self.max_x
            
            
            num_actors=self.num_envs
            
            self.camera_image_stack=1
            self.num_stacked_channels = self.camera_channels + self.camera_image_stack*self.camera_channels
            #self.obs_space = spaces.Box(np.ones((self.camera_height, self.camera_width, self.num_stacked_channels)) * -np.Inf,
            #                        np.ones((self.camera_height, self.camera_width, self.num_stacked_channels)) * np.Inf)

            #self.obs_buf = torch.zeros(
            #    (self.num_envs, self.camera_height, self.camera_width, self.num_stacked_channels), device=self.device, dtype=torch.float)
            self.obs_buf = torch.zeros(
                (self.num_envs, self.camera_height, self.camera_width, self.num_stacked_channels), device=self.device, dtype=torch.float)
            print("####################################self.obs_buf.shape=",self.obs_buf.shape)
            
            #self.obs_buf = torch.zeros(
            #    (self.num_envs, self.camera_height* self.camera_width* self.camera_channels), device=self.device, dtype=torch.float)
            #print("self.obs_buf.shape=",self.obs_buf.shape)
     
            if self.enable_tiled:
              window_type = 2
            else:
              window_type = 0
            self.viz = g.OpenGLUrdfVisualizer(width=self.width, height=self.height, window_type=window_type)
            self.viz.opengl_app.set_background_color(1.,1.,1.)
            self.viz.opengl_app.swap_buffer()
            self.viz.opengl_app.swap_buffer()

            if use_cuda_interop:
              self.render_texid = self.viz.opengl_app.enable_render_to_texture(self.width, self.height)
              self.viz.opengl_app.swap_buffer()
              self.viz.opengl_app.swap_buffer()
              self.cuda_tex = self.viz.opengl_app.cuda_register_texture_image(self.render_texid, True)
              self.cuda_num_bytes = self.width*self.height*4*2 #4 component half-float, each half-float 2 bytes
              print("cuda_num_bytes=", self.cuda_num_bytes)
              self.ttensor = torch.zeros(self.width*self.height*4, dtype=torch.float16, device=self.device)
              self.cuda_mem = self.ttensor.data_ptr()
              

            urdf = g.OpenGLUrdfStructures()
            parser = g.UrdfParser()
            file_name = "can.urdf"
            urdf = parser.load_urdf(file_name)
            print("urdf=",urdf)
            texture_path = "texture_map.png"
            self.viz.path_prefix = g.extract_path(file_name)
            print("viz.path_prefix=",self.viz.path_prefix)
            self.viz.convert_visuals(urdf, texture_path)
            print("create_instances")

            self.all_instances = self.viz.create_instances(urdf, texture_path, num_actors)
            #print("self.all_instances=",self.all_instances)
            verbose_print = False
            if verbose_print:
              print("len(self.all_instances)=",len(self.all_instances))
              for pairs in self.all_instances:
                print("len(pairs)=", len(pairs))
                for pair in pairs:
                 print("pair.visual_instance=",pair.visual_instance)
          
            if self.enable_tiled:
              
              print("tile_width=", self.tile_width)
              print("tile_height=", self.tile_height)
              print("self.num_envs=", self.num_envs)
              self.tiles=[]
              x=0
              y=0
              for t in range (self.num_envs):
                  tile = g.TinyViewportTile()
                  pairs = self.all_instances[t]
                  viz_instances = []
                  for pair in pairs:
                    viz_instances.append(pair.visual_instance)
                  print("viz_instances=",viz_instances)
                  tile.visual_instances = viz_instances#[t, 512+t, 1024+t]
                  print("tile.visual_instances=",tile.visual_instances)
                  cam = self.viz.opengl_app.renderer.get_active_camera()
                  tile.projection_matrix = cam.get_camera_projection_matrix()
                  tile.view_matrix = cam.get_camera_view_matrix()
                  tile.viewport_dims=[x*self.tile_width,y*self.tile_height,self.tile_width, self.tile_height]
                  self.tiles.append(tile)
                  x+=1
                  if x>=self.max_x:
                    x=0
                    y+=1

            cam = g.TinyCamera()
            cam.set_camera_up_axis(2)
            cam.set_camera_distance(0.7)
            cam.set_camera_pitch(0)#-30)
            cam.set_camera_yaw(90)#-30)
            cam.set_camera_target_position(0.,0.,0.0)
            #if not self.enable_tiled:
            self.viz.opengl_app.renderer.write_transforms()
            self.viz.opengl_app.renderer.set_camera(cam)
    def sync_transforms_cpu(self, rb_transforms):
        skip = 0
        if self.enable_tiled:
          sim_spacing = 0
        else:
          sim_spacing = 10
        self.viz.sync_visual_transforms(self.all_instances, rb_transforms, skip, sim_spacing,apply_visual_offset=True)


    def update_observations(self, write_transforms, camera_positions=None):
        if self.use_camera:
          import time
          start_time = time.time()
          
          #print("len(camera_positions)=",len(camera_positions))
          if use_cuda_interop:
            self.viz.opengl_app.enable_render_to_texture(self.width, self.height)
          
          if self.enable_tiled:
              
              cam = self.viz.opengl_app.renderer.get_active_camera()
              tile_index = 0
              x=0
              y=0
              #self.max_x
              for tile_index in range (self.num_envs):
                  tile = self.tiles[tile_index]
                  
                  if camera_positions is None:
                    tile.view_matrix = cam.get_camera_view_matrix()
                  else:
                    cam_target = g.TinyVector3f(camera_positions[tile_index][0],
                                                camera_positions[tile_index][1],
                                                camera_positions[tile_index][2])
                    cam_up = g.TinyVector3f(0. ,0., 1.)
                    cam_pos = g.TinyVector3f(camera_positions[tile_index][0],
                                                camera_positions[tile_index][1]-1.5,
                                                camera_positions[tile_index][2])

                    view_mat = g.compute_camera_view_matrix(cam_pos, cam_target, cam_up)
                    tile.view_matrix = view_mat

                  tile.viewport_dims=[x*self.tile_width,y*self.tile_height,self.tile_width, self.tile_height]
                  x+=1
                  if x>=self.max_x:
                    x=0
                    y+=1
              
              self.viz.render_tiled(self.tiles, do_swap_buffer = False, render_segmentation_mask=False, write_transforms=write_transforms)
          else:
            self.viz.render(do_swap_buffer=False, render_segmentation_mask=False, write_transforms=write_transforms)
            #up_axis = 2
            #self.viz.opengl_app.renderer.update_camera(up_axis)
            #self.viz.opengl_app.renderer.render_scene()
            #self.viz.opengl_app.swap_buffer()
          
          ct = time.time()
          if use_cuda_interop:
            self.viz.opengl_app.cuda_copy_texture_image(self.cuda_tex, self.cuda_mem, self.cuda_num_bytes)
            #print("self.ttensor.shape=",self.ttensor.shape)
            #print("self.ttensor=",self.ttensor)
          else:
            pixels = g.ReadPixelBuffer(self.viz.opengl_app)
          et = time.time()
          #print("cuda_copy_texture_image dt=", et-ct)
          
          
          end_time = time.time()
          #print("duration =", end_time-start_time)
          #print("fps =", float(self.num_envs)/(end_time-start_time))
          
          
          if self.use_matplot_lib:
            if use_cuda_interop:
              ftensor = self.ttensor.type(torch.float32)
              np_img_arr = ftensor.cpu().numpy()
              np_img_arr = np.reshape(np_img_arr, (self.height, self.width, 4))
              np_img_arr = np.flipud(np_img_arr)
            else:
              np_img_arr = pixels.rgba
              np_img_arr = np.reshape(np_img_arr, (self.height, self.width, 4))
              np_img_arr = np_img_arr * (1. / 255.)
              np_img_arr = np.flipud(np_img_arr)
          
            self.matplotlib_image.set_data(np_img_arr)
            self.ax.plot([0])
            self.plt.show()
            self.plt.pause(0.0001)
  
          self.viz.swap_buffer()
        

          #print("self.dof_pos[env_ids, 0].shape=",self.dof_pos[env_ids, 0].shape)
          #sq = self.dof_pos[env_ids, 0].squeeze()
          
          #print("sq.shape=",sq.shape)
          if use_image_observation:
            #this copy/reshaping is sub-optimal, need a person with some pytorch-fu
            #ftensor = self.ttensor.type(torch.float32)*255.
            ftensor = self.ttensor.type(torch.float32)
            ftensor  = torch.reshape(ftensor, (self.height, self.width, 4))
            #ftensor = torch.flipud(ftensor)
            ftensor = ftensor.reshape(self.max_x, self.tile_width, self.max_x, self.tile_height, 4)
            ftensor = ftensor.swapaxes(1,2)
            ftensor = ftensor.reshape(self.max_x*self.max_x, self.tile_width*self.tile_height*4)
            ftensor = ftensor[:self.num_envs,]
            #print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            #print("ftensor.shape=",ftensor.shape)
            ftensor = ftensor.reshape(self.max_x*self.max_x, self.tile_width,self.tile_height,4)
            #self.obs_buf = ftensor  
            #print("ftensor.shape=", ftensor.shape)
            #self.obs_buf = ftensor
          
            #self.gym.render_all_camera_sensors(self.sim)
            #self.gym.start_access_image_tensors(self.sim)
            stk = self.camera_image_stack
            #print("stk=", stk)
            if stk > 0:
                # move the previous (stack-1) frames 1 step forward in the buffer
                self.obs_buf[:, :, :, self.camera_channels] = self.obs_buf[:, :, :, 0]
                #self.obs_buf[:, :, :, (1) * self.camera_channels: (stk) * self.camera_channels] 
                #self.obs_buf[:, :, :, (0) * self.camera_channels: (stk-1) * self.camera_channels]
                  
            #print("self.obs_buf.shape=",self.obs_buf.shape)
            #print("ftensor.shape=", ftensor.shape)
            #print("self.obs_buf.shape=", self.obs_buf.shape)
            

            self.obs_buf[:, :, :, 0:self.camera_channels] = ftensor
            #print("self.camera_channels=",self.camera_channels)
            #print("self.obs_buf.shape=", self.obs_buf.shape)
            self.counter += 1
            if self.counter == 1:
              for id in np.arange(self.num_envs):
                #    camera_gpu_tensor = self.camera_tensors[id][:, :, 0:self.camera_channels].clone()
                #    #if (id==0):
                #    #  print("camera_gpu_tensor=",camera_gpu_tensor.float())
                #    #print(camera_gpu_tensor.shape)
                #    self.obs_buf[id, :, :, 0:self.camera_channels] = camera_gpu_tensor.float()
                if use_cv2:
                   if id == 13:
                     cv2.imshow("image", self.obs_buf[id, :, :, 0:3].cpu().numpy() )
                     #cv2.imshow("image", self.obs_buf[id, :, :, 0:3].cpu().numpy())
                     cv2.waitKey(1000)

        else:
          self.obs_buf[env_ids, 0] = self.dof_pos[env_ids, 0].squeeze()
          self.obs_buf[env_ids, 1] = self.dof_vel[env_ids, 0].squeeze()
          self.obs_buf[env_ids, 2] = self.dof_pos[env_ids, 1].squeeze()
          self.obs_buf[env_ids, 3] = self.dof_vel[env_ids, 1].squeeze()

        return self.obs_buf    


if __name__ == '__main__':
  cam = CartpoleTest(num_envs = 4, enable_tiled = True, use_matplot_lib = True)

  for f in range (10):
    
    #cam.sync_transforms_cpu(rb)
    cam.update_observations(write_transforms=True)
    ftensor = cam.ttensor.type(torch.float32)
    np_img_arr = ftensor.cpu().numpy()
    np_img_arr = np.reshape(np_img_arr, (cam.height, cam.width, 4))
    np_img_arr = np.flipud(np_img_arr)
    frame_img = np_img_arr[:,:,:3]
    frame_img = np.array(frame_img * 255, dtype=np.uint8)
    frames.append(frame_img)

  write_apng("tiled_image.png", frames, delay=100)
