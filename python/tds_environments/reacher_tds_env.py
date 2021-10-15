import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pytinydiffsim 


class ReacherPyTinyDiffSim(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self):
    
    self.tds_env = pytinydiffsim.ReacherEnv()

    self._render_width = 128
    self._render_height = 128
    self.force_mag = 1

    action_dim = 2
    action_high = np.array([self.force_mag] * action_dim, dtype=np.float32)
    self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
    high = np.array([1,1,1,1, 4,4,200,200,4,4])
    self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    self.seed()
    #    self.reset()
    self.viewer = None
    self.renderer = None
    self.result = None

    
    self._configure()
    

  def _configure(self, display=None):
    self.display = display

  

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    s = int(seed) & 0xffffffff
    self.tds_env.seed(s)
    return [seed]

  def rollout(self, max_steps, shift):
    res = self.tds_env.rollout(max_steps, shift)
    return res.total_reward, res.num_steps
  
  def policy(self, ob):
      ac = self.tds_env.policy(ob)
      return [ac]

  def step(self, action):
    force = action
    result = self.tds_env.step(force)
    self.state = result.obs
    #print("state=",self.state)
    done = result.done
    reward = result.reward
    self.result = result
    #print("step np.array(self.state)=",np.array(self.state))
    #print("step reward=",reward)
    #print("done=",done)
    
    return np.array(self.state), reward, done, {}

  def reset(self):
    #grav = pytinydiffsim.Vector3(0,0,-10)
    self.state = self.tds_env.reset()
    #print("reset self.state=", self.state)
    self.result = None
    return np.array(self.state)

  def update_weights(self, weights):
    #print("dir(self.tds_env=)", dir(self.tds_env))
    self.tds_env.update_weights(weights)
    
  def render(self, mode='human', close=False,  **kwargs):
    
    #print("ReacherPyTinyDiffSim render mode=",mode)
    #for k in kwargs:
    #  print("kwargs k=",k)
    #  print("kwargs[k]=",kwargs[k])
    if mode != "rgb_array":
      return []

    if mode == "rgb_array":
      if self.renderer==None:
        import pytinyrenderer
        self.scene = pytinyrenderer.TinySceneRenderer()
        class TextureRGB888:
          def __init__(self):
            self.pixels = [
                    255,0,0,#red, green, blue
                    0,255,0,
                    0,0,255,
                    255,255,255]
            self.width = 2
            self.height= 2
        
        texture = TextureRGB888()
        texture_target = TextureRGB888()
        texture_target.pixels = [32,255,32]
        texture_target.width=1
        texture_target.height=1

        up_axis = 2
        body0_model = self.scene.create_capsule(0.01,0.05,up_axis, texture.pixels, texture.width, texture.height)
        body1_model = self.scene.create_capsule(0.01,0.05,up_axis, texture.pixels, texture.width, texture.height)
        tip_model = self.scene.create_capsule(0.02,0.02,up_axis, texture.pixels, texture.width, texture.height)
        target_model = self.scene.create_capsule(0.02,0.02,up_axis, texture_target.pixels, texture_target.width, texture_target.height)
        self.body0_instance = self.scene.create_object_instance(body0_model)
        self.body1_instance = self.scene.create_object_instance(body1_model)
        self.tip_instance = self.scene.create_object_instance(tip_model)
        self.target_instance = self.scene.create_object_instance(target_model)
      
      if self.result!=None:
        self.scene.set_object_position(self.body0_instance, self.result.body0_graphics_pos)
        self.scene.set_object_orientation(self.body0_instance, self.result.body0_graphics_orn)
        self.scene.set_object_position(self.body1_instance, self.result.body1_graphics_pos)
        self.scene.set_object_orientation(self.body1_instance, self.result.body1_graphics_orn)
        self.scene.set_object_position(self.tip_instance, self.result.tip_graphics_pos)
        self.scene.set_object_orientation(self.tip_instance, self.result.tip_graphics_orn)
        self.scene.set_object_position(self.target_instance, self.result.target_graphics_pos)
      
      eye = [0.,-0.2,1.]
      target = [0., 0., 0.]
      light = pytinyrenderer.TinyRenderLight(has_shadow=False)
      camera = pytinyrenderer.TinyRenderCamera(viewWidth=self._render_width,
                                               viewHeight=self._render_height,
                                                position=eye, target=target)
      
      img = self.scene.get_camera_image([self.body0_instance,
                                            self.body1_instance,
                                            self.tip_instance,
                                            self.target_instance], light, camera)
      
      rgb_array = np.reshape(np.array(img.rgb,dtype=np.uint8), (img.height, img.width, -1))
      return rgb_array

    img = np.array([[[255,255,255]]*self._render_width]*self._render_height, dtype=np.uint8)
    rgb_array = np.reshape(np.array(img,dtype=np.uint8), (self._render_height, self._render_width, -1))
    return rgb_array

  def configure(self, args):
    pass
    
  def close(self):
    del self.tds_env
    
