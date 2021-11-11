import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import pytinydiffsim 


class CartpolePyTinyDiffSim(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self):
    
    self.tds_env = pytinydiffsim.CartpoleEnv()

    self._render_width = 128
    self._render_height = 128
    
    self.theta_threshold_radians = 12 * 2 * math.pi / 360
    self.x_threshold = 0.4  #2.4
    high = np.array([
        self.x_threshold * 2,
        np.finfo(np.float32).max, 
        self.theta_threshold_radians * 2,
        np.finfo(np.float32).max
    ]
    ,dtype=np.float32
    )

    self.force_mag = 10
    self.renderer = None
    self.result = None
    action_dim = 1
    action_high = np.array([self.force_mag] * action_dim, dtype=np.float32)
    self.action_space = spaces.Box(-action_high, action_high, dtype=np.float32)
    
    self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    self.seed()
    #    self.reset()
    self.viewer = None
    self._configure()

  def _configure(self, display=None):
    self.display = display

  

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    #print("seed=",seed)
    s = int(seed) & 0xffffffff
    #print("s=",s)
    self.tds_env.seed(s)
    return [seed]

  def rollout(self, max_steps, shift):
    res = self.tds_env.rollout(max_steps, shift)
    return res.total_reward, res.num_steps
  
  def policy(self, ob):
      ac = self.tds_env.policy(ob)
      return [ac]

  def step(self, action):
    force = action[0]
    result = self.tds_env.step(force)
    self.state = result.obs
    #print("state=",self.state)
    done = result.done
    self.result = result
    reward = 1.0
    return np.array(self.state), reward, done, {}

  def reset(self):
    self.state = self.tds_env.reset()
    #print("self.state=", self.state)
    self.result = None
    return np.array(self.state)

  def update_weights(self, weights):
    #print("dir(self.tds_env=)", dir(self.tds_env))
    self.tds_env.update_weights(weights)
    
  def render(self, mode='human', close=False):
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
        slider_model = self.scene.create_cube([15, 0.025, 0.025], [0,0,0], 1, 1,1)
        self.slider_instance = self.scene.create_object_instance(slider_model)
        self.scene.set_object_position(self.slider_instance, [0,0,1])
        
        cart_model = self.scene.create_cube([0.25, 0.25, 0.1], [100,100,200], 1, 1,1)
        self.cart_instance = self.scene.create_object_instance(cart_model)
        pole_model = self.scene.create_cube([0.025, 0.025, .5], [200,100,100], 1, 1,1)
        self.pole_instance = self.scene.create_object_instance(pole_model)
      
      if self.result!=None:
        self.scene.set_object_position(self.cart_instance, self.result.cart_graphics_pos)
        self.scene.set_object_orientation(self.cart_instance, self.result.cart_graphics_orn)
        self.scene.set_object_position(self.pole_instance, self.result.pole_graphics_pos)
        self.scene.set_object_orientation(self.pole_instance, self.result.pole_graphics_orn)
      
      eye = [0.,-5.2,1.]
      target = [0., 0., 1.]
      light = pytinyrenderer.TinyRenderLight(has_shadow=False)
      camera = pytinyrenderer.TinyRenderCamera(viewWidth=self._render_width,
                                               viewHeight=self._render_height,
                                                position=eye, target=target)
      
      img = self.scene.get_camera_image([self.slider_instance, self.cart_instance, self.pole_instance], light, camera)
      
      rgb_array = np.reshape(np.array(img.rgb,dtype=np.uint8), (img.height, img.width, -1))
      return rgb_array

    img = np.array([[[255,255,255]]*self._render_width]*self._render_height, dtype=np.uint8)
    rgb_array = np.reshape(np.array(img,dtype=np.uint8), (self._render_height, self._render_width, -1))
    return rgb_array

  def configure(self, args):
    pass
    
  def close(self):
    del self.tds_env
    
