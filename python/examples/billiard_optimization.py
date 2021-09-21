
#this is an example of using auto diff gradients versus finite differences
#note that the example is not very efficient, due to forward mode differentiation and inefficient dual number implementation.
#so finite differences performs very similar to forward mode auto differentiation in this example.
#using an efficient reverse mode auto diff makes it more suitable

import pytinyopengl3 as p

use_auto_diff = False
if use_auto_diff:
  import pytinydiffsim_dual as dp
else:
  import pytinydiffsim as dp
  
import copy

import math, time

def rollout(force_x, force_y, step, render):

  if render:
    app.renderer.remove_all_instances()
  
  bodies = []
  visuals=[]

  dt = dp.fraction(1, 60)
  gravity_z = dp.fraction(0,1)
  world = dp.TinyWorld()
  world.gravity = dp.Vector3(dp.fraction(0,1),dp.fraction(0,1),dp.fraction(0,1))
    
  radius = dp.fraction(1,2)
  mass = dp.fraction(1,1)
  deg_60 = dp.pi() / dp.fraction(3, 1) #even triangle
    
  dx = dp.cos(deg_60) * radius * dp.fraction(2,1)
  dy = dp.sin(deg_60) * radius * dp.fraction(2,1)
  rx = dp.fraction(0,1)
  y = dp.fraction(0,1)
  
  target_pos = dp.Vector3(dp.fraction(35, 10),dp.fraction(8, 1), dp.fraction(0,1))
  ball_id = 0
  target_id = 5
  rebuild = True
  if render:
    orn = p.TinyQuaternionf(0.,0.,0.,1.)
    pos = p.TinyVector3f(dp.get_debug_double(target_pos[0]),dp.get_debug_double(target_pos[1]),dp.get_debug_double(target_pos[2]))
    color = p.TinyVector3f(0,0,1)
    opacity = 1
    scaling = p.TinyVector3f(dp.get_debug_double(radius),dp.get_debug_double(radius),dp.get_debug_double(radius))
    textureIndex = -1
    shape = app.register_graphics_unit_sphere_shape(p.EnumSphereLevelOfDetail.SPHERE_LOD_HIGH, textureIndex)
    sphere_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity, rebuild)
    
  
  geoms = []
  for column in [1,2,3]:
    x = dp.copy(rx)
    i=0
    while (i<column):
      geom = dp.TinySphere(radius)
      geoms.append(geom)
      body = dp.TinyRigidBody(mass, geom)
      body.world_pose.position = dp.Vector3(x, y, dp.fraction(0,1))
      bodies.append(body)
      
      if render:
        orn = p.TinyQuaternionf(0.,0.,0.,1.)
        pos = p.TinyVector3f(dp.get_debug_double(body.world_pose.position[0]),
                             dp.get_debug_double(body.world_pose.position[1]),
                             dp.get_debug_double(body.world_pose.position[2]))
        if ball_id == target_id:
          color = p.TinyVector3f(1,0,0)
        else:
          color = p.TinyVector3f(1,1,1)
        opacity = 1
        scaling = p.TinyVector3f(dp.get_debug_double(radius),dp.get_debug_double(radius),dp.get_debug_double(radius))
        textureIndex = -1
        shape = app.register_graphics_unit_sphere_shape(p.EnumSphereLevelOfDetail.SPHERE_LOD_HIGH, textureIndex)
        rebuild = True
        sphere_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity, rebuild)
        visuals.append(sphere_id)
      
      ball_id += 1
      x += radius * dp.fraction(2,1)
      i+=1
    rx = rx - dx
    y = y + dy
  
  #white player ball
  white = dp.Vector3(dp.fraction(0,1), -dp.fraction(2,1), dp.fraction(0,1))
  white_geom = dp.TinySphere(radius);
  white_ball = dp.TinyRigidBody(mass, white_geom)
  white_ball.world_pose.position = white
  bodies.append(white_ball)
  white_ball.apply_central_force(dp.Vector3(force_x, force_y, dp.fraction(0,1)))

  if render:
    orn = p.TinyQuaternionf(0.,0.,0.,1.)
    pos = p.TinyVector3f(dp.get_debug_double(white_ball.world_pose.position[0]),
                         dp.get_debug_double(white_ball.world_pose.position[1]),
                         dp.get_debug_double(white_ball.world_pose.position[2]))
    color = p.TinyVector3f(1,1,1)
    opacity = 1
    scaling = p.TinyVector3f(dp.get_debug_double(radius),
                             dp.get_debug_double(radius),
                             dp.get_debug_double(radius))
    textureIndex = -1
    shape = app.register_graphics_unit_sphere_shape(p.EnumSphereLevelOfDetail.SPHERE_LOD_HIGH, textureIndex)
    sphere_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity, rebuild)
    visuals.append(sphere_id)

  rb_solver = dp.TinyConstraintSolver()
  
  if render:
    app.renderer.write_transforms()
  #world.step(dt)
  for iter in range(step):
    
    for b in bodies:
          b.apply_gravity(world.gravity)
          b.apply_force_impulse(dt)
          b.clear_forces()
    
    dispatcher = world.get_collision_dispatcher()
    contacts = world.compute_contacts_rigid_body(bodies,dispatcher)
    #print("contacts=",contacts)
    num_solver_iterations = 50
    for solver_iter in range(num_solver_iterations):
      for c in contacts:
        rb_solver.resolve_collision(c,dt)
    for b in bodies:
      b.integrate(dt)
    #sync visual transforms
    if render:
      for v in range (len(bodies)):
        #print("v=",v)
        b = bodies[v]
        pos = p.TinyVector3f(dp.get_debug_double(b.world_pose.position[0]),
                             dp.get_debug_double(b.world_pose.position[1]),
                             dp.get_debug_double(b.world_pose.position[2]))
        #print("pos=",pos)
        orn = p.TinyQuaternionf(0,0,0,1)
        app.renderer.write_single_instance_transform_to_cpu(pos, orn, visuals[v])
      
      app.renderer.write_transforms()
      
      app.renderer.update_camera(2)
      dg = p.DrawGridData()
     
      app.draw_grid(dg)
      app.renderer.render_scene()
      app.swap_buffer()
      
    
  #print("finished step!")
  diff = bodies[target_id].world_pose.position - target_pos
  cost = diff.sqnorm();
  print("cost=",cost)
  return cost

def grad_finite(force_x, force_y, steps = 300, eps = dp.fraction(1,10000)):
  cost = rollout(force_x, force_y, steps, False)
  cx = rollout(force_x + eps, force_y, steps, False)
  cy = rollout(force_x, force_y + eps, steps, False)
  d_force_x = (cx - cost) / eps
  d_force_y = (cy - cost) / eps
  return cost, d_force_x, d_force_y


def grad_dual(force_x, force_y, steps = 300, eps = dp.fraction(1,10000)):
  
  fx = dp.TinyDualDouble(force_x.real(), 1.)
  fy = dp.TinyDualDouble(force_y.real(), 0.)

  c = rollout(fx, fy, steps, False)
  cost = c.real()
  d_force_x = dp.TinyDualDouble(c.dual(), 0.)

  fx = dp.TinyDualDouble(force_x.real(), 0.)
  fy = dp.TinyDualDouble(force_y.real(), 1.0)

  c = rollout(fx, fy, steps, False)
  d_force_y = dp.TinyDualDouble(c.dual(), 0.)
  return cost, d_force_x, d_force_y




sphere2red_path = dp.find_file("sphere2red.urdf")
print("sphere2red_path=",sphere2red_path)

render = True
if render:
  app = p.TinyOpenGL3App("billiard_opt_example_gui")
  app.renderer.init()
  cam = p.TinyCamera()
  cam.set_camera_distance(4.)
  cam.set_camera_pitch(-30)
  app.renderer.set_camera(cam)


init_force_x = dp.fraction(0,1)
init_force_y = dp.fraction(500,1)
steps = 300
#do a rollout before optimization
rollout(init_force_x, init_force_y, steps, render)

learning_rate = dp.fraction(100,1)
force_x = init_force_x
force_y = init_force_y
#gradient descent using gradients
for iter in range (50):
      if use_auto_diff:
        cost, d_force_x, d_force_y = grad_dual(force_x, force_y, steps)
      else:
        cost, d_force_x, d_force_y = grad_finite(force_x, force_y, steps)
      
      print("Iteration:", iter, " cost:", cost,"force_x:", force_x, "force_y:",force_y)
      force_x -= learning_rate * d_force_x
      force_y -= learning_rate * d_force_y

#do a rollout after optimization
rollout(force_x, force_y, steps, render)


