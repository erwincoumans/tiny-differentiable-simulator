import numpy as np
import time
import pybullet
import ldi3d
import pybullet_utils.bullet_client as bc
import geom_utils
import math
import argparse
#import eval_proxies
import random
#import eval_cpp

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--obj_file', help='Wavefront obj file as target', type=str, default='teddy.obj')#cube.obj')#sphere_smooth.obj')#bunny.obj')
parser.add_argument('--json_file', help='Primitive Decomposition', type=str, default='')
args = parser.parse_args()


p = bc.BulletClient(pybullet.DIRECT)
p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)

np.set_printoptions(precision=5)
np.set_printoptions(suppress=True)

timingLog = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "ldi3d_timings.json")

prevSave = 0
prevSeed = 0


#load the obj wavefront name
filename = args.obj_file

flags=p.URDF_INITIALIZE_SAT_FEATURES
target_uid = geom_utils.createMultiBody(p,filename, [0,0,0], isConcave=True, meshScale=[1,1,1])
p.resetBasePositionAndOrientation(target_uid,[0,0,0],p.getQuaternionFromEuler([math.pi/2,0,0]))
#alternative, load a URDF file (which points to one or more Wavefront obj files and adds some more info)
#target_uid = p.loadURDF("unit_sphere.urdf", flags=flags)
#target_uid = p.loadURDF("ubox.urdf", flags=flags, globalScaling=0.1)

ldi = ldi3d.LDI(target_uid=target_uid, bullet_client=p)
print("target volume =", ldi._target_volume)


import pytinyopengl3 as p
import math, time

app = p.TinyOpenGL3App("ldi")
app.renderer.init()
cam = p.TinyCamera()
cam.set_camera_distance(2.)
cam.set_camera_pitch(-20)
app.renderer.set_camera(cam)

pos = p.TinyVector3f(0.,0.,0.)
orn = p.TinyQuaternionf(0.,0.,0.,1.)
orn.set_euler_rpy(p.TinyVector3f(math.pi/2.0,0.,0.))

color = p.TinyVector3f(1.,1.,1.)
bun_color = p.TinyVector3f(1.,0.,0.)


bun_scaling = p.TinyVector3f(1,1,1)

opacity = 1

app.renderer.write_transforms()


bunny_file = "../../../data/bunny.obj"
bun_shapes = p.load_obj_shapes(app, bunny_file, pos, orn, bun_scaling)
bun_mesh = app.renderer.register_graphics_instance(bun_shapes[0], pos, orn, bun_color, bun_scaling, 0.)

cube_file = "../../../data/cube.obj"



aabbMin = ldi._global_aabb[0]
aabbMax = ldi._global_aabb[1]
szx = aabbMax[0]-aabbMin[0]
szy = aabbMax[1]-aabbMin[1]
szz = aabbMax[2]-aabbMin[2]

scaling = p.TinyVector3f(szx/ldi._aabbRaySubdivision,szy/ldi._aabbRaySubdivision,szz/ldi._aabbRaySubdivision)#.1,0.1,0.1)
print("scaling=",scaling)
shapes = p.load_obj_shapes(app, cube_file, pos, orn, scaling)
cube_shape = shapes[0]


def check_overlap(ray_id, from_frac, to_frac):
  #print("from_frac=",from_frac)
  #print("to_frac=",to_frac)
  
  prim_fractions = [from_frac, to_frac]
  hits = False
  
  #sorted_ray_prim = sorted_rays_prim[ray]
  sorted_ray_prim = [0,0]
  
  sorted_ray_target = ldi._sorted_rays_target[ray]
  
  np_from = np.array(ldi._rayFrom[ray])
  np_to = np.array(ldi._rayTo[ray])
  
  #few different cases, want to unify those cases
  len_sorted_ray_prim = len(sorted_ray_prim)
  len_sorted_ray_target = len(sorted_ray_target)
  if (len_sorted_ray_prim>0 and len_sorted_ray_target>0):
    
    target_hit_index = 0
    prim_hit_index = 0
    cur_overlapping_primitives=[]
    inside_primitives=0
    inside_target=0

    cur_target_fraction = 0
    cur_prim_fraction = 0
        
    inside_primitive_array=[]
    box_id = 0
    while (prim_hit_index < len_sorted_ray_prim) and (target_hit_index<len_sorted_ray_target):
        #progress
        next_target_hit = sorted_ray_target[target_hit_index]
        prim_fraction = prim_fractions[prim_hit_index]
        target_fraction = next_target_hit["hit_fraction"]
        prim_uid = box_id
        #print("prim_fraction=",prim_fraction)
        #print("target_fraction=",target_fraction)
        
        #pick next smallest fraction from either prim or target hits
        if (prim_fraction<=target_fraction):
          cur_prim_fraction = prim_fraction
          #print("pt prim=",pt)
          #are we entering a new or existing primitive or leaving a existing primitive?
          if prim_uid in inside_primitive_array:
            #we must be leaving this primitive
            inside_primitive_array.remove(prim_uid)
            inside_primitives=inside_primitives-1
            if inside_target and inside_primitives==0:
              hits = True
          else:
            #we must be entering this primitive
            if len(inside_primitive_array)==0:
              prev_fraction = prim_fraction
            inside_primitive_array.append(prim_uid)
            inside_primitives=inside_primitives+1
          prim_hit_index+=1
        else:
          #entering or leaving target?
          if (inside_target):
            if inside_primitives>0:
              hits = True
            inside_target=inside_target-1
          else:
            #we are entering the target
            prev_fraction = target_fraction
            inside_target=inside_target+1
          target_hit_index+=1
  return hits
  
  
occupancy_grid = np.zeros((ldi._aabbRaySubdivision, ldi._aabbRaySubdivision, ldi._aabbRaySubdivision))
ray = 0
total_voxels = 0
for xx in range (ldi._aabbRaySubdivision):
    for yy in range (ldi._aabbRaySubdivision):
      for zz in range (ldi._aabbRaySubdivision):
        x = aabbMin[0]+(xx+0.5)/ldi._aabbRaySubdivision*(aabbMax[0]-aabbMin[0])
        y = aabbMin[1]+(yy+0.5)/ldi._aabbRaySubdivision*(aabbMax[1]-aabbMin[1])
        z = aabbMin[2]+(zz+0.5)/ldi._aabbRaySubdivision*(aabbMax[2]-aabbMin[2])
        from_frac = (zz)/ldi._aabbRaySubdivision
        to_frac = (zz+1)/ldi._aabbRaySubdivision
        if (check_overlap(ray, from_frac, to_frac)):
          total_voxels += 1
          occupancy_grid[xx][yy][zz] = 1
        #print("pos=",pos)
      ray += 1
print("total_voxels=",total_voxels)

def get_voxel_type(grid, x,y,z,dim):
  fully_inside = True
  border = False
  if x==0 or y==0 or z==0 or x>=(dim-1) or y>=(dim-1) or z>= (dim-1):
    fully_inside = False
    border=True
  else:
    fully_inside = grid[x-1][y][z] and grid[x][y-1][z] and grid[x][y][z-1] and grid[x+1][y][z] and grid[x][y+1][z] and grid[x][y][z+1]

  is_face = False
  is_edge = False
  if not border:
    is_face = (grid[x-1][y][z] and grid[x+1][y][z] and grid[x][y-1][z] and grid[x][y+1][z]) or \
      (grid[x-1][y][z] and grid[x+1][y][z] and grid[x][y][z-1] and grid[x][y][z+1]) or \
      (grid[x][y-1][z] and grid[x][y+1][z] and grid[x][y][z-1] and grid[x][y][z+1])
    if not is_face:
      is_edge = (grid[x-1][y][z] and grid[x+1][y][z]) or \
              (grid[x][y-1][z] and grid[x][y+1][z]) or \
              (grid[x][y][z-1] and grid[x][y][z+1])
  return fully_inside, is_face, is_edge
  
actual_voxels = 0
face = 0
edge = 0
for xx in range (ldi._aabbRaySubdivision):
    for yy in range (ldi._aabbRaySubdivision):
      for zz in range (ldi._aabbRaySubdivision):
        x = aabbMin[0]+(xx+0.5)/ldi._aabbRaySubdivision*(aabbMax[0]-aabbMin[0])
        y = aabbMin[1]+(yy+0.5)/ldi._aabbRaySubdivision*(aabbMax[1]-aabbMin[1])
        z = aabbMin[2]+(zz+0.5)/ldi._aabbRaySubdivision*(aabbMax[2]-aabbMin[2])
        from_frac = (zz)/ldi._aabbRaySubdivision
        to_frac = (zz+1)/ldi._aabbRaySubdivision
        if occupancy_grid[xx][yy][zz]:
          fully_inside, is_face, is_edge = get_voxel_type(occupancy_grid, xx, yy, zz, ldi._aabbRaySubdivision)
          if not fully_inside:
            pos = p.TinyVector3f(x,y,z)
            if is_face:
              voxel_color = p.TinyVector3f(0,1,0)
              face += 1
            else:
              if is_edge:
                voxel_color = p.TinyVector3f(0,0,1)
                edge+=1
              else:
                voxel_color = p.TinyVector3f(1,1,1)
              
            mesh = app.renderer.register_graphics_instance(cube_shape, pos, orn, voxel_color, scaling, opacity)
            actual_voxels += 1
        #print("pos=",pos)

print("actual_voxels = ",actual_voxels)
print("face-voxel = ", face)
print("edge-voxel = ", edge)
print("vertex-voxel = ", actual_voxels-face-edge)
app.renderer.write_transforms()

while not app.window.requested_exit():
  app.renderer.update_camera(2)
  dg = p.DrawGridData()
  dg.drawAxis = True
  app.draw_grid(dg)
  from_line=p.TinyVector3f(0,0,0)
  to_line=p.TinyVector3f(1,1,1)
  color=p.TinyVector3f(1,1,1)
  width=2
  #app.renderer.draw_line(from_line,to_line,color,width)
  #pos = p.TinyVector3f(0,0,math.sin(time.time())+1)
  #app.renderer.write_single_instance_color_to_cpu(color, opacity, mesh)
  #app.renderer.write_single_instance_transform_to_cpu(pos, orn,mesh)
  
  
  app.renderer.render_scene()
  app.draw_text_3d("hi",1,1,1,1)
  app.swap_buffer()

