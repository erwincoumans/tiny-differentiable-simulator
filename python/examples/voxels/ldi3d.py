import numpy as np
import math
import time
import geom_utils

dtype = [('body_uid', 'int'), ('hit_fraction', float)]
renderLines = True

class LDI(object):
  def __init__(self, target_uid, bullet_client):
    self._target_uid = target_uid
    self._p = bullet_client
    self._numThreads = 6 #leave to zero to use maximum number of threads, -1 for single-threaded, positive for specific number of threads
    self._targetGroup=8
    self._targetMask=(-1)^8
    self._primitiveGroup = 4
    self._primitiveMask = -1^4
    self._ignoreGroup = 1
    self._ignoreMask = -1
    
    self._p.setCollisionFilterGroupMask(self._target_uid,-1, self._targetGroup, self._targetMask)
    self._p.changeVisualShape(self._target_uid,-1,rgbaColor=[1,.2,.2,0.3])
    self._max_hits_per_ray = 32
    self._target_aabb = np.array(self._p.getAABB(self._target_uid,-1))
    ind_min = np.argmin(self._target_aabb[0])
    ind_max = np.argmax(self._target_aabb[1])
    for i in range (3):
      self._target_aabb[0][i] = self._target_aabb[0][ind_min]
      self._target_aabb[1][i] = self._target_aabb[1][ind_max]
      

      
    aabb_min=np.array(self._target_aabb[0])
    aabb_max=np.array(self._target_aabb[1])

    #object should not go outside of this aabb!
    self._aabb_margin=0.1
    self._global_aabb = np.array([[aabb_min[0]-self._aabb_margin,aabb_min[1]-self._aabb_margin,aabb_min[2]-self._aabb_margin],
                        [aabb_max[0]+self._aabb_margin,aabb_max[1]+self._aabb_margin,aabb_max[2]+self._aabb_margin]])
                        
    self._rayUnionHitColor= [0, 0, 1]
    self._rayTargetHitColor = [1, 0, 0]
    self._rayPrimitiveHitColor = [1, 1, 0]
    self._rayMissColor = [0, 1, 0]

    #print("enlarged aabb=", self._global_aabb)
    
    self._numRays=0
    self._rayFrom=[]
    self._rayTo=[]
    self._rayIds=[]
    self._aabbRaySubdivision = 32 #number of rays along one axis
    
    aabbMin = self._global_aabb[0]
    aabbMax = self._global_aabb[1]
    
    
    geom_utils.drawAABB(self._p, self._global_aabb)
    
    #create all rays
    for xx in range (self._aabbRaySubdivision):
      for yy in range (self._aabbRaySubdivision):
        x = aabbMin[0]+(xx+0.5)/self._aabbRaySubdivision*(aabbMax[0]-aabbMin[0])
        y = aabbMin[1]+(yy+0.5)/self._aabbRaySubdivision*(aabbMax[1]-aabbMin[1])
        #x = 0.5*(aabbMin[0]+aabbMax[0])
        #y = 0.5*(aabbMin[1]+aabbMax[1])
        f = [x,y, aabbMin[2]]
        t = [x,y, aabbMax[2]]
        self._rayFrom.append(f)
        self._rayTo.append(t)
        self._numRays+=1
        #if renderLines:
        #      self._rayIds.append(self._p.addUserDebugLine(f, t, self._rayMissColor))
    
    self._targetRayResults=[]
    for ray in range (self._numRays):
      self._targetRayResults.append([])

    if 1:
      for layer in range(self._max_hits_per_ray):
        has_hit = 0
        
        self._target_hit_results = None
        #rayTestBatch has a limitation on max rays, use very slow 'single' for testing
        single=False
        
        if single:
          print("single ray, wait...")
          for ray in range(self._numRays):
            rayf = self._rayFrom[ray]
            rayt = self._rayTo[ray]
            res = self._p.rayTest(rayf, rayt, reportHitNumber=layer)
            if self._target_hit_results == None:
              self._target_hit_results = res
            else:
              self._target_hit_results=self._target_hit_results+res
          print("done")
        else:
          self._target_hit_results = self.batchRay(self._rayFrom, self._rayTo, numThreads=self._numThreads, collisionFilterMask=self._targetGroup, reportHitNumber=layer)
        
        for ray in range(self._numRays):
            hitObjectUid = self._target_hit_results[ray][0]
            if (hitObjectUid >= 0):
              has_hit = 1
              hitPosition = self._target_hit_results[ray][3]
              hitFraction = self._target_hit_results[ray][2]
              self._targetRayResults[ray].append((hitObjectUid,hitFraction))
        if (not has_hit):
          #print("maximum number of hits (layers) per ray for target=", layer)
          break
          
        
    #print("sorting target rays...")
    self._target_volume = 0
    #each ray can contribute volume(aabb)/(self._numRays) to volume
    aabbWidth=aabbMax[0]-aabbMin[0]
    aabbLenght=aabbMax[1]-aabbMin[1]
    aabbHeight=aabbMax[2]-aabbMin[2]
    self._volume_aabb = aabbWidth*aabbLenght*aabbHeight
    self._volume_ray = self._volume_aabb/self._numRays
    
    self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING,0)

    #############################
    #for target concave triangle mesh
    self._sorted_rays_target=[]
    for ray in range (self._numRays):
      #print("====================================")
      #print("ray_index=",ray)
      one_ray_target = np.array(self._targetRayResults[ray],dtype=dtype)
      #print("one_ray=",one_ray)
      sorted_ray_target = np.sort(one_ray_target, order='hit_fraction')
      self._sorted_rays_target.append(sorted_ray_target)
      inside_target=0
      
      np_from = np.array(self._rayFrom[ray])
      np_to = np.array(self._rayTo[ray])
      prev_fraction = 0
      for hit in sorted_ray_target:
        fraction = hit["hit_fraction"]
        #print("fraction=",fraction)
        
        pt = np.array(np_from + fraction*(np_to-np_from))
        #print("pt=",pt)
        if (inside_target):
          t = pt
          self._p.addUserDebugLine(f, t, self._rayTargetHitColor)
          self._target_volume += self._volume_ray*(fraction-prev_fraction)
          #print("adding line (",f,") to (",t,")")
          inside_target=inside_target-1
        else:
          f = pt
          prev_fraction = fraction
          inside_target=inside_target+1
    #print("done sorting target rays.")   
    self._p.configureDebugVisualizer(self._p.COV_ENABLE_RENDERING,1)
    
  def valid_primitives_aabb(self, p, primitives):
    #aabb of all bodies
    if len(primitives)==0:
      return False
    
    for body_uid in primitives:
      body_aabb = p.getAABB(body_uid,-1)
      prim_aabb_min=np.array(body_aabb[0])
      prim_aabb_max=np.array(body_aabb[1])
      self._global_aabb
      aabb_min = np.minimum(prim_aabb_min, self._global_aabb[0])
      aabb_max = np.maximum(prim_aabb_max, self._global_aabb[1])
      if not (aabb_min==self._global_aabb[0]).all():
        return False
      if not (aabb_max==self._global_aabb[1]).all():
        return False
      return True
   

  def eval(self, primitives, visualize_intersection=False, visualize_primitive_volume=False):
    #lots of further possible optimizations: 
    #for example cache ray hit results and only recompute the rays that overlap with aabb of primitives changed etc.
    
    #clear all objects, except self._target_uid
    for b in range(self._p.getNumBodies()):
      body_uid = self._p.getBodyUniqueId(b)
      if body_uid != self._target_uid:
        self._p.setCollisionFilterGroupMask(body_uid,-1, self._ignoreGroup, self._ignoreMask)
      
    
    for prim_uid in primitives:
      self._p.setCollisionFilterGroupMask(prim_uid,-1, self._primitiveGroup, self._primitiveMask)
      self._p.changeVisualShape(prim_uid,-1,rgbaColor=[1,1,1,0.6])

    primitiveRayResults=[]
    for ray in range (self._numRays):
      primitiveRayResults.append([])

    for layer in range(self._max_hits_per_ray):#could use 2
          has_hit=0
          results = self.batchRay(self._rayFrom, self._rayTo, numThreads=self._numThreads, collisionFilterMask=self._primitiveGroup, reportHitNumber=layer)
          for ray in range(self._numRays):
              hitObjectUid = results[ray][0]
              if (hitObjectUid >= 0):
                has_hit=1
                hitPosition = results[ray][3]
                hitFraction = results[ray][2]
                primitiveRayResults[ray].append((hitObjectUid,hitFraction))
          if (not has_hit):
            #print("total used layers for primitives=", layer)
            break
    for layer in range(self._max_hits_per_ray):#could use 2
          has_hit=0
          results = self.batchRay(self._rayTo, self._rayFrom, numThreads=self._numThreads, collisionFilterMask=self._primitiveGroup, reportHitNumber=layer)
          for ray in range(self._numRays):
              hitObjectUid = results[ray][0]
              if (hitObjectUid >= 0):
                has_hit=1
                hitPosition = results[ray][3]
                hitFraction = results[ray][2]
                primitiveRayResults[ray].append((hitObjectUid,1-hitFraction))
          if (not has_hit):
            #print("total used layers for primitives=", layer)
            break
    #print("layer=",layer)
    #############################
    #for primitives
    sorted_rays_prim=[]
    primitive_volume = 0

    for ray in range (self._numRays):
      #print("====================================")
      #print("ray_index=",ray)
      one_ray_prim = np.array(primitiveRayResults[ray],dtype=dtype)
      #print("one_ray=",one_ray)
      sorted_ray_prim = np.sort(one_ray_prim, order='hit_fraction')
      sorted_rays_prim.append(sorted_ray_prim)
      inside_primitives=0
      
      np_from = np.array(self._rayFrom[ray])
      np_to = np.array(self._rayTo[ray])
      prev_fraction = 0
      inside_primitive_array=[]
      for hit in sorted_ray_prim:
        fraction = hit["hit_fraction"]
        prim_uid = hit["body_uid"]
        
        #print("fraction=",fraction)
        
        pt = np.array(np_from + fraction*(np_to-np_from))
        #print("pt=",pt)
        #enter or leave an new or existing primitive?
        if prim_uid in inside_primitive_array:
          #we must be leaving this primitive
          inside_primitive_array.remove(prim_uid)
          inside_primitives=inside_primitives-1
          t_prim = pt
          if inside_primitives==0:
            if visualize_primitive_volume:
              self._p.addUserDebugLine(f_prim, t_prim, self._rayPrimitiveHitColor)
            primitive_volume += self._volume_ray*(fraction-prev_fraction)
        else:
          inside_primitive_array.append(prim_uid)
          
          if inside_primitives==0:
            f_prim = pt
            prev_fraction = fraction
          inside_primitives=inside_primitives+1
          
        
    #print("inside_primitive_array=",inside_primitive_array)
    #print("primitive_volume=",primitive_volume)
        
    ################################################
    # intersection, both target and primitives
    intersection_volume = 0
    len_sorted_rays_prim = len(sorted_rays_prim)
    len_sorted_rays_target = len(self._sorted_rays_target)
    #print("len(sorted_rays_prim)=",len_sorted_rays_prim)
    #print("len(sorted_rays_target)=",len_sorted_rays_target)
    #print("numRays=",self._numRays)

    if 1:
      for ray in range (self._numRays):
        sorted_ray_prim = sorted_rays_prim[ray]
        sorted_ray_target = self._sorted_rays_target[ray]
        
        np_from = np.array(self._rayFrom[ray])
        np_to = np.array(self._rayTo[ray])
        
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
          
          while (prim_hit_index < len_sorted_ray_prim) and (target_hit_index<len_sorted_ray_target):
              #print("target_hit_index=",target_hit_index)
              #print("prim_hit_index=",prim_hit_index)
              
              #progress
              next_target_hit = sorted_ray_target[target_hit_index]
              next_prim_hit = sorted_ray_prim[prim_hit_index]
              
              prim_fraction = next_prim_hit["hit_fraction"]
              target_fraction = next_target_hit["hit_fraction"]
              prim_uid = next_prim_hit["body_uid"]
              #print("prim_fraction=",prim_fraction)
              #print("target_fraction=",target_fraction)
              
              #pick next smallest fraction from either prim or target hits
              if (prim_fraction<=target_fraction):
                cur_prim_fraction = prim_fraction
                pt = np.array(np_from + prim_fraction*(np_to-np_from))
                #print("pt prim=",pt)
                #are we entering a new or existing primitive or leaving a existing primitive?
                if prim_uid in inside_primitive_array:
                  #we must be leaving this primitive
                  inside_primitive_array.remove(prim_uid)
                  inside_primitives=inside_primitives-1
                  
                  t_prim = pt
                  if inside_target and inside_primitives==0:
                    if visualize_intersection:
                      self._p.addUserDebugLine(from_target_prim, t_prim, self._rayUnionHitColor)
                    intersection_volume += self._volume_ray*(prim_fraction-prev_fraction)
                    #print("adding line (",f,") to (",t,")")
                  
                else:
                  #we must be entering this primitive
                  if len(inside_primitive_array)==0:
                    from_target_prim = pt
                    prev_fraction = prim_fraction
                  inside_primitive_array.append(prim_uid)
                  inside_primitives=inside_primitives+1
                prim_hit_index+=1
              else:
                #entering or leaving target?
                pt = np.array(np_from + target_fraction*(np_to-np_from))
                if (inside_target):
                  t_target = pt
                  if inside_primitives>0:
                    if visualize_intersection:
                      self._p.addUserDebugLine(from_target_prim, t_target, self._rayUnionHitColor)
                    intersection_volume += self._volume_ray*(target_fraction-prev_fraction)
                    #print("adding line (",f,") to (",t,")")
                  inside_target=inside_target-1
                else:
                  #we are entering the target
                  from_target_prim = pt
                  prev_fraction = target_fraction
                  inside_target=inside_target+1
                target_hit_index+=1
              
    #print("intersection_volume=",intersection_volume)
    return primitive_volume,intersection_volume
  def batchRay(self, rayFromPositions, rayToPositions,numThreads,reportHitNumber, collisionFilterMask):
    segment_rays = True
    if (segment_rays):
      seg_length = 1000
      segFrom = [rayFromPositions[x:x+seg_length] for x in range(0,len(rayFromPositions),seg_length)]
      segTo = [rayToPositions[x:x+seg_length] for x in range(0,len(rayToPositions),seg_length)]  
      all_results = []
      for i in range (len(segFrom)):
        rf = segFrom[i]
        rt = segTo[i]
        res = self._p.rayTestBatch(rayFromPositions=rf, rayToPositions=rt, numThreads=numThreads, reportHitNumber=reportHitNumber, collisionFilterMask=collisionFilterMask)
        all_results.extend(res) 
      return all_results
    else:
      results = self._p.rayTestBatch(rayFromPositions=rayFromPositions, rayToPositions=rayToPositions, numThreads=numThreads, reportHitNumber=reportHitNumber, collisionFilterMask=collisionFilterMask)
      return results



