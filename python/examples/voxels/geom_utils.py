import numpy as np
np.set_printoptions(suppress=True)

def createSphereMultiBody(p, radius=1, worldPosition=[0,0,0]):
  collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=radius)
  
  body = p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId
                      )
  p.resetBasePositionAndOrientation(body,worldPosition,[0,0,0,1])
  return body

#worldOrientation in Euler roll/pitch/yaw
def createBoxMultiBody(p, halfExtents=[1,1,1], worldPosition=[0,0,0], worldOrientation=[0,0,0]):
  collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents)
  body = p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId)
  orn = p.getQuaternionFromEuler(worldOrientation)
  p.resetBasePositionAndOrientation(body,worldPosition,[0,0,0,1])
  return body



def createMultiBody(p, objFileName, worldPosition=[0,0,0],mass=0, isConcave=False, meshScale=[1,1,1]):
  flags = 0
  if isConcave:
    flags+=p.GEOM_FORCE_CONCAVE_TRIMESH
  collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName=objFileName,
                                          flags=flags,
                                          meshScale=meshScale
                                          )
  body = p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId)
  p.resetBasePositionAndOrientation(body,worldPosition,[0,0,0,1])
  return body

                                          


def drawAABB(p, aabb):
  aabbMin = aabb[0]
  aabbMax = aabb[1]
  f = [aabbMin[0], aabbMin[1], aabbMin[2]]
  t = [aabbMax[0], aabbMin[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 0, 0])
  f = [aabbMin[0], aabbMin[1], aabbMin[2]]
  t = [aabbMin[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [0, 1, 0])
  f = [aabbMin[0], aabbMin[1], aabbMin[2]]
  t = [aabbMin[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [0, 0, 1])

  f = [aabbMin[0], aabbMin[1], aabbMax[2]]
  t = [aabbMin[0], aabbMax[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])

  f = [aabbMin[0], aabbMin[1], aabbMax[2]]
  t = [aabbMax[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])

  f = [aabbMax[0], aabbMin[1], aabbMin[2]]
  t = [aabbMax[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])

  f = [aabbMax[0], aabbMin[1], aabbMin[2]]
  t = [aabbMax[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])

  f = [aabbMax[0], aabbMax[1], aabbMin[2]]
  t = [aabbMin[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])

  f = [aabbMin[0], aabbMax[1], aabbMin[2]]
  t = [aabbMin[0], aabbMax[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])

  f = [aabbMax[0], aabbMax[1], aabbMax[2]]
  t = [aabbMin[0], aabbMax[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
  f = [aabbMax[0], aabbMax[1], aabbMax[2]]
  t = [aabbMax[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])
  f = [aabbMax[0], aabbMax[1], aabbMax[2]]
  t = [aabbMax[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 1, 1])


vertices = [[-1.000000, -1.000000, 1.000000], [1.000000, -1.000000, 1.000000],
            [1.000000, 1.000000, 1.000000], [-1.000000, 1.000000, 1.000000],
            [-1.000000, -1.000000, -1.000000], [1.000000, -1.000000, -1.000000],
            [1.000000, 1.000000, -1.000000], [-1.000000, 1.000000, -1.000000],
            [-1.000000, -1.000000, -1.000000], [-1.000000, 1.000000, -1.000000],
            [-1.000000, 1.000000, 1.000000], [-1.000000, -1.000000, 1.000000],
            [1.000000, -1.000000, -1.000000], [1.000000, 1.000000, -1.000000],
            [1.000000, 1.000000, 1.000000], [1.000000, -1.000000, 1.000000],
            [-1.000000, -1.000000, -1.000000], [-1.000000, -1.000000, 1.000000],
            [1.000000, -1.000000, 1.000000], [1.000000, -1.000000, -1.000000],
            [-1.000000, 1.000000, -1.000000], [-1.000000, 1.000000, 1.000000],
            [1.000000, 1.000000, 1.000000], [1.000000, 1.000000, -1.000000]]

normals = [[0.000000, 0.000000, 1.000000], [0.000000, 0.000000, 1.000000],
           [0.000000, 0.000000, 1.000000], [0.000000, 0.000000, 1.000000],
           [0.000000, 0.000000, -1.000000], [0.000000, 0.000000, -1.000000],
           [0.000000, 0.000000, -1.000000], [0.000000, 0.000000, -1.000000],
           [-1.000000, 0.000000, 0.000000], [-1.000000, 0.000000, 0.000000],
           [-1.000000, 0.000000, 0.000000], [-1.000000, 0.000000, 0.000000],
           [1.000000, 0.000000, 0.000000], [1.000000, 0.000000, 0.000000],
           [1.000000, 0.000000, 0.000000], [1.000000, 0.000000, 0.000000],
           [0.000000, -1.000000, 0.000000], [0.000000, -1.000000, 0.000000],
           [0.000000, -1.000000, 0.000000], [0.000000, -1.000000, 0.000000],
           [0.000000, 1.000000, 0.000000], [0.000000, 1.000000, 0.000000],
           [0.000000, 1.000000, 0.000000], [0.000000, 1.000000, 0.000000]]

uvs = [[0.750000, 0.250000], [1.000000, 0.250000], [1.000000, 0.000000], [0.750000, 0.000000],
       [0.500000, 0.250000], [0.250000, 0.250000], [0.250000, 0.000000], [0.500000, 0.000000],
       [0.500000, 0.000000], [0.750000, 0.000000], [0.750000, 0.250000], [0.500000, 0.250000],
       [0.250000, 0.500000], [0.250000, 0.250000], [0.000000, 0.250000], [0.000000, 0.500000],
       [0.250000, 0.500000], [0.250000, 0.250000], [0.500000, 0.250000], [0.500000, 0.500000],
       [0.000000, 0.000000], [0.000000, 0.250000], [0.250000, 0.250000], [0.250000, 0.000000]]
indices = [0,1, 2, 0, 2, 3,  #//ground face
    6, 5, 4, 7, 6, 4,  #//top face
    10, 9,8,11,10,8, 12, 13, 14, 12, 14, 15, 18,
    17,16,19, 18,16, 20, 21, 22, 20, 22, 23]
