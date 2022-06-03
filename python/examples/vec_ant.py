import pytinydiffsim as pd
import numpy as np
import pybullet as p
import time

p.connect(p.DIRECT)
timestr = time.strftime("%Y%m%d-%H%M%S")
envname="tds_vec_ant"
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "sb3_pb_"+envname+timestr+"_tracing.json")
print("step 0")

import time

num_envs = 100
substeps = 100
auto_reset_when_done = True
ant = pd.VectorizedAntEnv(num_envs, auto_reset_when_done)
print("step 1")
vecobs = ant.reset()
print("step 2")
#print("len(vecobs)=",len(vecobs))
#print("vecobs=",vecobs)

#print("ant.action_dim()=",ant.action_dim())
#print("ant.obs_dim()=",ant.obs_dim())

actions = [[0] * ant.action_dim()]*num_envs

t0 = time.time()
p.submitProfileTiming("ant_step_"+str(num_envs))
for i in range (substeps):
  p.submitProfileTiming("substep")
  ant_res = ant.step(actions)
  p.submitProfileTiming()
  
p.submitProfileTiming()
t1 = time.time()
dt = t1-t0
print("dt=",dt)
print("steps/sec:",num_envs*substeps/dt)

#print("ant_res.obs=",ant_res.obs)
#print("ant_res.rewards=",ant_res.rewards)
#print("ant_res.dones=",ant_res.dones)

p.stopStateLogging(logId)
