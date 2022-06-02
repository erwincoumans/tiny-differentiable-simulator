import pytinydiffsim as pd
import numpy as np
import pybullet as p
import time

p.connect(p.DIRECT)
timestr = time.strftime("%Y%m%d-%H%M%S")
envname="tds_vec_laikago"
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "sb3_pb_"+envname+timestr+"_tracing.json")
print("step 0")

import time

num_envs = 10
substeps = 1000
auto_reset_when_done = True
agents = pd.VectorizedLaikagoEnv(num_envs, auto_reset_when_done)
print("step 1")
vecobs = agents.reset()
print("step 2")
#print("len(vecobs)=",len(vecobs))
#print("vecobs=",vecobs)

#print("agents.action_dim()=",agents.action_dim())
#print("agents.obs_dim()=",agents.obs_dim())

#actions = [[0] * agents.action_dim()]*num_envs
actions = [		[9.51359177	,	-16.0218258	,	-0.605172634	,	-4.29735327	,	-1.41641760	,	-1.21251166	,	-5.73428822	,	12.2695637	,	2.85404038	,	6.03182983	,	2.48567367	,	1.22785664]]* num_envs


t0 = time.time()
p.submitProfileTiming("agents_step_"+str(num_envs))
for i in range (substeps):
  p.submitProfileTiming("substep")
  agents_res = agents.step(actions)
  p.submitProfileTiming()
  
p.submitProfileTiming()
t1 = time.time()
dt = t1-t0
print("dt=",dt)
print("steps/sec:",num_envs*substeps/dt)

#print("agents_res.obs=",agents_res.obs)
#print("agents_res.rewards=",agents_res.rewards)
#print("agents_res.dones=",agents_res.dones)

p.stopStateLogging(logId)
