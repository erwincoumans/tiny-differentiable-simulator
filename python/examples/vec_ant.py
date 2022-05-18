import pytinydiffsim as pd
import numpy as np

num_envs = 16
ant = pd.VectorizedAntEnv(num_envs)
vecobs = ant.reset()
print("len(vecobs)=",len(vecobs))
print("vecobs=",vecobs)

print("ant.action_dim()=",ant.action_dim())
print("ant.obs_dim()=",ant.obs_dim())

actions = [[0] * ant.action_dim()]*num_envs
ant_res = ant.step(actions)
print("ant_res.obs=",ant_res.obs)
print("ant_res.rewards=",ant_res.rewards)
print("ant_res.dones=",ant_res.dones)
