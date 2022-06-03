import pytinydiffsim as tds
num_actors = 512
auto_reset_when_done = True
tds_robot = tds.VectorizedLaikagoEnv(num_actors, auto_reset_when_done)
tds_robot.reset()
actions = [[0] * tds_robot.action_dim()]*num_actors
res = tds_robot.step(actions)


import pytinyopengl3 as g
viz = g.OpenGLUrdfVisualizer()
urdf = g.OpenGLUrdfStructures()
parser = g.UrdfParser()
file_name = tds_robot.urdf_filename()
urdf = parser.load_urdf(file_name)
print("urdf=",urdf)
texture_path = "laikago_tex.jpg"
viz.path_prefix = g.extract_path(file_name)
print("viz.path_prefix=",viz.path_prefix)
viz.convert_visuals(urdf, texture_path)
print("create_instances")
all_instances = viz.create_instances(urdf, texture_path, num_actors)

#sync transforms
#for pairs in all_instances:
#  for pair in pairs:
#    print("pair.link=", pair.link_index, " pair.visual_instance=", pair.visual_instance)
sim_spacing = 2

print("len(all_instances)=",len(all_instances))
print("\nhold CTRL and right/left/middle mouse button to rotate/zoom/move camera")
while 1:
  res = tds_robot.step(actions)
  viz.sync_visual_transforms(all_instances, res.visual_world_transforms, tds_robot.obs_dim(), sim_spacing)
  viz.render()
          
          
          