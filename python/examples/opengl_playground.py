import pytinyopengl3 as p
import pytinydiffsim as pd
import opengl_utils_pd
import os
import numpy as np


# CAMERA SETUP
app = p.TinyOpenGL3App("title")
app.renderer.init()
cam = p.TinyCamera()
cam.set_camera_distance(2.)
cam.set_camera_pitch(-20)
cam.set_camera_target_position(0, 0, 0)
cam.set_camera_yaw(0)
app.renderer.set_camera(cam)
app.renderer.write_transforms()

world = pd.TinyWorld()
dispatcher = world.get_collision_dispatcher()

# LOAD THE URDF DATA

models_dir = "../../data/"
urdf_parser = pd.TinyUrdfParser()

laikago_urdf_data = urdf_parser.load_urdf(models_dir +
    "laikago/laikago_toes_zup_chassis_collision.urdf")

plane_urdf_data = urdf_parser.load_urdf(models_dir + "plane_implicit.urdf")


# CREATE VISUAL STRUCTURES

robot_vis = opengl_utils_pd.convert_visuals(urdf=laikago_urdf_data, app=app,
    path_prefix="../../data/laikago/")
plane_vis = opengl_utils_pd.convert_visuals(urdf=plane_urdf_data, app=app,
    path_prefix="../../data/")
urdf2mb = pd.UrdfToMultiBody2()


# CREATE PHYSICS MULTI-BODIES

plane_mb = pd.TinyMultiBody(False)
res = urdf2mb.convert2(plane_urdf_data, world, plane_mb)

is_floating = True
mb = pd.TinyMultiBody(is_floating)
res = urdf2mb.convert2(laikago_urdf_data, world, mb)

mb.set_position(pd.Vector3(0, 0, 3))

solver = pd.TinyMultiBodyConstraintSolver()

pd.forward_kinematics(mb, mb.q, mb.qd)
print(mb.q)

app.renderer.write_transforms()

dt = 1. / 1000.

while not app.window.requested_exit():
    # this is the magic line -- no idea what it does
    app.renderer.update_camera(2)
    dg = p.DrawGridData()
    dg.drawAxis = True
    app.draw_grid(dg)
    # pd.forward_kinematics(mb, mb.q, mb.qd)

    if app.get_sim_reset_flag() == True:
        mb.initialize()

        app.renderer.write_transforms()
        mb.set_position(pd.Vector3(0, 0, 3))

        pd.forward_kinematics(mb, mb.q, mb.qd)
        opengl_utils_pd.sync_visual_transforms(mb, robot_vis, app)

        app.renderer.write_transforms()
        app.set_sim_reset_flag(False)

    if app.get_sim_state() == False and app.get_sim_reset_flag() == False:
        # add some gravity
        pd.forward_dynamics(mb, pd.Vector3(0., 0., -10.))

        # compute contacts between floor and laikago
        multi_bodies = [plane_mb, mb]
        contacts = world.compute_contacts_multi_body(multi_bodies, dispatcher)

        for cps in contacts:
            solver.resolve_collision(cps, dt)

        pd.integrate_euler(mb, dt)

    opengl_utils_pd.sync_visual_transforms(mb, robot_vis, app)
    app.renderer.write_transforms()
    app.renderer.render_scene()
    app.swap_buffer()
