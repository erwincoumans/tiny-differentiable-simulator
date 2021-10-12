try:
    import pytinydiffsim as pd
    import pytinyopengl3 as p
except ImportError as e:
    raise error.DependencyNotInstalled(
        "{}. (HINT: you need to install pytinydiffsim, and also perform the \
        setup instructions here: \
        https://github.com/google-research/tiny-differentiable-simulator.)".format(
            e
        )
    )

class VisualLinkInfo(object):

    def __init__(self):
        self.uid = 1
        self.vis_name = ""
        self.origin_rpy = [1, 2, 3]
        self.origin_xyz = [4, 5, 6]


def convert_link_visuals(link, link_index, app, uid, b2vis, path_prefix,
collisions):
    '''
        Link:
        Link Index:
        App:

    '''
    # print("Link name", link.link_name)
    # print("convert_link_visuals:: num_visuals=", len(link.urdf_visual_shapes))
    # print("link.urdf_visual_shapes=", link.urdf_visual_shapes)
    # print("link.urdf_collision_shapes", link.urdf_collision_shapes)

    if collisions:
        geoms = link.urdf_collision_shapes
    else:
        geoms = link.urdf_visual_shapes

    for v in geoms:
        vis_name = link.link_name + str(uid)
        b2v = VisualLinkInfo()
        b2v.vis_name = vis_name
        b2v.link_index = link_index
        b2v.origin_rpy = v.origin_rpy
        b2v.origin_xyz = v.origin_xyz
        b2v.inertia_xyz = link.urdf_inertial.origin_xyz
        b2v.inertia_rpy = link.urdf_inertial.origin_rpy
        print("name", link.link_name)
        if v.geometry.geom_type == pd.SPHERE_TYPE:
            print("We've got a sphere!!")
            if (v.geometry.sphere.radius > 0.):
                radius = v.geometry.sphere.radius
                half_height = 0
                up_axis = 2
                texture_index = -1

                shape = app.register_graphics_capsule_shape(radius,
                                                    half_height,
                                                    up_axis,
                                                    texture_index)

                orn = p.TinyQuaternionf(0.0, 0.0, 0.0, 1.0)
                rpy = p.TinyVector3f(b2v.origin_rpy[0],
                                     b2v.origin_rpy[1],
                                     b2v.origin_rpy[2])
                orn.set_euler_rpy(rpy)

                pos = b2v.origin_xyz
                pos = p.TinyVector3f(pos[0], pos[1], pos[2])

                color = p.TinyVector3f(1., 0., 0.)
                scale = p.TinyVector3f(1., 1., 1.)

                alpha = 0.5
                b2v.mesh = app.renderer.register_graphics_instance(shape,
                    pos, orn, color, scale, alpha, True)

        if v.geometry.geom_type == pd.MESH_TYPE:
            print("mesh filename=", path_prefix + v.geometry.mesh.file_name)
            mesh_filename = path_prefix + v.geometry.mesh.file_name
            scaling = v.geometry.mesh.scale
            scaling = p.TinyVector3f(scaling[0], scaling[1], scaling[2])

            orn = p.TinyQuaternionf(0.0, 0.0, 0.0, 1.0)
            rpy = p.TinyVector3f(b2v.origin_rpy[0],
                                 b2v.origin_rpy[1],
                                 b2v.origin_rpy[2])

            orn.set_euler_rpy(rpy)

            pos = b2v.origin_xyz
            pos = p.TinyVector3f(pos[0], pos[1], pos[2])
            shapes = p.load_obj_shapes(app, mesh_filename, pos, orn, scaling)
            shape = shapes[0]
            print("registering shape", shape)
            color = p.TinyVector3f(1., 1., 1.)

            opacity = 1
            rebuild = True
            b2v.mesh = app.renderer.register_graphics_instance(shape,
                                                    pos,
                                                    orn,
                                                    color,
                                                    scaling,
                                                    opacity,
                                                    rebuild)

        if v.geometry.geom_type == pd.CAPSULE_TYPE:
            print("We've got a capsule!!")
            if (v.geometry.capsule.radius > 0.):
                radius = v.geometry.capsule.radius
                half_height = v.geometry.capsule.length
                up_axis = 2
                texture_index = -1

                shape = app.register_graphics_capsule_shape(radius,
                                                    half_height,
                                                    up_axis,
                                                    texture_index)

                orn = p.TinyQuaternionf(0.0, 0.0, 0.0, 1.0)
                rpy = p.TinyVector3f(b2v.origin_rpy[0],
                                     b2v.origin_rpy[1],
                                     b2v.origin_rpy[2])
                orn.set_euler_rpy(rpy)

                pos = b2v.origin_xyz
                pos = p.TinyVector3f(pos[0], pos[1], pos[2])

                color = p.TinyVector3f(1., 0., 0.)
                scale = p.TinyVector3f(1., 1., 1.)

                alpha = 0.5
                b2v.mesh = app.renderer.register_graphics_instance(shape,
                    pos, orn, color, scale, alpha, True)

        if v.geometry.geom_type == pd.PLANE_TYPE:
            print("We've got a plane!!")

        b2v.uid = uid
        b2vis[uid] = b2v
        uid += 1

    return b2vis, uid


def convert_visuals(urdf, app, path_prefix="", collisions=False):

    # convert_visuals takes in input the urdf data and generates a
    # scene that is displayed by OpenGL (app)
    link_name_to_index = {}
    link_name_to_index[urdf.base_links[0].link_name] = -1
    for link_index in range(len(urdf.links)):
        l = urdf.links[link_index]
        link_name_to_index[l.link_name] = link_index
    b2vis = {}
    uid = -1
    link_index = -1

    print("link_name_to_index", link_name_to_index)
    # print("urdf.base_links", urdf.base_links)
    # print("converting base link")
    b2v, uid = convert_link_visuals(urdf.base_links[0], link_index, app,
                                  uid, b2vis, path_prefix, collisions)

    # print("converting joints")
    # then convert each child link
    for joint in urdf.joints:
        link_index = link_name_to_index[joint.child_name]
        link = urdf.links[link_index]

        b2v, uid = convert_link_visuals(link, link_index, app,
                                        uid, b2vis, path_prefix,
                                        collisions)

    return b2vis


def sync_visual_transforms(mb, b2vis, app, collisions=False):
    # sync_visual_transforms takes in input the multi body (mb) representation
    # of the model, the visualization data and the app.

    # the goal is to apply the transformations contained in the multi body
    # in the visualization data and display them in the OpenGL app
    for key in b2vis:
        v = b2vis[key]
        link_world_trans = mb.get_world_transform(v.link_index)

        vpos = v.origin_xyz
        vorn = pd.Quaternion(0.0, 0.0, 0.0, 1.0)
        vorn.set_euler_rpy(v.origin_rpy)
        trv = pd.TinySpatialTransform()
        trv.translation = vpos
        trv.rotation = pd.Matrix3(vorn)
        gfx_world_trans = link_world_trans * trv

        m = gfx_world_trans.rotation
        m = p.TinyMatrix3x3f(m.get_at(0, 0), m.get_at(0, 1), m.get_at(0, 2),
                             m.get_at(1, 0), m.get_at(1, 1), m.get_at(1, 2),
                             m.get_at(2, 0), m.get_at(2, 1), m.get_at(2, 2))
        orn = m.getRotation()

        print("sync_visual_transforms orn", orn)

        pos = gfx_world_trans.translation
        pos = p.TinyVector3f(pos[0], pos[1], pos[2])

        try:
            res = app.renderer.write_single_instance_transform_to_cpu(pos,
                orn, v.mesh)

        except:
            if collisions:
                print("Error!!")
            pass
