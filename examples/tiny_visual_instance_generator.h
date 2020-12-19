
#ifndef TINY_VISUAL_INSTANCE_GENERATOR_H
#define TINY_VISUAL_INSTANCE_GENERATOR_H

#include "opengl_urdf_visualizer.h"

template <typename Algebra>
struct TinyVisualInstanceGenerator : public VisualInstanceGenerator
{
    OpenGLUrdfVisualizer<Algebra>& viz_;

    TinyVisualInstanceGenerator(OpenGLUrdfVisualizer<Algebra>& viz) : viz_(viz) {

    }
    virtual void create_visual_instance(int shape_uid, std::vector<int>& visual_instances )
    {
        auto& vis_link = viz_.m_b2vis[shape_uid];
        for (int v = 0; v < vis_link.visual_shape_uids.size(); v++)
        {
            int visual_shape = vis_link.visual_shape_uids[v];
            ::TINY::TinyVector3f color(1, 1, 1);
            ::TINY::TinyVector3f pos(0, 0, 0);
            ::TINY::TinyQuaternionf orn(0, 0, 0, 1);
            ::TINY::TinyVector3f scaling(1, 1, 1);
            
            //visualizer.m_b2vis
            int instance = viz_.m_opengl_app.m_renderer->register_graphics_instance(
                visual_shape, pos, orn, color, scaling);
            visual_instances.push_back(instance);
        }
    }
};

#endif
