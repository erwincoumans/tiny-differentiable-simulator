#ifndef VISUAL_INSTANCE_GENERATOR_H
#define VISUAL_INSTANCE_GENERATOR_H

#include <vector>

struct PyBulletInstanceGenerator : public VisualInstanceGenerator
{
    PyBulletVisualizerAPI* sim_;
    PyBulletInstanceGenerator(PyBulletVisualizerAPI* sim) : sim_(sim) {
    }

    virtual void create_visual_instance(int shape_uid, std::vector<int>& visual_instances)
    {
        b3RobotSimulatorCreateMultiBodyArgs args2;
        args2.m_baseVisualShapeIndex = shape_uid;
        args2.m_baseMass = 0;
        int visual_instance = sim_->createMultiBody(args2);
        visual_instances.push_back(visual_instance);
        b3RobotSimulatorChangeVisualShapeArgs args_change;
        args_change.m_objectUniqueId = visual_instance;
        args_change.m_linkIndex = -1;
        args_change.m_hasRgbaColor = true;
        args_change.m_rgbaColor.setValue(1, 1, 1, 1);
        sim_->changeVisualShape(args_change);
    }
};

#endif //VISUAL_INSTANCE_GENERATOR_H
