
#include "visualizer/opengl/utils/tiny_clock.h"
#include <fstream>
#include <iostream>
#include <signal.h>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <map>
#include "visualizer/opengl/tiny_opengl3_app.h"
#include "visualizer/opengl/utils/tiny_chrome_trace_util.h"
#include "visualizer/opengl/utils/tiny_logging.h"

#include "rbdl/Dynamics.h"
#include "rbdl/Model.h"
#include "rbdl/rbdl.h"
#include "urdfreader.h"
#include "utils/file_utils.hpp"


///////////////

#include <iostream>

///////////////


#include "imgui.h"
//#include "imgui_impl_glfw.h"
#include "opengl3/imgui_impl_opengl3.h"
//#include "imfilebrowser.h"
#include "imnodes.h"

#include "imgui.h"
#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif
#include "imgui_internal.h"

using namespace TINY;

bool releasedA = false;
TinyKeyboardCallback default_keyboard_callback = 0;

TinyMouseMoveCallback default_mouse_move_callback = 0;
TinyMouseButtonCallback default_mouse_button_callback = 0;

float g_MouseWheel = 0;
float gMouseX = 0;
float gMouseY = 0;
int g_MousePressed[3] = { 0,0,0 };

void MyMouseMoveCallback(float x, float y)
{
    gMouseX = x;
    gMouseY = y;
    if (default_mouse_move_callback)
        default_mouse_move_callback(x, y);
}
void MyMouseButtonCallback(int button, int state, float x, float y)
{
    gMouseX = x;
    gMouseY = y;
    g_MousePressed[button] = state;
    if (default_mouse_button_callback)
        default_mouse_button_callback(button, state, x, y);
}









void MyKeyboardCallback(int keycode, int state)
{

    if ((keycode == 'a') && (state == 0))
    {
        releasedA = true;
        //printf("keycode=%d, state=%d\n", keycode, state);
        //printf("key=%d\n", 'a');
    }

   
    ImGuiIO& io = ImGui::GetIO();
    if (state == 1)
        io.KeysDown[keycode] = true;
    if (state == 0)
        io.KeysDown[keycode] = false;

    if (state==1 &&
        (
            (keycode >= 'a' && keycode <= 'z') ||
            (keycode >= '0' && keycode <= '9') ||
            (keycode == ' ') ||
            (keycode == '!') || (keycode == '@') || (keycode == '#') || (keycode == '$') || (keycode == '%') || (keycode == '.') ||
            (keycode == ',') || (keycode == '\"') || (keycode == '\'') || (keycode == '~') || (keycode == '-') || (keycode == '=') ||
            (keycode == '+') || (keycode == '[') || (keycode == ']') || (keycode == '?') || (keycode == '*') || (keycode == '(') ||
            (keycode == ')') || (keycode == '^') || (keycode == '&')
        )
       )
    {
        io.AddInputCharacter(keycode);
    }
    // Modifiers are not reliable across systems
    if (keycode == TINY_KEY_ALT)
        io.KeyAlt = state;
    if (keycode == TINY_KEY_CONTROL)
        io.KeyCtrl = state;
    if (keycode == TINY_KEY_SHIFT)
        io.KeyShift = state;

//#ifdef _WIN32
    io.KeySuper = false;
//#else
//    io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
//#endif

    if (default_keyboard_callback)
    {
        default_keyboard_callback(keycode, state);
    }



}

RigidBodyDynamics::Model m_rbdl_model;

void step_rbdl(Eigen::VectorXd& Q,Eigen::VectorXd& QDot, double dt)
{
    std::vector<RigidBodyDynamics::Math::SpatialVector>* f_ext = 0;

    Eigen::VectorXd QDDot(m_rbdl_model.qdot_size);
    Eigen::VectorXd Tau(m_rbdl_model.qdot_size);
    for(int k=0;k<m_rbdl_model.qdot_size;k++)
    {
        QDDot(k) = 0.;
        Tau(k) = 0.;
    }
    RigidBodyDynamics::ForwardDynamics(m_rbdl_model,Q,QDot,Tau,QDDot,f_ext);

    QDot += QDDot*dt;

    //in case spherical joints and floating base, need to switch
    for(int i = 0 ; i<m_rbdl_model.mJoints.size();i++)
    {
        unsigned int q_index = m_rbdl_model.mJoints[i].q_index;
        unsigned int multdof3_w_index = m_rbdl_model.multdof3_w_index[i];
        switch(m_rbdl_model.mJoints[i].mJointType)
        {
        case RigidBodyDynamics::JointTypeSpherical:
        {
#if 0
            auto orn = Algebra::quat_from_xyzw(Q[q_index],
                Q[q_index + 1],
                Q[q_index + 2],
                Q[multdof3_w_index]);
            auto angvel = Algebra::Vector3(QDot[q_index],QDot[q_index+1],QDot[q_index+2]);
            //auto new_orn = Algebra::quat_integrate(orn,angvel,dt);

            Algebra::quat_increment(
                orn,Algebra::quat_velocity(orn,angvel,dt));

            Q(q_index+0) = orn.x();
            Q[q_index+1] = orn.y();
            Q[q_index+2] = orn.z();
            Q[multdof3_w_index] = orn.w();
#endif
            break;
        }
        case     RigidBodyDynamics::JointTypeRevolute:
        case     RigidBodyDynamics::JointTypePrismatic:
        case     RigidBodyDynamics::JointTypeRevoluteX:
        case     RigidBodyDynamics::JointTypeRevoluteY:
        case     RigidBodyDynamics::JointTypeRevoluteZ:
        {
            Q[q_index] += QDot[q_index]*dt;
            break;
        }
        case RigidBodyDynamics::JointTypeTranslationXYZ:
        {
            Q(q_index+0) += QDot(q_index+0)*dt;
            Q(q_index+1) += QDot(q_index+1)*dt;
            Q(q_index+2) += QDot(q_index+2)*dt;
            break;
        }
        case RigidBodyDynamics::JointTypeUndefined:
        {
            break;
        }

        default:
        {
            assert(0);
        }
        }
    };


}

int main(int argc, char* argv[]) {
    
    
    std::string urdf_filename;
    tds::FileUtils::find_file("sphere_small.urdf",urdf_filename);

    bool verbose= false;
    bool floating_base = true;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_filename.c_str(),&m_rbdl_model,floating_base,verbose);
    m_rbdl_model.gravity.set(0,0,-10);// = to_rbdl<Algebra>(world.get_gravity());
    Eigen::VectorXd Q(m_rbdl_model.q_size);
    Eigen::VectorXd QDot(m_rbdl_model.qdot_size);
    
    int rbdl_index_without_w = 0;
    //order of floating base RBDL is:
    //[root translation xyz, root orientation quaternion x,y,z, [...other joints], root orientation w,...]
    //TDS uses
    //[root orientation x,y,z,w, root translation x,y,z, [... other joints]
    //rbdl
    Q(0) = 0;
    Q(1) = 0;
    Q(2) = 1;//height
    Q(3) = 0;
    Q(4) = 0;
    Q(5) = 0;
    int w_index = m_rbdl_model.multdof3_w_index[2];//??
    Q(w_index) = 1;

    for(int k=0;k<m_rbdl_model.qdot_size;k++)
    {
        QDot(k) = 0.;
    }


    TinyOpenGL3App app("test", 1920, 1080);
    app.m_renderer->init();
    
    bool profile_timings = false;
    if (profile_timings)
        TinyChromeUtilsStartTimings();

    TinyClock clock;
    clock.reset();
    double prev_time = clock.get_time_seconds();
       
    app.set_up_axis(2);
    app.m_renderer->get_active_camera()->set_camera_distance(4);
    app.m_renderer->get_active_camera()->set_camera_pitch(-30);
    app.m_renderer->get_active_camera()->set_camera_target_position(0, 0, 0);

    default_mouse_move_callback = app.m_window->get_mouse_move_callback();
    app.m_window->set_mouse_move_callback(MyMouseMoveCallback);
    default_mouse_button_callback = app.m_window->get_mouse_button_callback();
    app.m_window->set_mouse_button_callback(MyMouseButtonCallback);
    default_keyboard_callback = app.m_window->get_keyboard_callback();
    app.m_window->set_keyboard_callback(MyKeyboardCallback);
    TinyVector3f pos(0, 0, -0.02);
    TinyQuaternionf orn(0, 0, 0, 1);
    TinyVector3f color(1, 1, 1);
    TinyVector3f scaling(1, 1, 1);
    int textureIndex = -1;
    int red = 0;
    int green = 128;
    int blue = 255;

    int texWidth = 1024;
    int texHeight = 1024;
    std::vector<unsigned char> texels;
    texels.resize(texWidth * texHeight * 3);
    for (int i = 0; i < texWidth * texHeight * 3; i++) texels[i] = 255;

    for (int i = 0; i < texWidth; i++) {
        for (int j = 0; j < texHeight; j++) {
            int a = i < texWidth / 2 ? 1 : 0;
            int b = j < texWidth / 2 ? 1 : 0;

            if (a == b) {
                texels[(i + j * texWidth) * 3 + 0] = red;
                texels[(i + j * texWidth) * 3 + 1] = green;
                texels[(i + j * texWidth) * 3 + 2] = blue;
            }
        }
    }

    textureIndex = app.m_instancingRenderer->register_texture(
        &texels[0], texWidth, texHeight);

    int shape = app.register_cube_shape(10, 10, 0.01, textureIndex, 40);  //, 10);
    app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);
    pos.setValue(0, 0, 2);
    shape = app.register_graphics_unit_sphere_shape(SPHERE_LOD_HIGH);
    TinyVector3f sphere_scaling(0.03,0.03,0.03);
    int sphere_id = app.m_renderer->register_graphics_instance(shape, pos, orn, color,sphere_scaling);

    app.m_renderer->write_transforms();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
        
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    io.KeyMap[ImGuiKey_Tab] = TINY_KEY_TAB;
    io.KeyMap[ImGuiKey_LeftArrow] = TINY_KEY_LEFT_ARROW;
    io.KeyMap[ImGuiKey_RightArrow] = TINY_KEY_RIGHT_ARROW;
    io.KeyMap[ImGuiKey_UpArrow] = TINY_KEY_UP_ARROW;
    io.KeyMap[ImGuiKey_DownArrow] = TINY_KEY_DOWN_ARROW;
    io.KeyMap[ImGuiKey_PageUp] = TINY_KEY_PAGE_UP;
    io.KeyMap[ImGuiKey_PageDown] = TINY_KEY_PAGE_DOWN;
    io.KeyMap[ImGuiKey_Home] = TINY_KEY_HOME;
    io.KeyMap[ImGuiKey_End] = TINY_KEY_END;
    io.KeyMap[ImGuiKey_Insert] = TINY_KEY_INSERT;
    io.KeyMap[ImGuiKey_Delete] = TINY_KEY_DELETE;
    io.KeyMap[ImGuiKey_Backspace] = TINY_KEY_BACKSPACE;
    io.KeyMap[ImGuiKey_Space] = '`';// ' ';
    io.KeyMap[ImGuiKey_Enter] = TINY_KEY_RETURN;
    io.KeyMap[ImGuiKey_Escape] = TINY_KEY_ESCAPE;
    io.KeyMap[ImGuiKey_KeyPadEnter] = 0;
        
    io.KeyMap[ImGuiKey_A] = 'a';
    io.KeyMap[ImGuiKey_C] = 'c';
    io.KeyMap[ImGuiKey_V] = 'v';
    io.KeyMap[ImGuiKey_X] = 'x';
    io.KeyMap[ImGuiKey_Y] = 'y';
    io.KeyMap[ImGuiKey_Z] = 'z';


    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    if (0)
    {
      float SCALE = 2.0f;
      ImFontConfig cfg;
      cfg.SizePixels = 13 * SCALE;
      ImGui::GetIO().Fonts->AddFontDefault(&cfg)->DisplayOffset.y = SCALE;
      ImGui::GetStyle().ScaleAllSizes(2);
    }
    //ImGui::StyleColorsClassic();
    const char* glsl_version = "#version 330";

    // Setup Platform/Renderer bindings
    ImGui_ImplOpenGL3_Init(glsl_version);
    bool res = ImGui_ImplOpenGL3_CreateDeviceObjects();


    app.m_renderer->get_active_camera()->set_camera_distance(3);

    while (!app.m_window->requested_exit()) {
        B3_PROFILE("mainloop");
        int upAxis = 2;

        app.m_renderer->update_camera(upAxis);
        DrawGridData data;
        data.drawAxis = true;
        data.upAxis = 2;

        {
            // B3_PROFILE("draw_grid");
            //app.draw_grid(data);
        }
        //const char* bla = "3d label";
        //app.draw_text_3d(bla, 0, 0, 1, 1);

        step_rbdl(Q,QDot, 0.001);
        

        bool update_kinematics = true;
        RigidBodyDynamics::Math::Vector3d rbdl_local(0,0,0);
        int body_a_id = 2;//??
        //auto pos_world_a = RigidBodyDynamics::CalcBodyToBaseCoordinates(m_rbdl_model,Q,body_a_id,rbdl_local,update_kinematics);
        //auto     orn_world_a = RigidBodyDynamics::CalcBodyWorldOrientation(m_rbdl_model,Q,body_a_id,update_kinematics);

#if 0
        #print("orn_world_a=",orn_world_a)
            #print("orn_world_a[0]=",orn_world_a[0])
            #print("orn_world_a[0][0]=",orn_world_a[0][0])
            orn_a_mat = p.TinyMatrix3x3f(orn_world_a[0][0],orn_world_a[1][0],orn_world_a[2][0],
                orn_world_a[0][1],orn_world_a[1][1],orn_world_a[2][1],
                orn_world_a[0][2],orn_world_a[1][2],orn_world_a[2][2])
            orn_a = orn_a_mat.getRotation()
#endif

        pos[0] = Q(0);
        pos[1] = Q(1);
        pos[2] = Q(2);
        orn[0] = Q(3);
        orn[1] = Q(4);
        orn[2] = Q(5);
        orn[3] = Q(w_index);
        app.m_renderer->write_single_instance_transform_to_cpu(pos,orn,sphere_id);
        app.m_renderer->write_transforms();

        {
            B3_PROFILE("render_scene");
            app.m_renderer->render_scene();
        }



        float width = (float)app.m_window->get_width();
        float height = (float)app.m_window->get_height();

        io.DisplaySize = ImVec2((float)width, (float)height);
        io.DisplayFramebufferScale = ImVec2(app.m_window->get_retina_scale(), app.m_window->get_retina_scale());
        double t = clock.get_time_seconds();
        float dt = t - prev_time;
        prev_time = t;
        io.DeltaTime = (float)dt;
        io.MousePos = ImVec2((float)gMouseX, (float)gMouseY);
        io.RenderDrawListsFn = ImGui_ImplOpenGL3_RenderDrawData;// ImGui_ImplBullet_RenderDrawLists;

        for (int i = 0; i < 3; i++)
        {
            io.MouseDown[i] = g_MousePressed[i];
        }
        io.MouseWheel = g_MouseWheel;

        ImGui::NewFrame();

        
        
        ImGui::ShowDemoWindow();
     
        ImGui::SetNextWindowSize(ImVec2(300,300));
        {
            const char* window_name = "Hi";
            if (ImGui::Begin(window_name))
            {
                static int clicked = 0;
                if (ImGui::Button("Play!"))
                    clicked++;
                if (clicked & 1)
                {
                    ImGui::SameLine();
                    ImGui::Text("Thanks for clicking me!");
                }
            }
            ImGui::End();
        }
       

        ImGui::Render();
        ImGui::EndFrame();
        {
            // B3_PROFILE("swap_buffer");
            app.swap_buffer();
        }
    }

    //example::NodeEditorShutdown();

  if(profile_timings)
    TinyChromeUtilsStopTimingsAndWriteJsonFile("diffsim.json");

  return 0;
}

