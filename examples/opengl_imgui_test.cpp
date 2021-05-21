
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




namespace example
{
    namespace
    {
        struct Node
        {
            int id;
            float value;

            Node() = default;

            Node(const int i, const float v) : id(i), value(v) {}
        };

        struct Link
        {
            int id;
            int start_attr, end_attr;
        };

        class SaveLoadEditor
        {
        public:
            SaveLoadEditor() : nodes_(), links_(), current_id_(0) {}

            void show()
            {
                //ImGui::Begin("Save & load example");
                ImGui::TextUnformatted("A -- add node");
                ImGui::TextUnformatted(
                    "Close the executable and rerun it -- your nodes should be exactly "
                    "where you left them!");

                imnodes::BeginNodeEditor();

                if ((ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && imnodes::IsEditorHovered() && releasedA))
                {
                    releasedA = false;
                    const int node_id = ++current_id_;
                    imnodes::SetNodeScreenSpacePos(node_id, ImGui::GetMousePos());
                    nodes_.push_back(Node(node_id, 0.f));
                }
                
                int node_index = 0;
                for (Node& node : nodes_)
                {
                    imnodes::BeginNode(node.id);

                    imnodes::BeginNodeTitleBar();
                    char nodename[1024];
                    sprintf(nodename, "node %d", node_index++);
                    ImGui::TextUnformatted("node");
                    imnodes::EndNodeTitleBar();

                    imnodes::BeginInputAttribute(node.id << 8);
                    ImGui::TextUnformatted("input");
                    imnodes::EndInputAttribute();

                    imnodes::BeginStaticAttribute(node.id << 16);
                    ImGui::PushItemWidth(120.f);
                    ImGui::DragFloat("value", &node.value, 0.01f);
                    ImGui::PopItemWidth();
                    imnodes::EndStaticAttribute();

                    imnodes::BeginOutputAttribute(node.id << 24);
                    const float text_width = ImGui::CalcTextSize("output").x;
                    ImGui::Indent(120.f + ImGui::CalcTextSize("value").x - text_width);
                    ImGui::TextUnformatted("output");
                    imnodes::EndOutputAttribute();

                    imnodes::EndNode();
                }

                for (const Link& link : links_)
                {
                    imnodes::Link(link.id, link.start_attr, link.end_attr);
                }

               

                imnodes::EndNodeEditor();

                {
                    Link link;
                    if (imnodes::IsLinkCreated(&link.start_attr, &link.end_attr))
                    {
                        link.id = ++current_id_;
                        links_.push_back(link);
                    }
                }

                {
                    int link_id;
                    if (imnodes::IsLinkDestroyed(&link_id))
                    {
                        auto iter =
                            std::find_if(links_.begin(), links_.end(), [link_id](const Link& link) -> bool {
                            return link.id == link_id;
                                });
                        assert(iter != links_.end());
                        links_.erase(iter);
                    }
                }

                //ImGui::End();
            }

            void save()
            {
                // Save the internal imnodes state
                imnodes::SaveCurrentEditorStateToIniFile("save_load.ini");
                //imnodes::SaveEditorStateToIniFile()
                // Dump our editor state as bytes into a file

                std::fstream fout(
                    "save_load.bytes", std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);

                // copy the node vector to file
                const size_t num_nodes = nodes_.size();
                fout.write(
                    reinterpret_cast<const char*>(&num_nodes),
                    static_cast<std::streamsize>(sizeof(size_t)));
                fout.write(
                    reinterpret_cast<const char*>(nodes_.data()),
                    static_cast<std::streamsize>(sizeof(Node) * num_nodes));

                // copy the link vector to file
                const size_t num_links = links_.size();
                fout.write(
                    reinterpret_cast<const char*>(&num_links),
                    static_cast<std::streamsize>(sizeof(size_t)));
                fout.write(
                    reinterpret_cast<const char*>(links_.data()),
                    static_cast<std::streamsize>(sizeof(Link) * num_links));

                // copy the current_id to file
                fout.write(
                    reinterpret_cast<const char*>(&current_id_), static_cast<std::streamsize>(sizeof(int)));
            }

            void load()
            {
                // Load the internal imnodes state
                imnodes::LoadCurrentEditorStateFromIniFile("save_load.ini");

                // Load our editor state into memory

                std::fstream fin("save_load.bytes", std::ios_base::in | std::ios_base::binary);

                if (!fin.is_open())
                {
                    return;
                }

                // copy nodes into memory
                size_t num_nodes;
                fin.read(reinterpret_cast<char*>(&num_nodes), static_cast<std::streamsize>(sizeof(size_t)));
                nodes_.resize(num_nodes);
                fin.read(
                    reinterpret_cast<char*>(nodes_.data()),
                    static_cast<std::streamsize>(sizeof(Node) * num_nodes));

                // copy links into memory
                size_t num_links;
                fin.read(reinterpret_cast<char*>(&num_links), static_cast<std::streamsize>(sizeof(size_t)));
                links_.resize(num_links);
                fin.read(
                    reinterpret_cast<char*>(links_.data()),
                    static_cast<std::streamsize>(sizeof(Link) * num_links));

                // copy current_id into memory
                fin.read(reinterpret_cast<char*>(&current_id_), static_cast<std::streamsize>(sizeof(int)));
            }

        private:
            std::vector<Node> nodes_;
            std::vector<Link> links_;
            int current_id_;
        };

        static SaveLoadEditor editor;
    } // namespace

    void NodeEditorInitialize()
    {
        imnodes::GetIO().link_detach_with_modifier_click.modifier = &ImGui::GetIO().KeyCtrl;
        imnodes::PushAttributeFlag(imnodes::AttributeFlags_EnableLinkDetachWithDragClick);
        editor.load();
    }

    void NodeEditorShow() { editor.show(); }

    void NodeEditorShutdown()
    {
        imnodes::PopAttributeFlag();
        editor.save();
    }
} // namespace example












float noteToFreq(int note) {
    float a = 440.; // frequency of A (coomon value is 440Hz)
    return (a / 3.) * pow(2., ((note - 9.) / 12.));
}
std::map<char, int> key2note = {
    {'a',0},
        {'w',1},
    {'s',2},
        {'e',3},
    {'d',4},
    {'f',5},
        {'t',6},
    {'g',7},
        {'y',8},
    {'h',9},
        {'u',10},
    {'j',11},
    {'k',12},
};

int octave = 1;

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


//std::vector<char> custom_pool;
#define POOL_SIZE (1024 * 1024 * 16)
char custom_pool[POOL_SIZE];
size_t pool_index = 0;
int allocation_count = 0;

void* custom_pool_allocate(size_t size)
{
    printf("================\n");
    printf("allocation %d\n", allocation_count++);
    printf("current usage: %d\n", pool_index);
    printf("allocation: %d\n", size);

    if (pool_index + size >= POOL_SIZE )
    {
        printf("Running out of memory!\n");
        exit(0);
    }
    assert(pool_index + size < POOL_SIZE );
    void* ptr = &custom_pool[pool_index];
    pool_index += size;
    return ptr;
}

ImGuiID  MyDockSpace()
{
    ImGuiID dockspace_id = 0;
    
    bool p_open = true;
    static bool opt_fullscreen = true;
    static bool opt_padding = false;
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_PassthruCentralNode;// ImGuiDockNodeFlags_None;

    // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
    // because it would be confusing to have two docking targets within each others.
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    if (opt_fullscreen)
    {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->GetWorkPos());
        ImGui::SetNextWindowSize(viewport->GetWorkSize());
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
        dockspace_flags |= ImGuiDockNodeFlags_PassthruCentralNode;
    }
    else
    {
        dockspace_flags &= ~ImGuiDockNodeFlags_PassthruCentralNode;
    }

    // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
    // and handle the pass-thru hole, so we ask Begin() to not render a background.
    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
        window_flags |= ImGuiWindowFlags_NoBackground;

    // Important: note that we proceed even if Begin() returns false (aka window is collapsed).
    // This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
    // all active windows docked into it will lose their parent and become undocked.
    // We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
    // any change of dockspace/settings would lead to windows being stuck in limbo and never being visible.
    if (!opt_padding)
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace Demo", &p_open, window_flags);
    if (!opt_padding)
        ImGui::PopStyleVar();

    if (opt_fullscreen)
        ImGui::PopStyleVar(2);

    // DockSpace
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
    {
        dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
    }
    
    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Options"))
        {
            // Disabling fullscreen would allow the window to be moved to the front of other windows,
            // which we can't undo at the moment without finer window depth/z control.
            ImGui::MenuItem("Fullscreen", NULL, &opt_fullscreen);
            ImGui::MenuItem("Padding", NULL, &opt_padding);
            ImGui::Separator();

            if (ImGui::MenuItem("Flag: NoSplit", "", (dockspace_flags & ImGuiDockNodeFlags_NoSplit) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_NoSplit; }
            if (ImGui::MenuItem("Flag: NoResize", "", (dockspace_flags & ImGuiDockNodeFlags_NoResize) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_NoResize; }
            if (ImGui::MenuItem("Flag: NoDockingInCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_NoDockingInCentralNode) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_NoDockingInCentralNode; }
            if (ImGui::MenuItem("Flag: AutoHideTabBar", "", (dockspace_flags & ImGuiDockNodeFlags_AutoHideTabBar) != 0)) { dockspace_flags ^= ImGuiDockNodeFlags_AutoHideTabBar; }
            if (ImGui::MenuItem("Flag: PassthruCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode) != 0, opt_fullscreen)) { dockspace_flags ^= ImGuiDockNodeFlags_PassthruCentralNode; }
            ImGui::Separator();

            if (ImGui::MenuItem("Close", NULL, false, p_open != NULL))
                p_open = false;
            ImGui::EndMenu();
        }
      
        ImGui::EndMenuBar();
    }


    ImGui::End();
    return dockspace_id;
}


struct MyAppLog
{
    ImGuiTextBuffer     Buf;
    
    ImVector<int>       LineOffsets; // Index to lines offset. We maintain this with AddLog() calls.
    bool                AutoScroll;  // Keep scrolling if already at the bottom.

    MyAppLog()
    {
        AutoScroll = true;
        Clear();
    }

    void    Clear()
    {
        Buf.clear();
        LineOffsets.clear();
        LineOffsets.push_back(0);
    }

    void    AddLog(const char* fmt, ...) IM_FMTARGS(2)
    {
        int old_size = Buf.size();
        va_list args;
        va_start(args, fmt);
        Buf.appendfv(fmt, args);
        va_end(args);
        for (int new_size = Buf.size(); old_size < new_size; old_size++)
            if (Buf[old_size] == '\n')
                LineOffsets.push_back(old_size + 1);
    }

    void    Draw(const char* title, bool* p_open = NULL)
    {
        if (!ImGui::Begin(title, p_open))
        {
            ImGui::End();
            return;
        }

        if (ImGui::SmallButton("[Debug] Add 5 entries"))
        {
            static int counter = 0;
            const char* categories[3] = { "info", "warn", "error" };
            const char* words[] = { "Bumfuzzled", "Cattywampus", "Snickersnee", "Abibliophobia", "Absquatulate", "Nincompoop", "Pauciloquent" };
            for (int n = 0; n < 5; n++)
            {
                const char* category = categories[counter % IM_ARRAYSIZE(categories)];
                const char* word = words[counter % IM_ARRAYSIZE(words)];
                AddLog("[%05d] [%s] Hello, current time is %.1f, here's a word: '%s'\n",
                    ImGui::GetFrameCount(), category, ImGui::GetTime(), word);
                counter++;
            }
        }
        ImGui::SameLine();
#if 0
        // Options menu
        if (ImGui::BeginPopup("Options"))
        {
            ImGui::Checkbox("Auto-scroll", &AutoScroll);
            ImGui::EndPopup();
        }

        // Main window
        if (ImGui::Button("Options"))
            ImGui::OpenPopup("Options");
#endif
        ImGui::SameLine();
        bool clear = ImGui::SmallButton("Clear");
        ImGui::SameLine();
        bool copy = ImGui::SmallButton("Copy");
        ImGui::SameLine();
        

        ImGui::Separator();
        ImGui::BeginChild("scrolling", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

        if (clear)
            Clear();
        if (copy)
            ImGui::LogToClipboard();

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 0));
        const char* buf = Buf.begin();
        const char* buf_end = Buf.end();
        if (0)//Filter.IsActive())
        {
#if 0
            // In this example we don't use the clipper when Filter is enabled.
            // This is because we don't have a random access on the result on our filter.
            // A real application processing logs with ten of thousands of entries may want to store the result of
            // search/filter.. especially if the filtering function is not trivial (e.g. reg-exp).
            for (int line_no = 0; line_no < LineOffsets.Size; line_no++)
            {
                const char* line_start = buf + LineOffsets[line_no];
                const char* line_end = (line_no + 1 < LineOffsets.Size) ? (buf + LineOffsets[line_no + 1] - 1) : buf_end;
                if (Filter.PassFilter(line_start, line_end))
                    ImGui::TextUnformatted(line_start, line_end);
            }
#endif
        }
        else
        {
            // The simplest and easy way to display the entire buffer:
            //   ImGui::TextUnformatted(buf_begin, buf_end);
            // And it'll just work. TextUnformatted() has specialization for large blob of text and will fast-forward
            // to skip non-visible lines. Here we instead demonstrate using the clipper to only process lines that are
            // within the visible area.
            // If you have tens of thousands of items and their processing cost is non-negligible, coarse clipping them
            // on your side is recommended. Using ImGuiListClipper requires
            // - A) random access into your data
            // - B) items all being the  same height,
            // both of which we can handle since we an array pointing to the beginning of each line of text.
            // When using the filter (in the block of code above) we don't have random access into the data to display
            // anymore, which is why we don't use the clipper. Storing or skimming through the search result would make
            // it possible (and would be recommended if you want to search through tens of thousands of entries).
            ImGuiListClipper clipper;
            clipper.Begin(LineOffsets.Size);
            while (clipper.Step())
            {
                for (int line_no = clipper.DisplayStart; line_no < clipper.DisplayEnd; line_no++)
                {
                    const char* line_start = buf + LineOffsets[line_no];
                    const char* line_end = (line_no + 1 < LineOffsets.Size) ? (buf + LineOffsets[line_no + 1] - 1) : buf_end;
                    ImGui::TextUnformatted(line_start, line_end);
                }
            }
            clipper.End();
        }
        ImGui::PopStyleVar();

        if (AutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndChild();
        ImGui::End();
    }
};

// Demonstrate creating a simple log window with basic filtering.
static void ShowMyAppLog(bool* p_open)
{
    static MyAppLog log;

    // For the demo: add a debug button _BEFORE_ the normal log window contents
    // We take advantage of a rarely used feature: multiple calls to Begin()/End() are appending to the _same_ window.
    // Most of the contents of the window will be added by the log.Draw() call.
#if 0
    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    
    ImGui::Begin("Log", p_open);
   
    ImGui::End();
#endif
    // Actually call in the regular Log helper (which will Begin() into the same window as we just did)
    log.Draw("Log2", p_open);
    

    
}


uint16_t reverb_buffer[65536];
int main(int argc, char* argv[]) {
    
    
    TinyOpenGL3App app("test", 1024, 768);
    app.m_renderer->init();
    
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
    app.m_renderer->register_graphics_instance(shape, pos, orn, color, scaling);

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
    if (1)
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


#if 0
    ImGui::FileBrowser fileDialog;

    // (optional) set browser properties
    fileDialog.SetTitle("title");
    fileDialog.SetTypeFilters({ ".wav" });
#endif
    imnodes::Initialize();
    //imnodes::SetNodeGridSpacePos(1, ImVec2(200.0f, 200.0f));

    // Setup style

    imnodes::StyleColorsDark();
    //imnodes::StyleColorsClassic();



    while (!app.m_window->requested_exit()) {
        B3_PROFILE("mainloop");
        int upAxis = 2;

        app.m_renderer->update_camera(upAxis);
        DrawGridData data;
        data.upAxis = 2;

        {
            // B3_PROFILE("draw_grid");
            //app.draw_grid(data);
        }
        const char* bla = "3d label";
        app.draw_text_3d(bla, 0, 0, 1, 1);
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

        ImGuiID dockSpaceId = MyDockSpace();
        
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
        ImGui::SetNextWindowSize(ImVec2(300, 300));
        {
            const char* window_name = "Hi2";
            if (ImGui::Begin(window_name))
            {
                static int clicked = 0;
                int prev_clicked = clicked;
                if (ImGui::Button("Play!"))
                {
                    clicked++;
                }
                if (prev_clicked != clicked)
                {
                    if (clicked & 1)
                    {
                        printf("on %d!\n", clicked);
                    }
                    else
                    {
                        printf("off %d!\n", clicked);
                    }
                    
                }
                if (clicked & 1)
                {
                    ImGui::SameLine();
                    ImGui::Text("Thanks for clicking me!");
                    
                }
                else
                {
                    
                }
            }

            
            static std::vector<const char*> items;
            static std::vector<std::string> samples;

            static ImGuiTextFilter     Filter;

            ImVec2 vMin = ImGui::GetWindowContentRegionMin();
            ImVec2 vMax = ImGui::GetWindowContentRegionMax();
            ImGui::PushItemWidth(vMax.x - vMin.x - 10);
            bool refresh = false;

            static int count = 0;

            if (Filter.Draw("Filter", -100.0f))
            {
                refresh = true;
                printf("count=%d\n", count++);
            }

            if (ImGui::Button("Refresh") || count == 0)
            {
                count++;
                samples.clear();
                refresh = true;
            }

            if (refresh)
            {
                items.clear();
                if (Filter.IsActive())
                {
                    for (int i = 0; i < samples.size(); i++)
                    {
                        if (Filter.PassFilter(samples[i].c_str()))
                        {
                            items.push_back(samples[i].c_str());
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < samples.size(); i++)
                    {
                        items.push_back(samples[i].c_str());
                    }
                }
            }

            static int item_current = 1;
            ImGui::PushItemWidth(vMax.x-vMin.x-10);
            //instruments
            ImGui::ListBox("", &item_current,items.size() ? &items[0] : 0, items.size(), 25);

            ImGui::End();
        }
        
        

        
        bool open = true;
        ShowMyAppLog(&open);

        static bool firstRun = true;

        if (firstRun) {
            ImGui::DockBuilderRemoveNode(dockSpaceId);

            ImGuiDockNodeFlags dockSpaceFlags = 0;
            dockSpaceFlags |= ImGuiDockNodeFlags_PassthruCentralNode;
            dockSpaceFlags |= ImGuiDockNodeFlags_DockSpace;
            dockSpaceFlags |= ImGuiDockNodeFlags_NoResize;
            ImGui::DockBuilderAddNode(dockSpaceId, dockSpaceFlags);
            ImGui::DockBuilderSetNodeSize(dockSpaceId, ImGui::GetMainViewport()->Size);

            ImGuiID dockMain = dockSpaceId;
            
            ImGuiID dockLeft = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Left, 0.2f, NULL, &dockMain);
            
            ImGuiID dockRight = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Right, 0.2f, NULL, &dockMain);
            ImGuiID dockBottom = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Down, 0.2f, NULL, &dockMain);

            ImGui::DockBuilderDockWindow("Hi", dockRight);
            ImGui::DockBuilderDockWindow("Dear ImGui Demo", dockRight);
            ImGui::DockBuilderDockWindow("Hi2", dockLeft);
            
            ImGui::DockBuilderDockWindow("Log", dockBottom);
            ImGui::DockBuilderDockWindow("Log2", dockBottom);

            

            ImGui::DockBuilderFinish(dockSpaceId);
            firstRun = false;
    }




#if 0
        /// <returns></returns>
        /// 

        if (ImGui::Begin("test_window"))
        {
            static bool initialized = false;
            if (!initialized)
            {
                initialized = true;
                example::NodeEditorInitialize();
            }


            //////////////////////////
            example::NodeEditorShow();
        }
        ImGui::End();

#if 0
        if (ImGui::Begin("play window"))
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
#endif

#if 0
        ImGui::Text("Hello, world!");
        static int counter = 0;
        if (ImGui::Button("Button"))  // Buttons return true when clicked (NB: most widgets return true when edited/activated)
            counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        
#endif
#endif

        ImGui::Render();
        ImGui::EndFrame();
        {
            // B3_PROFILE("swap_buffer");
            app.swap_buffer();
        }
    }

    //example::NodeEditorShutdown();



  TinyChromeUtilsStopTimingsAndWriteJsonFile("diffsim.json");

  
  return 0;
}

