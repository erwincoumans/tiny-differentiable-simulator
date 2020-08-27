#include "utils/tiny_clock.h"
#include <fstream>

#include <iostream>


#include "RtAudio.h"
//#include "Stk.h"


#include <signal.h>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "b3ReadWavFile.h"
#include "b3WriteWavFile.h"

#ifdef USE_WAV_READER
b3ReadWavFile wavReader;
b3WavTicker wavTicker;
#endif//USE_WAV_READER

#ifdef USE_WAV_WRITER
b3WriteWavFile wavWriter;
#endif //USE_WAV_WRITER

//using namespace stk;
double sampleRate = 44100.0;


#define MY_BUFFER_SIZE 512

static bool animate = true;

struct b3SoundOscillator
{
    int m_type;
    double m_frequency;
    double m_amplitude;
    double m_phase;

    
    double sampleSineWaveForm(double sampleRate)
    {
        while (m_phase >= 2*M_PI)
            m_phase -= 2 * M_PI;

        double z = sinf(m_phase);
        double sample = m_amplitude * z;

        m_phase += 2 * M_PI * (1. / sampleRate) * m_frequency;
        return sample;
    }

    double sampleSawWaveForm(double sampleRate)
    {
        while (m_phase >= 2 * M_PI)
            m_phase -= 2 * M_PI;

        double z = 2. * (m_phase) / 2 * M_PI - 1.;
        double sample = m_amplitude * z;

        m_phase += 2 * M_PI * (1. / sampleRate) * m_frequency;
        return sample;
    }

    void reset()
    {
        m_phase = 0;
    }

    b3SoundOscillator()
        : m_type(0),
        m_frequency(442.),
        m_amplitude(1),
        m_phase(0)
    {
        
    }
};

b3SoundOscillator leftOsc;
b3SoundOscillator rightOsc;

b3SoundOscillator leftOsc2;
b3SoundOscillator rightOsc2;
b3SoundOscillator lfo;

float wav_data[MY_BUFFER_SIZE];

int tick(void* outputBuffer, void* inputBuffer1, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status, void* dataPointer)
{
    

    double* samples = (double*)outputBuffer;
    int index = 0;
    for (int i = 0; i < nBufferFrames; i++)
    {
        double lfoMod = lfo.sampleSineWaveForm(sampleRate);
        
        leftOsc.m_frequency = 420 + 220 * lfoMod;
        rightOsc.m_frequency = 420 + 120 * lfoMod;
#ifdef USE_WAV_READER
        wavReader.tick(0, &wavTicker);
        if (wavTicker.finished_)
        {
            wavTicker.time_ = 0;
            wavTicker.finished_ = false;
        }
        samples[index] = wavTicker.lastFrame_[0];
#else
        samples[index] = leftOsc.sampleSineWaveForm(sampleRate);// +leftOsc2.sampleSineWaveForm(sampleRate));
#endif

        if (animate)
        {
            float scaling = 1;
            wav_data[i] = scaling*samples[index];
        }
        index++;
#ifdef USE_WAV_READER
        samples[index] = wavTicker.lastFrame_[1];
#else
        samples[index] = rightOsc2.sampleSineWaveForm(sampleRate);
#endif
        
        index++;

        //double data = env * m_data->m_oscillators[osc].m_amplitude * m_data->m_wavFilePtr->tick(frame, &m_data->m_oscillators[osc].m_wavTicker);
    }
#ifdef USE_WAV_WRITER
    wavWriter.tick(samples, nBufferFrames);
#endif
    return 0;
}

bool done;
static void finish(int ignore) 
{ 
    done = true; 
}






#include "opengl_window/tiny_opengl3_app.h"
#include "utils/tiny_chrome_trace_util.h"
#include "utils/tiny_logging.h"


#include "imgui.h"
//#include "imgui_impl_glfw.h"
#include "opengl3/imgui_impl_opengl3.h"
//#include "imfilebrowser.h"
#include "imnodes.h"


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












void MyKeyboardCallback(int keycode, int state)
{
    if ((keycode== 'a') && (state == 0))
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


int main(int argc, char* argv[]) {

    void* data = 0;

#ifdef USE_WAV_WRITER
    wavWriter.setWavFile("d:/mywav.wav", sampleRate, 2, true);
#endif
#ifdef USE_WAV_READER
    const char* wavFileName = "D:/ForestAmbience.wav";
    wavReader.getWavInfo(wavFileName);
    wavReader.resize();
    wavReader.read(0, true);
    wavTicker = wavReader.createWavTicker(sampleRate);
#endif
    RtAudio dac;
    int i;

    int audioDevice = -1;


    // If you want to change the default sample rate (set in Stk.h), do
    // it before instantiating any objects!  If the sample rate is
    // specified in the command line, it will override this setting.

    //Stk::setSampleRate(sampleRate);




    // Parse the command-line arguments.
    unsigned int port = 2001;
    for (i = 1; i < argc; i++) {

        //if (!strcmp(argv[i], "-s") && (i + 1 < argc) && argv[i + 1][0] != '-')
        //    Stk::setSampleRate(atoi(argv[++i]));
        if (!strcmp(argv[i], "-o") && (i + 1 < argc) && argv[i + 1][0] != '-')
            audioDevice = atoi(argv[++i]);
    }


    // Allocate the dac here.
    RtAudioFormat format = RTAUDIO_FLOAT64;// : RTAUDIO_FLOAT32;
    RtAudio::StreamParameters parameters;
    parameters.deviceId = audioDevice >= 0 ? audioDevice : dac.getDefaultOutputDevice();
    parameters.nChannels = 2;
    bool dacOK = true;

    unsigned int bufferFrames = MY_BUFFER_SIZE;
    try {
        dac.openStream(&parameters, NULL, format, (unsigned int)sampleRate, &bufferFrames, &tick, (void*)&data);
    }
    catch (RtAudioError& error) {
        error.printMessage();
        dacOK = false;
    }


    if (dacOK)
    {
        // Install an interrupt handler function.
        (void)signal(SIGINT, finish);

        // If realtime output, set our callback function and start the dac.
        try {
            dac.startStream();
        }
        catch (RtAudioError& error) {
            error.printMessage();
            dacOK = false;
        }
    }
    // Setup finished.
    //while (!done) {
        // Periodically check "done" status.
        //Stk::sleep(50);

    //}




    if (dacOK)
    {
       
        lfo.m_frequency = 1;


        //TinyChromeUtilsStartTimings();
        TinyClock clock;
        clock.reset();
        double prev_time = clock.get_time_seconds();
        TinyOpenGL3App app("test", 1024, 768);
        app.m_renderer->init();
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

            /// <summary>
#if 0
            if (ImGui::Begin("dummy window"))
            {
                // open file dialog when user clicks this button
                if (ImGui::Button("open file dialog"))
                    fileDialog.Open();
            }
            ImGui::End();

            fileDialog.Display();

            if (fileDialog.HasSelected())
            {
                std::cout << "Selected filename" << fileDialog.GetSelected().string() << std::endl;
                fileDialog.ClearSelected();
            }
#endif

            /// <returns></returns>
            /// 

            static bool initialized = false;
            if (!initialized)
            {
                initialized = true;
                example::NodeEditorInitialize();
            }


            //////////////////////////
            example::NodeEditorShow();

            //////////////////////////

            if (ImGui::Begin("wave window"))
            {
                
                ImGui::Checkbox("Animate", &animate);

                static float arr[] = { 0.6f, 0.1f, 1.0f, 0.5f, 0.92f, 0.1f, 0.2f };
                ImGui::PlotLines("Frame Times", arr, IM_ARRAYSIZE(arr));

                // Plots can display overlay texts
                // (in this example, we will display an average value)
                {
                    float average = 0.0f;
                    for (int n = 0; n < IM_ARRAYSIZE(wav_data); n++)
                        average += wav_data[n];
                    average /= (float)IM_ARRAYSIZE(wav_data);
                    char overlay[32];
                    sprintf(overlay, "avg %f", average);
                    ImGui::PlotLines("Lines", wav_data, IM_ARRAYSIZE(wav_data), 0, overlay, -1.0f, 1.0f, ImVec2(0, 80.0f));
                }
                
            }
            ImGui::End();
            
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


            ImGui::Text("Hello, world!");
            static int counter = 0;
            if (ImGui::Button("Button"))  // Buttons return true when clicked (NB: most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::ShowDemoWindow();
            ImGui::Render();
            ImGui::EndFrame();
            {
                // B3_PROFILE("swap_buffer");
                app.swap_buffer();
            }
        }

        example::NodeEditorShutdown();
    

  // Shut down the output stream.
  try {
      dac.closeStream();
  }
  catch (RtAudioError& error) {
      error.printMessage();
  }
  }


#ifdef USE_WAV_WRITER
  wavWriter.closeWavFile();
#endif

  return 0;

  //TinyChromeUtilsStopTimingsAndWriteJsonFile("diffsim.json");

}

