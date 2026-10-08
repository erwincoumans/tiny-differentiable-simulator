// GLFW owns Wayland/EGL surfaces, input, resizing and frame scheduling. Loading
// its stable C API dynamically keeps the X11 build free of a GLFW dependency.
#if defined(__linux__)
#include "tiny_wayland_opengl_window.h"
#include "glad/gl.h"
#include <dlfcn.h>
#include <cstdlib>
#include <stdexcept>
#include <map>
#include <algorithm>

namespace {
struct GLFWwindow;
using GLProc = void (*)();
using KeyCallback = void (*)(GLFWwindow*, int, int, int, int);
using CursorCallback = void (*)(GLFWwindow*, double, double);
using ButtonCallback = void (*)(GLFWwindow*, int, int, int);
using ScrollCallback = void (*)(GLFWwindow*, double, double);
using SizeCallback = void (*)(GLFWwindow*, int, int);
// GLFW 3.4 stable ABI constants. No GLFW development headers are required.
constexpr int PLATFORM = 0x00050003, WAYLAND = 0x00060003;
constexpr int CONTEXT_MAJOR = 0x00022002, CONTEXT_MINOR = 0x00022003;
constexpr int OPENGL_PROFILE = 0x00022008, CORE_PROFILE = 0x00032001;
constexpr int CONTEXT_API = 0x0002200b, EGL_API = 0x00036002;
struct Api {
  void* library = nullptr;
  int (*Init)();
  void (*InitHint)(int,int);
  int (*GetPlatform)();
  void (*DefaultWindowHints)();
  void (*WindowHint)(int,int);
  GLFWwindow* (*CreateWindow)(int,int,const char*,void*,GLFWwindow*);
  void (*DestroyWindow)(GLFWwindow*);
  void (*MakeContextCurrent)(GLFWwindow*);
  GLFWwindow* (*GetCurrentContext)();
  GLProc (*GetProcAddress)(const char*);
  void (*SwapBuffers)(GLFWwindow*);
  void (*SwapInterval)(int);
  int (*GetError)(const char**);
  void (*PollEvents)();
  int (*WindowShouldClose)(GLFWwindow*);
  void (*SetWindowShouldClose)(GLFWwindow*,int);
  void (*GetWindowSize)(GLFWwindow*,int*,int*);
  void (*GetFramebufferSize)(GLFWwindow*,int*,int*);
  void (*GetCursorPos)(GLFWwindow*,double*,double*);
  int (*GetKey)(GLFWwindow*,int);
  void (*SetWindowTitle)(GLFWwindow*,const char*);
  double (*GetTime)();
  KeyCallback (*SetKeyCallback)(GLFWwindow*,KeyCallback);
  CursorCallback (*SetCursorPosCallback)(GLFWwindow*,CursorCallback);
  ButtonCallback (*SetMouseButtonCallback)(GLFWwindow*,ButtonCallback);
  ScrollCallback (*SetScrollCallback)(GLFWwindow*,ScrollCallback);
  SizeCallback (*SetFramebufferSizeCallback)(GLFWwindow*,SizeCallback);

  explicit Api(const char* path) {
    const char* selected = path && *path ? path : std::getenv("PYTINYOPENGL3_GLFW_LIBRARY");
    library = dlopen(selected && *selected ? selected : "libglfw.so.3", RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(std::string("Native Wayland needs GLFW 3.4+ with Wayland support: ")+dlerror());
    try {
#define LOAD(name) name = reinterpret_cast<decltype(name)>(dlsym(library,"glfw" #name)); if (!name) throw std::runtime_error("GLFW 3.4+ required: missing glfw" #name)
      LOAD(Init); LOAD(InitHint); LOAD(GetPlatform); LOAD(DefaultWindowHints); LOAD(WindowHint);
      LOAD(CreateWindow); LOAD(DestroyWindow); LOAD(MakeContextCurrent); LOAD(GetCurrentContext);
      LOAD(GetProcAddress); LOAD(SwapBuffers); LOAD(SwapInterval); LOAD(GetError); LOAD(PollEvents);
      LOAD(WindowShouldClose); LOAD(SetWindowShouldClose); LOAD(GetWindowSize); LOAD(GetFramebufferSize);
      LOAD(GetCursorPos); LOAD(GetKey); LOAD(SetWindowTitle); LOAD(GetTime); LOAD(SetKeyCallback);
      LOAD(SetCursorPosCallback); LOAD(SetMouseButtonCallback); LOAD(SetScrollCallback); LOAD(SetFramebufferSizeCallback);
#undef LOAD
      InitHint(PLATFORM,WAYLAND);
      if (!Init() || GetPlatform() != WAYLAND) throw std::runtime_error("GLFW could not initialize native Wayland (check WAYLAND_DISPLAY and GLFW Wayland support)");
    } catch (...) { dlclose(library); library=nullptr; throw; }
  }
  // Keep a successfully initialized library resident for the process lifetime:
  // glfwTerminate/dlclose could invalidate windows belonging to another client.
};
Api& api(const char* path = nullptr) {
  static Api instance(path);
  return instance;
}
int tinyKey(int key) {
  if (key >= 65 && key <= 90) return key + 32;
  if (key >= 32 && key <= 126) return key;
  if (key >= 290 && key <= 304) return TINY_KEY_F1 + key - 290;
  switch (key) {
    case 256: return TINY_KEY_ESCAPE; case 257: return TINY_KEY_RETURN;
    case 258: return TINY_KEY_TAB; case 259: return TINY_KEY_BACKSPACE;
    case 260: return TINY_KEY_INSERT; case 261: return TINY_KEY_DELETE;
    case 262: return TINY_KEY_RIGHT_ARROW; case 263: return TINY_KEY_LEFT_ARROW;
    case 264: return TINY_KEY_DOWN_ARROW; case 265: return TINY_KEY_UP_ARROW;
    case 266: return TINY_KEY_PAGE_UP; case 267: return TINY_KEY_PAGE_DOWN;
    case 268: return TINY_KEY_HOME; case 269: return TINY_KEY_END;
    case 340: case 344: return TINY_KEY_SHIFT;
    case 341: case 345: return TINY_KEY_CONTROL;
    case 342: case 346: return TINY_KEY_ALT;
    default: return -1;
  }
}
}
struct TinyWaylandOpenGLWindow::Data {
  GLFWwindow* window = nullptr;
  Api& gl;
  TinyMouseMoveCallback move = nullptr;
  TinyMouseButtonCallback button = nullptr;
  TinyResizeCallback resize = nullptr;
  TinyWheelCallback wheel = nullptr;
  TinyKeyboardCallback key = nullptr;
  TinyRenderCallback render = nullptr;
  explicit Data(const char* path) : gl(api(path)) {}
  static std::map<GLFWwindow*,Data*>& windows() { static std::map<GLFWwindow*,Data*> value; return value; }
  static void onKey(GLFWwindow* w,int key,int,int action,int) {
    auto* d=windows().at(w); int mapped=tinyKey(key);
    if (mapped >= 0 && d->key) d->key(mapped,action ? 1 : 0);
  }
  static void onCursor(GLFWwindow* w,double x,double y) {
    auto* d=windows().at(w); if (d->move) d->move(float(x),float(y));
  }
  static void onButton(GLFWwindow* w,int button,int action,int) {
    auto* d=windows().at(w); double x,y; d->gl.GetCursorPos(w,&x,&y);
    // GLFW right/middle are 1/2; Tiny uses left/middle/right = 0/1/2.
    int mapped=button==1 ? 2 : button==2 ? 1 : button;
    if (d->button) d->button(mapped,action ? 1 : 0,float(x),float(y));
  }
  static void onScroll(GLFWwindow* w,double x,double y) {
    auto* d=windows().at(w); if(d->wheel) d->wheel(float(x*100),float(y*100));
  }
  static void onSize(GLFWwindow* w,int,int) {
    auto* d=windows().at(w); int x,y; d->gl.GetWindowSize(w,&x,&y);
    if(d->resize) d->resize(float(x),float(y));
  }
};
TinyWaylandOpenGLWindow::TinyWaylandOpenGLWindow(const char* path) : m_data(new Data(path)) {}
TinyWaylandOpenGLWindow::~TinyWaylandOpenGLWindow() { close_window(); }
void TinyWaylandOpenGLWindow::create_window(const TinyWindowConstructionInfo& ci) {
  close_window(); auto& d=*m_data;
  d.gl.DefaultWindowHints();
  d.gl.WindowHint(CONTEXT_MAJOR,3); d.gl.WindowHint(CONTEXT_MINOR,3);
  d.gl.WindowHint(OPENGL_PROFILE,CORE_PROFILE); d.gl.WindowHint(CONTEXT_API,EGL_API);
  d.window=d.gl.CreateWindow(ci.m_width,ci.m_height,ci.m_title,nullptr,nullptr);
  if (!d.window) throw std::runtime_error("Failed to create Wayland/EGL OpenGL 3.3 window");
  Data::windows()[d.window]=&d;
  d.gl.MakeContextCurrent(d.window);
  if (!gladLoadGL(reinterpret_cast<GLADloadfunc>(d.gl.GetProcAddress))) {
    close_window(); throw std::runtime_error("Failed to load Wayland OpenGL functions");
  }
  d.gl.SetKeyCallback(d.window,Data::onKey);
  d.gl.SetCursorPosCallback(d.window,Data::onCursor);
  d.gl.SetMouseButtonCallback(d.window,Data::onButton);
  d.gl.SetScrollCallback(d.window,Data::onScroll);
  d.gl.SetFramebufferSizeCallback(d.window,Data::onSize);
}
void TinyWaylandOpenGLWindow::close_window() {
  auto& d=*m_data; if (d.window) { d.gl.DestroyWindow(d.window); Data::windows().erase(d.window); d.window=nullptr; }
}
void TinyWaylandOpenGLWindow::pump_messages() { m_data->gl.PollEvents(); }
void TinyWaylandOpenGLWindow::start_rendering() {
  pump_messages(); auto& d=*m_data; if(!d.window) return;
  d.gl.MakeContextCurrent(d.window); int x,y; d.gl.GetFramebufferSize(d.window,&x,&y);
  glViewport(0,0,x,y); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT); glEnable(GL_DEPTH_TEST);
}
void TinyWaylandOpenGLWindow::end_rendering() { if(m_data->window) m_data->gl.SwapBuffers(m_data->window); }
void TinyWaylandOpenGLWindow::run_main_loop() { while(!requested_exit()) { start_rendering(); if(m_data->render) m_data->render(); end_rendering(); } }
float TinyWaylandOpenGLWindow::get_time_in_seconds() { return float(m_data->gl.GetTime()); }
bool TinyWaylandOpenGLWindow::requested_exit() const { return !m_data->window || m_data->gl.WindowShouldClose(m_data->window); }
void TinyWaylandOpenGLWindow::set_request_exit() { if(m_data->window) m_data->gl.SetWindowShouldClose(m_data->window,1); }
bool TinyWaylandOpenGLWindow::set_vsync(bool enabled) {
  auto& d=*m_data; if(!d.window || d.gl.GetCurrentContext()!=d.window) return false;
  d.gl.GetError(nullptr); d.gl.SwapInterval(enabled ? 1 : 0); return d.gl.GetError(nullptr)==0;
}
bool TinyWaylandOpenGLWindow::is_modifier_key_pressed(int key) {
  if(!m_data->window) return false;
  int left=key==TINY_KEY_SHIFT ? 340 : key==TINY_KEY_CONTROL ? 341 : key==TINY_KEY_ALT ? 342 : -1;
  return left>=0 && (m_data->gl.GetKey(m_data->window,left) || m_data->gl.GetKey(m_data->window,left+4));
}
void TinyWaylandOpenGLWindow::set_window_title(const char* title) { if(m_data->window) m_data->gl.SetWindowTitle(m_data->window,title); }
float TinyWaylandOpenGLWindow::get_retina_scale() const {
  int w,h,fw,fh; if(!m_data->window) return 1;
  m_data->gl.GetWindowSize(m_data->window,&w,&h); m_data->gl.GetFramebufferSize(m_data->window,&fw,&fh);
  return w>0 ? float(fw)/w : 1.f;
}
void TinyWaylandOpenGLWindow::set_allow_retina(bool) {} // Wayland compositor controls output scaling.
int TinyWaylandOpenGLWindow::get_width() const { int w=0,h=0; if(m_data->window) m_data->gl.GetWindowSize(m_data->window,&w,&h); return w; }
int TinyWaylandOpenGLWindow::get_height() const { int w=0,h=0; if(m_data->window) m_data->gl.GetWindowSize(m_data->window,&w,&h); return h; }
#define CALLBACK(type,name,field) \
void TinyWaylandOpenGLWindow::set_##name##_callback(type cb) { m_data->field=cb; } \
type TinyWaylandOpenGLWindow::get_##name##_callback() { return m_data->field; }
CALLBACK(TinyMouseMoveCallback,mouse_move,move)
CALLBACK(TinyMouseButtonCallback,mouse_button,button)
CALLBACK(TinyResizeCallback,resize,resize)
CALLBACK(TinyWheelCallback,wheel,wheel)
CALLBACK(TinyKeyboardCallback,keyboard,key)
#undef CALLBACK
void TinyWaylandOpenGLWindow::set_render_callback(TinyRenderCallback cb) { m_data->render=cb; }
#endif
