// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef B3_USE_GLFW
#include "glfw_opengl_window.h"

#include <GLFW/glfw3.h>
#include <glad/gl.h>

#include <stdio.h>
#include <stdlib.h>

struct GLFWOpenGLWindowInternalData {
  bool m_requestedExit;
  bool m_hasCursorPos;
  bool m_altPressed;
  bool m_shiftPressed;
  bool m_ctrlPressed;
  float m_cursorXPos;
  float m_cursorYPos;
  b3MouseMoveCallback m_mouseMoveCallback;
  b3MouseButtonCallback m_mouseButtonCallback;
  b3ResizeCallback m_resizeCallback;
  TinyWheelCallback m_wheelCallback;
  b3KeyboardCallback m_keyboardCallback;
  b3RenderCallback m_renderCallback;
  int m_width;
  int m_height;
  float m_retinaScaleFactor;

  GLFWwindow* m_glfwWindow;

  GLFWOpenGLWindowInternalData()
      : m_requestedExit(false),
        m_hasCursorPos(false),
        m_altPressed(false),
        m_shiftPressed(false),
        m_ctrlPressed(false),
        m_cursorXPos(0),
        m_cursorYPos(0),
        m_mouseMoveCallback(0),
        m_mouseButtonCallback(0),
        m_resizeCallback(0),
        m_wheelCallback(0),
        m_keyboardCallback(0),
        m_renderCallback(0),
        m_width(0),
        m_height(0),
        m_retinaScaleFactor(1),
        m_glfwWindow(0) {}
};

static void GLFWErrorCallback(int error, const char* description) {
  fprintf(stderr, "Error: %s\n", description);
}

static void GLFWMouseButtonCallback(GLFWwindow* window, int button,
                                    int glfwState, int) {
  GLFWOpenGLWindow* wnd = (GLFWOpenGLWindow*)glfwGetWindowUserPointer(window);
  if (wnd && wnd->get_mouse_button_callback()) {
    int state = (glfwState == GLFW_PRESS) ? 1 : 0;
    wnd->mouseButtonCallbackInternal(button, state);
  }
}

static void GLFWScrollCallback(GLFWwindow* window, double deltaX,
                               double deltaY) {
  GLFWOpenGLWindow* wnd = (GLFWOpenGLWindow*)glfwGetWindowUserPointer(window);
  if (wnd && wnd->get_wheel_callback()) {
    wnd->get_wheel_callback()(deltaX * 100, deltaY * 100);
  }
}

static void GLFWCursorPosCallback(GLFWwindow* window, double xPos,
                                  double yPos) {
  GLFWOpenGLWindow* wnd = (GLFWOpenGLWindow*)glfwGetWindowUserPointer(window);
  if (wnd && wnd->get_mouse_move_callback()) {
    wnd->mouseCursorCallbackInternal(xPos, yPos);
  }
}

static void GLFWKeyCallback(GLFWwindow* window, int key, int scancode,
                            int action, int mods) {
  GLFWOpenGLWindow* wnd = (GLFWOpenGLWindow*)glfwGetWindowUserPointer(window);
  if (wnd) {
    wnd->keyboardCallbackInternal(key, action);
  }
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

static void GLFWSizeCallback(GLFWwindow* window, int width, int height) {
  GLFWOpenGLWindow* wnd = (GLFWOpenGLWindow*)glfwGetWindowUserPointer(window);
  { wnd->resizeInternal(width, height); }
}

GLFWOpenGLWindow::GLFWOpenGLWindow() {
  m_data = new GLFWOpenGLWindowInternalData();
}

GLFWOpenGLWindow::~GLFWOpenGLWindow() {
  if (m_data->m_glfwWindow) {
    close_window();
  }
  delete m_data;
}

int getBulletKeyFromGLFWKeycode(int glfwKeyCode) {
  int keycode = -1;
  if (glfwKeyCode >= 'A' && glfwKeyCode <= 'Z') {
    return glfwKeyCode + 32;  // todo: fix the ascii A vs a input
  }
  if (glfwKeyCode >= '0' && glfwKeyCode <= '9') {
    return glfwKeyCode;
  }

  switch (glfwKeyCode) {
    case GLFW_KEY_ENTER: {
      keycode = B3G_RETURN;
      break;
    };
    case GLFW_KEY_ESCAPE: {
      keycode = B3G_ESCAPE;
      break;
    };
    case GLFW_KEY_F1: {
      keycode = B3G_F1;
      break;
    }
    case GLFW_KEY_F2: {
      keycode = B3G_F2;
      break;
    }
    case GLFW_KEY_F3: {
      keycode = B3G_F3;
      break;
    }
    case GLFW_KEY_F4: {
      keycode = B3G_F4;
      break;
    }
    case GLFW_KEY_F5: {
      keycode = B3G_F5;
      break;
    }
    case GLFW_KEY_F6: {
      keycode = B3G_F6;
      break;
    }
    case GLFW_KEY_F7: {
      keycode = B3G_F7;
      break;
    }
    case GLFW_KEY_F8: {
      keycode = B3G_F8;
      break;
    }
    case GLFW_KEY_F9: {
      keycode = B3G_F9;
      break;
    }
    case GLFW_KEY_F10: {
      keycode = B3G_F10;
      break;
    }

      // case GLFW_KEY_SPACE: {keycode= ' '; break;}

    case GLFW_KEY_PAGE_DOWN: {
      keycode = B3G_PAGE_DOWN;
      break;
    }
    case GLFW_KEY_PAGE_UP: {
      keycode = B3G_PAGE_UP;
      break;
    }

    case GLFW_KEY_INSERT: {
      keycode = B3G_INSERT;
      break;
    }
    case GLFW_KEY_BACKSPACE: {
      keycode = B3G_BACKSPACE;
      break;
    }
    case GLFW_KEY_DELETE: {
      keycode = B3G_DELETE;
      break;
    }

    case GLFW_KEY_END: {
      keycode = B3G_END;
      break;
    }
    case GLFW_KEY_HOME: {
      keycode = B3G_HOME;
      break;
    }
    case GLFW_KEY_LEFT: {
      keycode = B3G_LEFT_ARROW;
      break;
    }
    case GLFW_KEY_UP: {
      keycode = B3G_UP_ARROW;
      break;
    }
    case GLFW_KEY_RIGHT: {
      keycode = B3G_RIGHT_ARROW;
      break;
    }
    case GLFW_KEY_DOWN: {
      keycode = B3G_DOWN_ARROW;
      break;
    }
    case GLFW_KEY_RIGHT_SHIFT: {
      keycode = B3G_SHIFT;
      break;
    }
    case GLFW_KEY_LEFT_SHIFT: {
      keycode = B3G_SHIFT;
      break;
    }
    case GLFW_KEY_MENU: {
      keycode = B3G_ALT;
      break;
    }
    case GLFW_KEY_RIGHT_CONTROL: {
      keycode = B3G_CONTROL;
      break;
    }
    case GLFW_KEY_LEFT_CONTROL: {
      keycode = B3G_CONTROL;
      break;
    }
    default: {
      // keycode = MapVirtualKey( virtualKeyCode, MAPGLFW_KEY_GLFW_KEY_TO_CHAR )
      // & 0x0000FFFF;
    }
  };

  return keycode;
}

void GLFWOpenGLWindow::keyboardCallbackInternal(int key, int state) {
  if (get_keyboard_callback()) {
    // convert keyboard codes from glfw to bullet
    int btcode = getBulletKeyFromGLFWKeycode(key);
    int btstate = (state == GLFW_RELEASE) ? 0 : 1;

    switch (btcode) {
      case B3G_SHIFT: {
        m_data->m_shiftPressed = state != 0;
        break;
      }
      case B3G_ALT: {
        m_data->m_altPressed = state != 0;
        break;
      }
      case B3G_CONTROL: {
        m_data->m_ctrlPressed = state != 0;
        break;
      }
      default: {}
    }

    get_keyboard_callback()(btcode, btstate);
  }
}

void GLFWOpenGLWindow::mouseButtonCallbackInternal(int button, int state) {
  if (get_mouse_button_callback() && m_data->m_hasCursorPos) {
    get_mouse_button_callback()(button, state, m_data->m_cursorXPos,
                                m_data->m_cursorYPos);
  }
}

void GLFWOpenGLWindow::mouseCursorCallbackInternal(double xPos, double yPos) {
  if (get_mouse_move_callback()) {
    m_data->m_hasCursorPos = true;
    m_data->m_cursorXPos = xPos;
    m_data->m_cursorYPos = yPos;
    get_mouse_move_callback()(xPos, yPos);
  }
}

void GLFWOpenGLWindow::resizeInternal(int width, int height) {
  glfwGetFramebufferSize(m_data->m_glfwWindow, &m_data->m_width,
                         &m_data->m_height);
  glViewport(0, 0, m_data->m_width, m_data->m_height);

  if (getResizeCallback()) {
    getResizeCallback()(m_data->m_width / m_data->m_retinaScaleFactor,
                        m_data->m_height / m_data->m_retinaScaleFactor);
  }
}

void GLFWOpenGLWindow::create_default_window(int width, int height,
                                             const char* title) {
  b3gWindowConstructionInfo ci;
  ci.m_width = width;
  ci.m_height = height;
  ci.m_title = title;

  create_window(ci);
}

void GLFWOpenGLWindow::create_window(const b3gWindowConstructionInfo& ci) {
  btAssert(m_data->m_glfwWindow == 0);
  if (m_data->m_glfwWindow == 0) {
    glfwSetErrorCallback(GLFWErrorCallback);

    if (!glfwInit()) exit(EXIT_FAILURE);

    if (ci.m_openglVersion == 2) {
      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    } else {
      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    }

    m_data->m_glfwWindow =
        glfwCreateWindow(ci.m_width, ci.m_height, ci.m_title, NULL, NULL);

    if (!m_data->m_glfwWindow) {
      glfwTerminate();
      exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(m_data->m_glfwWindow, GLFWKeyCallback);
    glfwSetMouseButtonCallback(m_data->m_glfwWindow, GLFWMouseButtonCallback);

    glfwSetCursorPosCallback(m_data->m_glfwWindow, GLFWCursorPosCallback);
    glfwSetScrollCallback(m_data->m_glfwWindow, GLFWScrollCallback);

    glfwSetWindowSizeCallback(m_data->m_glfwWindow, GLFWSizeCallback);
    glfwSetWindowUserPointer(m_data->m_glfwWindow, this);

    glfwMakeContextCurrent(m_data->m_glfwWindow);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(0);  // 1);
    glfwGetFramebufferSize(m_data->m_glfwWindow, &m_data->m_width,
                           &m_data->m_height);
    int windowWidth, windowHeight;
    glfwGetWindowSize(m_data->m_glfwWindow, &windowWidth, &windowHeight);
    m_data->m_retinaScaleFactor = float(m_data->m_width) / float(windowWidth);
    glViewport(0, 0, m_data->m_width, m_data->m_height);
  }
}

void GLFWOpenGLWindow::close_window() {
  if (m_data->m_glfwWindow) {
    glfwDestroyWindow(m_data->m_glfwWindow);

    glfwTerminate();
    m_data->m_glfwWindow = 0;
  }
}

void GLFWOpenGLWindow::run_main_loop() {}

float GLFWOpenGLWindow::get_time_in_seconds() { return 0.f; }

bool GLFWOpenGLWindow::requested_exit() const {
  bool shouldClose = m_data->m_requestedExit;

  if (m_data->m_glfwWindow) {
    shouldClose = shouldClose || glfwWindowShouldClose(m_data->m_glfwWindow);
  }
  return shouldClose;
}

void GLFWOpenGLWindow::set_request_exit() {
  if (m_data->m_glfwWindow) {
    glfwSetWindowShouldClose(m_data->m_glfwWindow, GLFW_TRUE);
  }
  m_data->m_requestedExit = true;
}

void GLFWOpenGLWindow::start_rendering() {
  if (m_data->m_glfwWindow) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  }
}

void GLFWOpenGLWindow::end_rendering() {
  glfwPollEvents();
  glfwSwapBuffers(m_data->m_glfwWindow);
}

bool GLFWOpenGLWindow::is_modifier_key_pressed(int key) {
  bool result = false;

  switch (key) {
    case B3G_SHIFT: {
      result = m_data->m_shiftPressed;
      break;
    }
    case B3G_ALT: {
      result = m_data->m_altPressed;
      break;
    }
    case B3G_CONTROL: {
      result = m_data->m_ctrlPressed;
      break;
    }
    default: {}
  }
  return result;
}

void GLFWOpenGLWindow::set_mouse_move_callback(
    b3MouseMoveCallback mouseCallback) {
  m_data->m_mouseMoveCallback = mouseCallback;
}

b3MouseMoveCallback GLFWOpenGLWindow::get_mouse_move_callback() {
  return m_data->m_mouseMoveCallback;
}

void GLFWOpenGLWindow::set_mouse_button_callback(
    b3MouseButtonCallback mouseCallback) {
  m_data->m_mouseButtonCallback = mouseCallback;
}

b3MouseButtonCallback GLFWOpenGLWindow::get_mouse_button_callback() {
  return m_data->m_mouseButtonCallback;
}

void GLFWOpenGLWindow::set_resize_callback(b3ResizeCallback resizeCallback) {
  m_data->m_resizeCallback = resizeCallback;
  getResizeCallback()(m_data->m_width / get_retina_scale(),
                      m_data->m_height / get_retina_scale());
}

b3ResizeCallback GLFWOpenGLWindow::getResizeCallback() {
  return m_data->m_resizeCallback;
}

void GLFWOpenGLWindow::set_wheel_callback(TinyWheelCallback wheelCallback) {
  m_data->m_wheelCallback = wheelCallback;
}

TinyWheelCallback GLFWOpenGLWindow::get_wheel_callback() {
  return m_data->m_wheelCallback;
}

void GLFWOpenGLWindow::set_keyboard_callback(
    b3KeyboardCallback keyboardCallback) {
  m_data->m_keyboardCallback = keyboardCallback;
}

b3KeyboardCallback GLFWOpenGLWindow::get_keyboard_callback() {
  return m_data->m_keyboardCallback;
}

void GLFWOpenGLWindow::set_render_callback(b3RenderCallback renderCallback) {
  m_data->m_renderCallback = renderCallback;
}

void GLFWOpenGLWindow::set_window_title(const char* title) {
  if (m_data->m_glfwWindow) {
    glfwSetWindowTitle(m_data->m_glfwWindow, title);
  }
}

float GLFWOpenGLWindow::get_retina_scale() const {
  return m_data->m_retinaScaleFactor;
}
void GLFWOpenGLWindow::set_allow_retina(bool allow) {}

int GLFWOpenGLWindow::get_width() const {
  if (m_data->m_glfwWindow) {
    glfwGetFramebufferSize(m_data->m_glfwWindow, &m_data->m_width,
                           &m_data->m_height);
  }
  int width = m_data->m_width / m_data->m_retinaScaleFactor;
  return width;
}
int GLFWOpenGLWindow::get_height() const {
  if (m_data->m_glfwWindow) {
    glfwGetFramebufferSize(m_data->m_glfwWindow, &m_data->m_width,
                           &m_data->m_height);
  }
  return m_data->m_height / m_data->m_retinaScaleFactor;
}

int GLFWOpenGLWindow::file_open_dialog(char* fileName, int maxFileNameLength) {
  return 0;
}

#endif  // B3_USE_GLFW
