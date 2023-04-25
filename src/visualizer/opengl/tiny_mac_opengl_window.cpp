#ifndef B3_USE_GLFW
#ifdef __APPLE__

#include "tiny_mac_opengl_window.h"

#include "tiny_mac_opengl_window_objc.h"
#include "tiny_opengl_include.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

MacOpenGLWindow::MacOpenGLWindow() : m_internalData(0) {
  m_internalData = Mac_createData();
}

MacOpenGLWindow::~MacOpenGLWindow() { Mac_destroyData(m_internalData); }

void MacOpenGLWindow::close_window() {
  Mac_destroyData(m_internalData);
  m_internalData = Mac_createData();
}

bool MacOpenGLWindow::is_modifier_key_pressed(int key) {
  return Mac_isModifierKeyPressed(m_internalData, key);
}

float MacOpenGLWindow::get_time_in_seconds() { return 0.f; }

void MacOpenGLWindow::set_render_callback(TinyRenderCallback renderCallback) {}

void MacOpenGLWindow::set_window_title(const char* windowTitle) {
  Mac_setWindowTitle(m_internalData, windowTitle);
}

void MacOpenGLWindow::create_window(const TinyWindowConstructionInfo& ci) {
  MacWindowConstructionInfo windowCI;
  windowCI.m_width = ci.m_width;
  windowCI.m_height = ci.m_height;
  windowCI.m_fullscreen = ci.m_fullscreen;
  windowCI.m_colorBitsPerPixel = ci.m_colorBitsPerPixel;
  windowCI.m_windowHandle = ci.m_windowHandle;
  windowCI.m_title = ci.m_title;
  windowCI.m_openglVersion = ci.m_openglVersion;
  windowCI.m_allowRetina = true;

  Mac_createWindow(m_internalData, &windowCI);
}

void MacOpenGLWindow::run_main_loop() {}

void MacOpenGLWindow::pump_messages()
{
  Mac_pumpMessage(m_internalData);
}

void MacOpenGLWindow::start_rendering() { Mac_updateWindow(m_internalData); }

void MacOpenGLWindow::end_rendering() { Mac_swapBuffer(m_internalData); }

bool MacOpenGLWindow::requested_exit() const {
  return Mac_requestedExit(m_internalData);
}

void MacOpenGLWindow::set_request_exit() { Mac_setRequestExit(m_internalData); }

int MacOpenGLWindow::file_open_dialog(char* filename, int maxNameLength) {
  return Mac_fileOpenDialog(filename, maxNameLength);
}

void MacOpenGLWindow::get_mouse_coordinates(int& x, int& y) {
  int* xPtr = &x;
  int* yPtr = &y;

  Mac_getMouseCoordinates(m_internalData, xPtr, yPtr);
}

int MacOpenGLWindow::get_width() const { return Mac_getWidth(m_internalData); }

int MacOpenGLWindow::get_height() const {
  return Mac_getHeight(m_internalData);
}

void MacOpenGLWindow::set_resize_callback(TinyResizeCallback resizeCallback) {
  Mac_setResizeCallback(m_internalData, resizeCallback);
}

TinyResizeCallback MacOpenGLWindow::get_resize_callback() {
  return Mac_getResizeCallback(m_internalData);
}

void MacOpenGLWindow::set_mouse_button_callback(
    TinyMouseButtonCallback mouseCallback) {
  Mac_setMouseButtonCallback(m_internalData, mouseCallback);
}

void MacOpenGLWindow::set_mouse_move_callback(
    TinyMouseMoveCallback mouseCallback) {
  Mac_setMouseMoveCallback(m_internalData, mouseCallback);
}

void MacOpenGLWindow::set_keyboard_callback(
    TinyKeyboardCallback keyboardCallback) {
  Mac_setKeyboardCallback(m_internalData, keyboardCallback);
}

TinyMouseMoveCallback MacOpenGLWindow::get_mouse_move_callback() {
  return Mac_getMouseMoveCallback(m_internalData);
}

TinyMouseButtonCallback MacOpenGLWindow::get_mouse_button_callback() {
  return Mac_getMouseButtonCallback(m_internalData);
}

void MacOpenGLWindow::set_wheel_callback(TinyWheelCallback wheelCallback) {
  Mac_setWheelCallback(m_internalData, wheelCallback);
}

TinyWheelCallback MacOpenGLWindow::get_wheel_callback() {
  return Mac_getWheelCallback(m_internalData);
}

TinyKeyboardCallback MacOpenGLWindow::get_keyboard_callback() {
  return Mac_getKeyboardCallback(m_internalData);
}

float MacOpenGLWindow::get_retina_scale() const {
  return Mac_getRetinaScale(m_internalData);
}

void MacOpenGLWindow::set_allow_retina(bool allow) {
  Mac_setAllowRetina(m_internalData, allow);
}

#endif  //__APPLE__
#endif  // B3_USE_GLFW
