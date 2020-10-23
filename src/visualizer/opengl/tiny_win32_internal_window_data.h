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

#ifndef TINY_WIN32_INTERNAL_WINDOW_DATA_H
#define TINY_WIN32_INTERNAL_WINDOW_DATA_H

#include <windows.h>

struct TinyWin32InternalData2 {
  HWND m_hWnd;
  ;
  int m_fullWindowWidth;  // includes borders etc
  int m_fullWindowHeight;

  int m_openglViewportWidth;  // just the 3d viewport/client area
  int m_openglViewportHeight;

  HDC m_hDC;
  HGLRC m_hRC;
  bool m_OpenGLInitialized;
  int m_oldScreenWidth;
  int m_oldHeight;
  int m_oldBitsPerPel;
  bool m_quit;
  int m_mouseLButton;
  int m_mouseRButton;
  int m_mouseMButton;
  int m_mouseXpos;
  int m_mouseYpos;

  int m_internalKeyModifierFlags;

  TinyWheelCallback m_wheelCallback;
  TinyMouseMoveCallback m_mouseMoveCallback;
  TinyMouseButtonCallback m_mouseButtonCallback;
  TinyResizeCallback m_resizeCallback;
  TinyKeyboardCallback m_keyboardCallback;

  TinyWin32InternalData2() {
    m_hWnd = 0;
    m_mouseLButton = 0;
    m_mouseRButton = 0;
    m_mouseMButton = 0;
    m_internalKeyModifierFlags = 0;
    m_fullWindowWidth = 0;
    m_fullWindowHeight = 0;
    m_openglViewportHeight = 0;
    m_openglViewportWidth = 0;
    m_hDC = 0;
    m_hRC = 0;
    m_OpenGLInitialized = false;
    m_oldScreenWidth = 0;
    m_oldHeight = 0;
    m_oldBitsPerPel = 0;
    m_quit = false;

    m_keyboardCallback = 0;
    m_mouseMoveCallback = 0;
    m_mouseButtonCallback = 0;
    m_resizeCallback = 0;
    m_wheelCallback = 0;
  }
};

#endif  // TINY_WIN32_INTERNAL_WINDOW_DATA_H