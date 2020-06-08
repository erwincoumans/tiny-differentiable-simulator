#ifndef B3_USE_GLFW
#ifdef _WIN32
/*
Copyright (c) 2012 Advanced Micro Devices, Inc.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software. Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
// Originally written by Erwin Coumans

#include "tiny_win32_opengl_window.h"

#include "tiny_opengl_include.h"

//#include "Bullet3Common/b3Vector3.h"

#include <stdio.h>
#include <stdlib.h>
#include "tiny_win32_internal_window_data.h"

void TinyWin32OpenGLWindow::enableOpenGL() {
  PIXELFORMATDESCRIPTOR pfd;
  int format;

  // get the device context (DC)
  m_data->m_hDC = GetDC(m_data->m_hWnd);

  // set the pixel format for the DC
  ZeroMemory(&pfd, sizeof(pfd));
  pfd.nSize = sizeof(pfd);
  pfd.nVersion = 1;
  pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
  pfd.iPixelType = PFD_TYPE_RGBA;
  pfd.cColorBits = 32;
  pfd.cRedBits = 8;
  pfd.cGreenBits = 8;
  pfd.cBlueBits = 8;
  pfd.cAlphaBits = 8;

  pfd.cDepthBits = 24;
  pfd.cStencilBits = 8;  // 1;
  pfd.iLayerType = PFD_MAIN_PLANE;
  format = ChoosePixelFormat(m_data->m_hDC, &pfd);
  SetPixelFormat(m_data->m_hDC, format, &pfd);

  // create and enable the render context (RC)
  m_data->m_hRC = wglCreateContext(m_data->m_hDC);
  wglMakeCurrent(m_data->m_hDC, m_data->m_hRC);

  // printGLString("Extensions", GL_EXTENSIONS);
}

void TinyWin32OpenGLWindow::disableOpenGL() {
  wglMakeCurrent(NULL, NULL);
  wglDeleteContext(m_data->m_hRC);
  //	ReleaseDC( m_data->m_hWnd, m_data->m_hDC );
}

void TinyWin32OpenGLWindow::create_window(
    const TinyWindowConstructionInfo& ci) {
  TinyWin32Window::create_window(ci);

  // VideoDriver = video::createOpenGLDriver(CreationParams, FileSystem, this);
  enableOpenGL();

  if (!gladLoaderLoadGL()) {
    printf("gladLoaderLoadGL failed!\n");
    exit(-1);
  }
}

TinyWin32OpenGLWindow::TinyWin32OpenGLWindow() {}

TinyWin32OpenGLWindow::~TinyWin32OpenGLWindow() {}

void TinyWin32OpenGLWindow::close_window() {
  disableOpenGL();

  TinyWin32Window::close_window();
}

void TinyWin32OpenGLWindow::start_rendering() {
  pumpMessage();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // glCullFace(GL_BACK);
  // glFrontFace(GL_CCW);
  glEnable(GL_DEPTH_TEST);
}

void TinyWin32OpenGLWindow::renderAllObjects() {}

void TinyWin32OpenGLWindow::end_rendering() { SwapBuffers(m_data->m_hDC); }

int TinyWin32OpenGLWindow::file_open_dialog(char* fileName,
                                            int maxFileNameLength) {
#if 0
	//wchar_t wideChars[1024];

		OPENFILENAME ofn ;
	ZeroMemory( &ofn , sizeof( ofn));
	ofn.lStructSize = sizeof ( ofn );
	ofn.hwndOwner = NULL  ;

#ifdef UNICODE
	WCHAR bla[1024];
	ofn.lpstrFile = bla;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = 1023;
	ofn.lpstrFilter = L"All Files\0*.*\0URDF\0*.urdf\0.bullet\0*.bullet\0";
#else
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = 1023;
	//ofn.lpstrFilter = "All\0*.*\0Text\0*.TXT\0";
	ofn.lpstrFilter = "All Files\0*.*\0URDF\0*.urdf\0.bullet\0*.bullet\0";

#endif

	ofn.nFilterIndex =1;
	ofn.lpstrFileTitle = NULL ;
	ofn.nMaxFileTitle = 0 ;
	ofn.lpstrInitialDir=NULL ;
	ofn.Flags = OFN_PATHMUSTEXIST|OFN_FILEMUSTEXIST ;
	GetOpenFileName( &ofn );
	return strlen(fileName);
#else
  return 0;
#endif
}

int TinyWin32OpenGLWindow::get_width() const {
  if (m_data) return m_data->m_openglViewportWidth;
  return 0;
}

int TinyWin32OpenGLWindow::get_height() const {
  if (m_data) return m_data->m_openglViewportHeight;
  return 0;
}

#endif

#endif  // B3_USE_GLFW
