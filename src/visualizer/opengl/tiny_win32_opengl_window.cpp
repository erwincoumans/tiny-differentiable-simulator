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

typedef HGLRC WINAPI wglCreateContextAttribsARB_type(HDC hdc, HGLRC hShareContext,
	const int* attribList);
wglCreateContextAttribsARB_type* wglCreateContextAttribsARB = 0;

// See https://www.opengl.org/registry/specs/ARB/wgl_create_context.txt for all values
#define WGL_CONTEXT_MAJOR_VERSION_ARB             0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB             0x2092
#define WGL_CONTEXT_PROFILE_MASK_ARB              0x9126
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB          0x00000001

typedef BOOL WINAPI wglChoosePixelFormatARB_type(HDC hdc, const int* piAttribIList,
    const FLOAT* pfAttribFList, UINT nMaxFormats, int* piFormats, UINT* nNumFormats);
wglChoosePixelFormatARB_type* wglChoosePixelFormatARB = 0;

void fatal_error(const char* msg)
{
    printf("Error:%s\n", msg);
    exit(0);
}

static void
init_opengl_extensions(void)
{
    // Before we can load extensions, we need a dummy OpenGL context, created using a dummy window.
    // We use a dummy window because you can only set the pixel format for a window once. For the
    // real window, we want to use wglChoosePixelFormatARB (so we can potentially specify options
    // that aren't available in PIXELFORMATDESCRIPTOR), but we can't load and use that before we
    // have a context.
    WNDCLASSA window_class;
    ZeroMemory(&window_class, sizeof(WNDCLASSA));

    window_class.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    window_class.lpfnWndProc = DefWindowProcA;
    window_class.hInstance = GetModuleHandle(0);
    window_class.lpszClassName = "Dummy_WGL_djuasiodwa";

    if (!RegisterClassA(&window_class)) {
        fatal_error("Failed to register dummy OpenGL window.");
    }

    HWND dummy_window = CreateWindowExA(
        0,
        window_class.lpszClassName,
        "Dummy OpenGL Window",
        0,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        0,
        0,
        window_class.hInstance,
        0);

    if (!dummy_window) {
        fatal_error("Failed to create dummy OpenGL window.");
    }

    HDC dummy_dc = GetDC(dummy_window);

    PIXELFORMATDESCRIPTOR pfd;
    ZeroMemory(&pfd, sizeof(PIXELFORMATDESCRIPTOR));
    
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.cColorBits = 32;
    pfd.cAlphaBits = 8;
    pfd.iLayerType = PFD_MAIN_PLANE;
    pfd.cDepthBits = 24;
    pfd.cStencilBits = 8;
    

    int pixel_format = ChoosePixelFormat(dummy_dc, &pfd);
    if (!pixel_format) {
        fatal_error("Failed to find a suitable pixel format.");
    }
    if (!SetPixelFormat(dummy_dc, pixel_format, &pfd)) {
        fatal_error("Failed to set the pixel format.");
    }

    HGLRC dummy_context = wglCreateContext(dummy_dc);
    if (!dummy_context) {
        fatal_error("Failed to create a dummy OpenGL rendering context.");
    }

    if (!wglMakeCurrent(dummy_dc, dummy_context)) {
        fatal_error("Failed to activate dummy OpenGL rendering context.");
    }

    wglCreateContextAttribsARB = (wglCreateContextAttribsARB_type*)wglGetProcAddress(
        "wglCreateContextAttribsARB");
    wglChoosePixelFormatARB = (wglChoosePixelFormatARB_type*)wglGetProcAddress(
        "wglChoosePixelFormatARB");

    wglMakeCurrent(dummy_dc, 0);
    wglDeleteContext(dummy_context);
    ReleaseDC(dummy_window, dummy_dc);
    DestroyWindow(dummy_window);
}

void TinyWin32OpenGLWindow::enableOpenGL() {

  //init_opengl_extensions();

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


  // Specify that we want to create an OpenGL 3.3 core profile context
  int gl33_attribs[] = {
	  WGL_CONTEXT_MAJOR_VERSION_ARB, 3,
	  WGL_CONTEXT_MINOR_VERSION_ARB, 3,
	  WGL_CONTEXT_PROFILE_MASK_ARB,  WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
	  0,
  };

  
  if (wglCreateContextAttribsARB)
  {
	  m_data->m_hRC = wglCreateContextAttribsARB(m_data->m_hDC, 0, gl33_attribs);
	  if (!m_data->m_hRC) {
		  printf("Failed to create OpenGL 3.3 context.");
		  exit(0);
	  }
  }
  else
  {
	  m_data->m_hRC = wglCreateContext(m_data->m_hDC);
  }

  // create and enable the render context (RC)
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
