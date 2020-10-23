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

#ifndef TINY_COMMON_GRAPHICS_APP_INTERFACE_H
#define TINY_COMMON_GRAPHICS_APP_INTERFACE_H

#include "tiny_camera.h"
#include "math/tiny/tiny_float_utils.h"
#include "tiny_gl_instancing_renderer.h"
#include "tiny_window_interface.h"

struct DrawGridData {
  int gridSize;
  float upOffset;
  int upAxis;
  bool drawAxis;
  float gridColor[4];

  DrawGridData(int upAx = 2) : gridSize(10), upOffset(0.001f), drawAxis(false), upAxis(upAx) {
    gridColor[0] = 0.6f;
    gridColor[1] = 0.6f;
    gridColor[2] = 0.6f;
    gridColor[3] = 1.f;
  }
  void set_color(float red, float green, float blue, float alpha)
  {
      gridColor[0] = red;
      gridColor[1] = green;
      gridColor[2] = blue;
      gridColor[3] = alpha;
  }
};

enum EnumSphereLevelOfDetail {
  SPHERE_LOD_POINT_SPRITE = 0,
  SPHERE_LOD_LOW,
  SPHERE_LOD_MEDIUM,
  SPHERE_LOD_HIGH,
};
struct TinyCommonGraphicsApp {
  enum drawText3DOption {
    eDrawText3D_OrtogonalFaceCamera = 1,
    eDrawText3D_TrueType = 2,
    eDrawText3D_TrackObject = 4,
  };
  class TinyWindowInterface* m_window;
  class TinyGLInstancingRenderer* m_renderer;
  struct CommonParameterInterface* m_parameterInterface;
  struct Common2dCanvasInterface* m_2dCanvasInterface;

  bool m_leftMouseButton;
  bool m_middleMouseButton;
  bool m_rightMouseButton;
  float m_wheelMultiplier;
  float m_mouseMoveMultiplier;
  float m_mouseXpos;
  float m_mouseYpos;
  bool m_mouseInitialized;
  float m_backgroundColorRGB[3];

  TinyCommonGraphicsApp()
      : m_window(0),
        m_renderer(0),
        m_parameterInterface(0),
        m_2dCanvasInterface(0),
        m_leftMouseButton(false),
        m_middleMouseButton(false),
        m_rightMouseButton(false),
        m_wheelMultiplier(0.01f),
        m_mouseMoveMultiplier(0.4f),
        m_mouseXpos(0.f),
        m_mouseYpos(0.f),
        m_mouseInitialized(false) {
    m_backgroundColorRGB[0] = 0.95;
    m_backgroundColorRGB[1] = 0.95;
    m_backgroundColorRGB[2] = 0.95;
  }
  virtual ~TinyCommonGraphicsApp() {}

  virtual void dump_next_frame_to_png(const char* pngFilename) {}
  virtual void dump_frames_to_video(const char* mp4Filename) {}

  virtual void get_screen_pixels(unsigned char* rgbaBuffer,
                                 int bufferSizeInBytes, float* depthBuffer,
                                 int depthBufferSizeInBytes) {}
  virtual void set_viewport(int width, int height) {}

  virtual void getBackgroundColor(float* red, float* green, float* blue) const {
    if (red) *red = m_backgroundColorRGB[0];
    if (green) *green = m_backgroundColorRGB[1];
    if (blue) *blue = m_backgroundColorRGB[2];
  }
  virtual void set_mp4_fps(int fps) {}
  virtual void set_background_color(float red, float green, float blue) {
    m_backgroundColorRGB[0] = red;
    m_backgroundColorRGB[1] = green;
    m_backgroundColorRGB[2] = blue;
  }
  virtual void setMouseWheelMultiplier(float mult) { m_wheelMultiplier = mult; }
  virtual float getMouseWheelMultiplier() const { return m_wheelMultiplier; }

  virtual void setMouseMoveMultiplier(float mult) {
    m_mouseMoveMultiplier = mult;
  }

  virtual float getMouseMoveMultiplier() const { return m_mouseMoveMultiplier; }

  virtual void draw_grid(DrawGridData data = DrawGridData()) = 0;
  virtual void set_up_axis(int axis) = 0;
  virtual int get_up_axis() const = 0;

  virtual void swap_buffer() = 0;
  virtual void draw_text(const char* txt, int posX, int posY) {
    float size = 1;
    float colorRGBA[4] = {0, 0, 0, 1};
    draw_text(txt, posX, posY, size, colorRGBA);
  }

  virtual void draw_text(const char* txt, int posX, int posY, float size) {
    float colorRGBA[4] = {0, 0, 0, 1};
    draw_text(txt, posX, posY, size, colorRGBA);
  }
  virtual void draw_text(const char* txt, int posX, int posY, float size,
                         float colorRGBA[4]) = 0;
  virtual void draw_text_3d(const char* txt, float posX, float posY, float posZ,
                            float size) = 0;
  virtual void draw_text_3d(const char* txt, float position[3],
                            float orientation[4], float color[4], float size,
                            int optionFlag) = 0;
  virtual void draw_textured_tect(float x0, float y0, float x1, float y1,
                                  float color[4], float u0, float v0, float u1,
                                  float v1, int useRGBA) = 0;
  virtual int register_cube_shape(float halfExtentsX, float halfExtentsY,
                                  float halfExtentsZ, int textureIndex = -1,
                                  float textureScaling = 1) = 0;
  virtual int register_graphics_unit_sphere_shape(EnumSphereLevelOfDetail lod,
                                                  int textureId = -1) = 0;

  virtual void register_grid(int xres, int yres, const ::TINY::TinyVector3f& color0,
                             const ::TINY::TinyVector3f& color1) = 0;

  void defaultMouseButtonCallback(int button, int state, float x, float y) {
    if (button == 0) m_leftMouseButton = (state == 1);
    if (button == 1) m_middleMouseButton = (state == 1);

    if (button == 2) m_rightMouseButton = (state == 1);

    m_mouseXpos = x;
    m_mouseYpos = y;
    m_mouseInitialized = true;
  }
  void defaultMouseMoveCallback(float x, float y) {
    if (m_window && m_renderer) {
      TinyCamera* camera = m_renderer->get_active_camera();

      bool isAltPressed = m_window->is_modifier_key_pressed(TINY_KEY_ALT);
      bool isControlPressed =
          m_window->is_modifier_key_pressed(TINY_KEY_CONTROL);

      if (isAltPressed || isControlPressed) {
        float xDelta = x - m_mouseXpos;
        float yDelta = y - m_mouseYpos;
        float cameraDistance = camera->get_camera_distance();
        float pitch = camera->get_camera_pitch();
        float yaw = camera->get_camera_yaw();

        ::TINY::TinyVector3f targPos;
        ::TINY::TinyVector3f camPos;

        camera->get_camera_target_position(targPos);
        camera->get_camera_position(camPos);

        ::TINY::TinyVector3f cameraPosition = camPos;

        ::TINY::TinyVector3f cameraTargetPosition = targPos;

        ::TINY::TinyVector3f cameraUp = ::TINY::TinyVector3f(0, 0, 0);
        cameraUp[camera->get_camera_up_axis()] = 1.f;

        if (m_leftMouseButton) {
          //			if (b3Fabs(xDelta)>b3Fabs(yDelta))
          //			{
          pitch -= yDelta * m_mouseMoveMultiplier;
          //			} else
          //			{
          yaw -= xDelta * m_mouseMoveMultiplier;
          //			}
        }

        if (m_middleMouseButton) {
          cameraTargetPosition +=
              cameraUp * yDelta * m_mouseMoveMultiplier * 0.01f;

          ::TINY::TinyVector3f fwd = cameraTargetPosition - cameraPosition;
          ::TINY::TinyVector3f side = cameraUp.cross(fwd);
          side.normalize();
          cameraTargetPosition += side * xDelta * m_mouseMoveMultiplier * 0.01f;
        }
        if (m_rightMouseButton) {
          cameraDistance -= xDelta * m_mouseMoveMultiplier * 0.01f;
          cameraDistance -= yDelta * m_mouseMoveMultiplier * 0.01f;
          if (cameraDistance < 1) cameraDistance = 1;
          if (cameraDistance > 1000) cameraDistance = 1000;
        }
        camera->set_camera_distance(cameraDistance);
        camera->set_camera_pitch(pitch);
        camera->set_camera_yaw(yaw);
        camera->set_camera_target_position(cameraTargetPosition[0],
                                           cameraTargetPosition[1],
                                           cameraTargetPosition[2]);
      }

    }  // m_window &&  m_renderer

    m_mouseXpos = x;
    m_mouseYpos = y;
    m_mouseInitialized = true;
  }
  //	void defaultKeyboardCallback(int key, int state)
  //	{
  //	}
  void defaultWheelCallback(float deltax, float deltay) {
    if (m_renderer) {
      ::TINY::TinyVector3f cameraTargetPosition, cameraPosition,
          cameraUp = ::TINY::TinyVector3f(0, 0, 0);
      cameraUp[get_up_axis()] = 1;
      TinyCamera* camera = m_renderer->get_active_camera();

      camera->get_camera_position(cameraPosition);
      camera->get_camera_target_position(cameraTargetPosition);

      if (!m_leftMouseButton) {
        float cameraDistance = camera->get_camera_distance();
        if (deltay < 0 || cameraDistance > 1) {
          cameraDistance -= deltay * m_wheelMultiplier;
          if (cameraDistance < 1) cameraDistance = 1;
          camera->set_camera_distance(cameraDistance);
        } else {
          ::TINY::TinyVector3f fwd = cameraTargetPosition - cameraPosition;
          fwd.normalize();
          cameraTargetPosition +=
              fwd * deltay * m_wheelMultiplier;  // todo: expose it in the GUI?
        }
      } else {
        if (::TINY::FloatUtils::abs(deltax) > ::TINY::FloatUtils::abs(deltay)) {
          ::TINY::TinyVector3f fwd = cameraTargetPosition - cameraPosition;
          ::TINY::TinyVector3f side = cameraUp.cross(fwd);
          side.normalize();
          cameraTargetPosition += side * deltax * m_wheelMultiplier;
        } else {
          cameraTargetPosition -= cameraUp * deltay * m_wheelMultiplier;
        }
      }

      camera->set_camera_target_position(cameraTargetPosition[0],
                                         cameraTargetPosition[1],
                                         cameraTargetPosition[2]);
    }
  }
};

#endif  // TINY_COMMON_GRAPHICS_APP_INTERFACE_H
