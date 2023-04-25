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

#include "tiny_camera.h"

#include "math/tiny/tiny_float_utils.h"

using namespace TINY;

struct TinyCameraInternalData {
  
  TinyCameraInternalData()
      : m_cameraTargetPosition(TinyVector3f(0, 0, 0)),
        m_cameraDistance(20),
        m_cameraUp(TinyVector3f(0, 1, 0)),
        m_cameraForward(TinyVector3f(1, 0, 0)),
        m_cameraUpAxis(1),
        m_yaw(20),
        m_pitch(0),
        m_aspect(1),
        m_frustumZNear(0.01),
        m_frustumZFar(1000) {}

  TinyVector3f m_cameraTargetPosition;
  float m_cameraDistance;
  TinyVector3f m_cameraUp;
  TinyVector3f m_cameraForward;
  int m_cameraUpAxis;
  // the m_cameraPosition is a cached value, recomputed from other values
  TinyVector3f m_cameraPosition;
  float m_yaw;

  float m_pitch;
  float m_aspect;
  float m_frustumZNear;
  float m_frustumZFar;

  float m_viewMatrixVR[16];
  float m_projectionMatrixVR[16];
};

  TinyCamera::TinyCamera(const TinyCamera& other)
  {
    m_data = new TinyCameraInternalData(*other.m_data);
  }

TinyCamera::TinyCamera() { 
    m_data = new TinyCameraInternalData; 
}
TinyCamera::~TinyCamera() { 
    delete m_data; 
}




static void b3CreateFrustum(float left, float right, float bottom, float top,
                            float nearVal, float farVal, float frustum[16]) {
  frustum[0 * 4 + 0] = (float(2) * nearVal) / (right - left);
  frustum[0 * 4 + 1] = float(0);
  frustum[0 * 4 + 2] = float(0);
  frustum[0 * 4 + 3] = float(0);

  frustum[1 * 4 + 0] = float(0);
  frustum[1 * 4 + 1] = (float(2) * nearVal) / (top - bottom);
  frustum[1 * 4 + 2] = float(0);
  frustum[1 * 4 + 3] = float(0);

  frustum[2 * 4 + 0] = (right + left) / (right - left);
  frustum[2 * 4 + 1] = (top + bottom) / (top - bottom);
  frustum[2 * 4 + 2] = -(farVal + nearVal) / (farVal - nearVal);
  frustum[2 * 4 + 3] = float(-1);

  frustum[3 * 4 + 0] = float(0);
  frustum[3 * 4 + 1] = float(0);
  frustum[3 * 4 + 2] = -(float(2) * farVal * nearVal) / (farVal - nearVal);
  frustum[3 * 4 + 3] = float(0);
}

#if 0
static void b3CreateDiagonalMatrix(float value, float result[4][4])
{
	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			if (i==j)
			{
				result[i][j] = value;
			} else
			{
				result[i][j] = 0.f;
			}
		}
	}
}
static void b3CreateOrtho(float left, float right, float bottom, float top, float zNear, float zFar, float result[4][4])
{
	b3CreateDiagonalMatrix(1.f,result);

	result[0][0] = 2.f / (right - left);
	result[1][1] = 2.f / (top - bottom);
	result[2][2] = - 2.f / (zFar - zNear);
	result[3][0] = - (right + left) / (right - left);
	result[3][1] = - (top + bottom) / (top - bottom);
	result[3][2] = - (zFar + zNear) / (zFar - zNear);
}
#endif
static void b3CreateLookAt(const TinyVector3f& eye, const TinyVector3f& center,
                           const TinyVector3f& up, float result[16]) {
  TinyVector3f f = (center - eye).normalized();
  TinyVector3f u = up.normalized();
  TinyVector3f s = (f.cross(u)).normalized();
  u = s.cross(f);

  result[0 * 4 + 0] = s.x();
  result[1 * 4 + 0] = s.y();
  result[2 * 4 + 0] = s.z();

  result[0 * 4 + 1] = u.x();
  result[1 * 4 + 1] = u.y();
  result[2 * 4 + 1] = u.z();

  result[0 * 4 + 2] = -f.x();
  result[1 * 4 + 2] = -f.y();
  result[2 * 4 + 2] = -f.z();

  result[0 * 4 + 3] = 0.f;
  result[1 * 4 + 3] = 0.f;
  result[2 * 4 + 3] = 0.f;

  result[3 * 4 + 0] = -s.dot(eye);
  result[3 * 4 + 1] = -u.dot(eye);
  result[3 * 4 + 2] = f.dot(eye);
  result[3 * 4 + 3] = 1.f;
}

void TinyCamera::set_camera_up_axis(int upAxis) {
  m_data->m_cameraUpAxis = upAxis;

  update();
}

int TinyCamera::get_camera_up_axis() const { return m_data->m_cameraUpAxis; }

void TinyCamera::update() {
  float yawRad = m_data->m_yaw * float(0.01745329251994329547);  // rads per deg
  float pitchRad =
      m_data->m_pitch * float(0.01745329251994329547);  // rads per deg
  float rollRad = 0.0;
  TinyQuaternionf eyeRot;

  int forwardAxis(-1);
  switch (m_data->m_cameraUpAxis) {
    case 1:
      forwardAxis = 2;
      m_data->m_cameraUp = TinyVector3f(0, 1, 0);
      // gLightPos = TinyVector3f(-50.f,100,30);
      eyeRot.set_euler_rpy(TinyVector3f(-pitchRad, yawRad,0));
      break;
    case 2:
      forwardAxis = 1;
      m_data->m_cameraUp = TinyVector3f(0, 0, 1);
      // gLightPos = TinyVector3f(-50.f,30,100);
      eyeRot.set_euler_rpy(TinyVector3f(pitchRad, 0, yawRad));
      break;
    default: {
      // b3Assert(0);
      return;
    }
  };

  TinyVector3f eyePos = TinyVector3f(0, 0, 0);
  eyePos[forwardAxis] = -m_data->m_cameraDistance;
  eyePos = TinyMatrix3x3f(eyeRot) * eyePos;

  m_data->m_cameraPosition = eyePos;

  m_data->m_cameraPosition += m_data->m_cameraTargetPosition;

  m_data->m_cameraForward =
      m_data->m_cameraTargetPosition - m_data->m_cameraPosition;
  m_data->m_cameraForward.normalize();
}

void TinyCamera::get_camera_projection_matrix(
    float projectionMatrix[16]) const {
  b3CreateFrustum(-m_data->m_aspect * m_data->m_frustumZNear,
                  m_data->m_aspect * m_data->m_frustumZNear,
                  -m_data->m_frustumZNear, m_data->m_frustumZNear,
                  m_data->m_frustumZNear, m_data->m_frustumZFar,
                  projectionMatrix);
}

void TinyCamera::get_camera_view_matrix(float viewMatrix[16]) const {
  b3CreateLookAt(m_data->m_cameraPosition, m_data->m_cameraTargetPosition,
                 m_data->m_cameraUp, viewMatrix);
}

void TinyCamera::get_camera_target_position(double pos[3]) const {
  pos[0] = m_data->m_cameraTargetPosition[0];
  pos[1] = m_data->m_cameraTargetPosition[1];
  pos[2] = m_data->m_cameraTargetPosition[2];
}

void TinyCamera::get_camera_target_position(TinyVector3f& pos) const {
  pos[0] = m_data->m_cameraTargetPosition[0];
  pos[1] = m_data->m_cameraTargetPosition[1];
  pos[2] = m_data->m_cameraTargetPosition[2];
}
void TinyCamera::get_camera_position(TinyVector3f& pos) const {
  pos[0] = m_data->m_cameraPosition[0];
  pos[1] = m_data->m_cameraPosition[1];
  pos[2] = m_data->m_cameraPosition[2];
}

void TinyCamera::set_camera_target_position(float x, float y, float z) {
  m_data->m_cameraTargetPosition.setValue(x, y, z);
  update();
}
float TinyCamera::get_camera_distance() const {
  return m_data->m_cameraDistance;
}

void TinyCamera::set_camera_distance(float dist) {
  m_data->m_cameraDistance = dist;
  update();
}
void TinyCamera::set_camera_up_vector(float x, float y, float z) {
  m_data->m_cameraUp.setValue(x, y, z);
  update();
}

void TinyCamera::get_camera_up_vector(float up[3]) const {
  up[0] = float(m_data->m_cameraUp[0]);
  up[1] = float(m_data->m_cameraUp[1]);
  up[2] = float(m_data->m_cameraUp[2]);
}

void TinyCamera::get_camera_forward_vector(float fwd[3]) const {
  fwd[0] = float(m_data->m_cameraForward[0]);
  fwd[1] = float(m_data->m_cameraForward[1]);
  fwd[2] = float(m_data->m_cameraForward[2]);
}

void TinyCamera::set_camera_yaw(float yaw) {
  m_data->m_yaw = yaw;
  update();
}

float TinyCamera::get_camera_yaw() const { return m_data->m_yaw; }

void TinyCamera::set_camera_pitch(float pitch) {
  m_data->m_pitch = pitch;
  update();
}

void TinyCamera::set_aspect_ratio(float ratio) {
  m_data->m_aspect = ratio;
  update();
}

float TinyCamera::get_camera_pitch() const { return m_data->m_pitch; }
float TinyCamera::get_aspect_ratio() const { return m_data->m_aspect; }

float TinyCamera::get_camera_frustum_far() const {
  return m_data->m_frustumZFar;
}

float TinyCamera::get_camera_frustum_near() const {
  return m_data->m_frustumZNear;
}

void TinyCamera::set_camera_frustum_far(float far) {
  m_data->m_frustumZFar = far;
}

void TinyCamera::set_camera_frustum_near(float near) {
  m_data->m_frustumZNear = near;
}

void TinyCamera::copy_data(const TinyCamera& other) {
  *this->m_data = *other.m_data;
}
