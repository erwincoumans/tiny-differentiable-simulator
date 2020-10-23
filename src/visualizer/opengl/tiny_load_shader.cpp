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

#include "tiny_load_shader.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "tiny_opengl_include.h"

// Load the shader from the source text
void gltLoadShaderSrc(const char *szShaderSrc, GLuint shader) {
  GLchar *fsStringPtr[1];

  fsStringPtr[0] = (GLchar *)szShaderSrc;
  glShaderSource(shader, 1, (const GLchar **)fsStringPtr, NULL);
}

GLuint gltLoadShaderPair(const char *szVertexProg, const char *szFragmentProg) {
  assert(glGetError() == GL_NO_ERROR);

  // Temporary Shader objects
  GLuint hVertexShader;
  GLuint hFragmentShader;
  GLuint hReturn = 0;
  GLint testVal;

  // Create shader objects
  hVertexShader = glCreateShader(GL_VERTEX_SHADER);
  hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

  gltLoadShaderSrc(szVertexProg, hVertexShader);
  gltLoadShaderSrc(szFragmentProg, hFragmentShader);

  // Compile them
  glCompileShader(hVertexShader);
  assert(glGetError() == GL_NO_ERROR);

  glGetShaderiv(hVertexShader, GL_COMPILE_STATUS, &testVal);
  if (testVal == GL_FALSE) {
    char temp[256] = "";
    glGetShaderInfoLog(hVertexShader, 256, NULL, temp);
    fprintf(stderr, "Compile failed:\n%s\n", temp);
    assert(0);
    return 0;
    glDeleteShader(hVertexShader);
    glDeleteShader(hFragmentShader);
    return (GLuint)0;
  }

  assert(glGetError() == GL_NO_ERROR);

  glCompileShader(hFragmentShader);
  assert(glGetError() == GL_NO_ERROR);

  glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
  if (testVal == GL_FALSE) {
    char temp[256] = "";
    glGetShaderInfoLog(hFragmentShader, 256, NULL, temp);
    fprintf(stderr, "Compile failed:\n%s\n", temp);
    assert(0);
    exit(EXIT_FAILURE);
    glDeleteShader(hVertexShader);
    glDeleteShader(hFragmentShader);
    return (GLuint)0;
  }

  assert(glGetError() == GL_NO_ERROR);

  // Check for errors

  // Link them - assuming it works...
  hReturn = glCreateProgram();
  glAttachShader(hReturn, hVertexShader);
  glAttachShader(hReturn, hFragmentShader);

  glLinkProgram(hReturn);

  // These are no longer needed
  glDeleteShader(hVertexShader);
  glDeleteShader(hFragmentShader);

  // Make sure link worked too
  glGetProgramiv(hReturn, GL_LINK_STATUS, &testVal);
  if (testVal == GL_FALSE) {
    GLsizei maxLen = 4096;
    GLchar infoLog[4096];
    GLsizei actualLen;

    glGetProgramInfoLog(hReturn, maxLen, &actualLen, infoLog);

    printf("Warning/Error in GLSL shader:\n");
    printf("%s\n", infoLog);
    glDeleteProgram(hReturn);
    return (GLuint)0;
  }

  return hReturn;
}
