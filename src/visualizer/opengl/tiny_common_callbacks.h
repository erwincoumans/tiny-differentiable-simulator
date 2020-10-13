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

#ifndef TINY_COMMON_CALLBACKS_H
#define TINY_COMMON_CALLBACKS_H

typedef void (*TinyWheelCallback)(float deltax, float deltay);
typedef void (*TinyResizeCallback)(float width, float height);
typedef void (*TinyMouseMoveCallback)(float x, float y);
typedef void (*TinyMouseButtonCallback)(int button, int state, float x,
                                        float y);
typedef void (*TinyKeyboardCallback)(int keycode, int state);
typedef void (*TinyRenderCallback)();

enum {
  TINY_KEY_ESCAPE = 27,
  TINY_KEY_SPACE = 32,
  TINY_KEY_F1 = 128,
  TINY_KEY_F2,
  TINY_KEY_F3,
  TINY_KEY_F4,
  TINY_KEY_F5,
  TINY_KEY_F6,
  TINY_KEY_F7,
  TINY_KEY_F8,
  TINY_KEY_F9,
  TINY_KEY_F10,
  TINY_KEY_F11,
  TINY_KEY_F12,
  TINY_KEY_F13,
  TINY_KEY_F14,
  TINY_KEY_F15,
  TINY_KEY_LEFT_ARROW,
  TINY_KEY_RIGHT_ARROW,
  TINY_KEY_UP_ARROW,
  TINY_KEY_DOWN_ARROW,
  TINY_KEY_PAGE_UP,
  TINY_KEY_PAGE_DOWN,
  TINY_KEY_END,
  TINY_KEY_HOME,
  TINY_KEY_INSERT,
  TINY_KEY_DELETE,
  TINY_KEY_BACKSPACE,
  TINY_KEY_SHIFT,
  TINY_KEY_CONTROL,
  TINY_KEY_ALT,
  TINY_KEY_RETURN,
  TINY_KEY_TAB,

};

#endif  // TINY_COMMON_CALLBACKS_H
