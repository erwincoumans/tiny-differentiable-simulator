/*
Copyright (c) 2013 Advanced Micro Devices, Inc.

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

#include "tiny_logging.h"

#include <stdarg.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#endif  //_WIN32

void TinyPrintfFuncDefault(const char* msg) {
#ifdef _WIN32
  OutputDebugStringA(msg);
#endif
  printf("%s", msg);
  // is this portable?
  fflush(stdout);
}

void TinyWarningMessageFuncDefault(const char* msg) {
#ifdef _WIN32
  OutputDebugStringA(msg);
#endif
  printf("%s", msg);
  // is this portable?
  fflush(stdout);
}

void TinyErrorMessageFuncDefault(const char* msg) {
#ifdef _WIN32
  OutputDebugStringA(msg);
#endif
  printf("%s", msg);

  // is this portable?
  fflush(stdout);
}

static TinyPrintfFunc* Tinys_printfFunc = TinyPrintfFuncDefault;
static TinyWarningMessageFunc* Tinys_warningMessageFunc =
    TinyWarningMessageFuncDefault;
static TinyErrorMessageFunc* Tinys_errorMessageFunc =
    TinyErrorMessageFuncDefault;

/// The developer can route TinyPrintf output using their own implementation
void TinySetCustomPrintfFunc(TinyPrintfFunc* printfFunc) {
  Tinys_printfFunc = printfFunc;
}
void TinySetCustomWarningMessageFunc(TinyPrintfFunc* warningMessageFunc) {
  Tinys_warningMessageFunc = warningMessageFunc;
}
void TinySetCustomErrorMessageFunc(TinyPrintfFunc* errorMessageFunc) {
  Tinys_errorMessageFunc = errorMessageFunc;
}

//#define B3_MAX_DEBUG_STRING_LENGTH 2048
#define B3_MAX_DEBUG_STRING_LENGTH 32768

void TinyOutputPrintfVarArgsInternal(const char* str, ...) {
  char strDebug[B3_MAX_DEBUG_STRING_LENGTH] = {0};
  va_list argList;
  va_start(argList, str);
#ifdef _MSC_VER
  vsprintf_s(strDebug, B3_MAX_DEBUG_STRING_LENGTH, str, argList);
#else
  vsnprintf(strDebug, B3_MAX_DEBUG_STRING_LENGTH, str, argList);
#endif
  (Tinys_printfFunc)(strDebug);
  va_end(argList);
}
void TinyOutputWarningMessageVarArgsInternal(const char* str, ...) {
  char strDebug[B3_MAX_DEBUG_STRING_LENGTH] = {0};
  va_list argList;
  va_start(argList, str);
#ifdef _MSC_VER
  vsprintf_s(strDebug, B3_MAX_DEBUG_STRING_LENGTH, str, argList);
#else
  vsnprintf(strDebug, B3_MAX_DEBUG_STRING_LENGTH, str, argList);
#endif
  (Tinys_warningMessageFunc)(strDebug);
  va_end(argList);
}
void TinyOutputErrorMessageVarArgsInternal(const char* str, ...) {
  char strDebug[B3_MAX_DEBUG_STRING_LENGTH] = {0};
  va_list argList;
  va_start(argList, str);
#ifdef _MSC_VER
  vsprintf_s(strDebug, B3_MAX_DEBUG_STRING_LENGTH, str, argList);
#else
  vsnprintf(strDebug, B3_MAX_DEBUG_STRING_LENGTH, str, argList);
#endif
  (Tinys_errorMessageFunc)(strDebug);
  va_end(argList);
}
#ifdef TINY_ENABLE_NVTX
#include "/usr/local/cuda-11.6/nsight-systems-2021.5.2/target-linux-x64/nvtx/include/nvtx3/nvToolsExt.h"
#endif

void TinyEnterProfileZoneDefault(const char* name) 
{
#ifdef TINY_ENABLE_NVTX
nvtxRangePushA(name);
#endif
}
void TinyLeaveProfileZoneDefault() 
{
#ifdef TINY_ENABLE_NVTX
nvtxRangePop();
#endif
}
static TinyEnterProfileZoneFunc* Tinys_enterFunc = TinyEnterProfileZoneDefault;
static TinyLeaveProfileZoneFunc* Tinys_leaveFunc = TinyLeaveProfileZoneDefault;
void TinyEnterProfileZone(const char* name) { (Tinys_enterFunc)(name); }
void TinyLeaveProfileZone() { (Tinys_leaveFunc)(); }

void TinySetCustomEnterProfileZoneFunc(TinyEnterProfileZoneFunc* enterFunc) {
  Tinys_enterFunc = enterFunc;
}
void TinySetCustomLeaveProfileZoneFunc(TinyLeaveProfileZoneFunc* leaveFunc) {
  Tinys_leaveFunc = leaveFunc;
}

#ifndef _MSC_VER
#undef vsprintf_s
#endif
