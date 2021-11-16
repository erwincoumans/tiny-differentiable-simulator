
#include "tiny_chrome_trace_util.h"
#include <assert.h>
#include <stdio.h>
#include <vector>
#include "tiny_clock.h"
#include "tiny_logging.h"

struct TinyTiming {
  const char* m_name;
  int m_threadId;
  unsigned long long int m_usStartTime;
  unsigned long long int m_usEndTime;
};

static FILE* gTimingFile = 0;
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif  //__STDC_FORMAT_MACROS

// see
// http://stackoverflow.com/questions/18107426/printf-format-for-unsigned-int64-on-windows
#ifndef _WIN32
#include <inttypes.h>
#endif

#define TINY_TIMING_CAPACITY 16 * 65536
static bool gFirstTiming = true;

struct TinyTimings {
  TinyTimings() : m_numTimings(0), m_activeBuffer(0) {}
  void flush() {
    for (int i = 0; i < m_numTimings; i++) {
      const char* name = m_timings[i].m_name;
      int threadId = m_timings[i].m_threadId;
      unsigned long long int startTime = m_timings[i].m_usStartTime;
      unsigned long long int endTime = m_timings[i].m_usEndTime;

      if (!gFirstTiming) {
        fprintf(gTimingFile, ",\n");
      }

      gFirstTiming = false;

      if (startTime > endTime) {
        endTime = startTime;
      }

      unsigned long long int startTimeDiv1000 = startTime / 1000;
      unsigned long long int endTimeDiv1000 = endTime / 1000;

#if 0

			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64 ".123 ,\"ph\":\"B\",\"name\":\"%s\",\"args\":{}},\n",
				threadId, startTimeDiv1000, name);
			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64 ".234 ,\"ph\":\"E\",\"name\":\"%s\",\"args\":{}}",
				threadId, endTimeDiv1000, name);

#else

      unsigned int startTimeRem1000 = startTime % 1000;
      unsigned int endTimeRem1000 = endTime % 1000;

      char startTimeRem1000Str[16];
      char endTimeRem1000Str[16];

      if (startTimeRem1000 < 10) {
        sprintf(startTimeRem1000Str, "00%d", startTimeRem1000);
      } else {
        if (startTimeRem1000 < 100) {
          sprintf(startTimeRem1000Str, "0%d", startTimeRem1000);
        } else {
          sprintf(startTimeRem1000Str, "%d", startTimeRem1000);
        }
      }

      if (endTimeRem1000 < 10) {
        sprintf(endTimeRem1000Str, "00%d", endTimeRem1000);
      } else {
        if (endTimeRem1000 < 100) {
          sprintf(endTimeRem1000Str, "0%d", endTimeRem1000);
        } else {
          sprintf(endTimeRem1000Str, "%d", endTimeRem1000);
        }
      }

      char newname[1024];
      static int counter2 = 0;
      sprintf(newname, "%s%d", name, counter2++);

#ifdef _WIN32

      fprintf(gTimingFile,
              "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%I64d.%s "
              ",\"ph\":\"B\",\"name\":\"%s\",\"args\":{}},\n",
              threadId, startTimeDiv1000, startTimeRem1000Str, newname);
      fprintf(gTimingFile,
              "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%I64d.%s "
              ",\"ph\":\"E\",\"name\":\"%s\",\"args\":{}}",
              threadId, endTimeDiv1000, endTimeRem1000Str, newname);

#else
      fprintf(gTimingFile,
              "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64
              ".%s ,\"ph\":\"B\",\"name\":\"%s\",\"args\":{}},\n",
              threadId, startTimeDiv1000, startTimeRem1000Str, newname);
      fprintf(gTimingFile,
              "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64
              ".%s ,\"ph\":\"E\",\"name\":\"%s\",\"args\":{}}",
              threadId, endTimeDiv1000, endTimeRem1000Str, newname);
#endif
#endif
    }
    m_numTimings = 0;
  }

  void addTiming(const char* name, int threadId,
                 unsigned long long int startTime,
                 unsigned long long int endTime) {
    if (m_numTimings >= TINY_TIMING_CAPACITY) {
      return;
    }

    if (m_timings.size() == 0) {
      m_timings.resize(TINY_TIMING_CAPACITY);
    }

    int slot = m_numTimings++;

    m_timings[slot].m_name = name;
    m_timings[slot].m_threadId = threadId;
    m_timings[slot].m_usStartTime = startTime;
    m_timings[slot].m_usEndTime = endTime;
  }

  int m_numTimings;
  int m_activeBuffer;
  std::vector<TinyTiming> m_timings;
};
//#ifndef BT_NO_PROFILE
TinyTimings gTimings[TINY_QUICKPROF_MAX_THREAD_COUNT];
#define MAX_NESTING 1024
int gStackDepths[TINY_QUICKPROF_MAX_THREAD_COUNT] = {0};
const char* gFuncNames[TINY_QUICKPROF_MAX_THREAD_COUNT][MAX_NESTING];
unsigned long long int gStartTimes[TINY_QUICKPROF_MAX_THREAD_COUNT]
                                  [MAX_NESTING];
//#endif

TinyClock clk;

static bool gProfileDisabled = true;

void MyDummyEnterProfileZoneFunc(const char* msg) {}

void MyDummyLeaveProfileZoneFunc() {}

// clang-format off
#if defined(_WIN32) && (defined(__MINGW32__) || defined(__MINGW64__))
#define BT_HAVE_TLS 1
#elif __APPLE__ 
  // TODO: Modern versions of iOS support TLS now with updated version checking.
#define BT_HAVE_TLS 1
#elif __linux__
#define BT_HAVE_TLS 1
#endif

// __thread is broken on Andorid clang until r12b. See
// https://github.com/android-ndk/ndk/issues/8
#if defined(__ANDROID__) && defined(__clang__)
#if __has_include(<android/ndk-version.h>)
#include <android/ndk-version.h>
#endif  // __has_include(<android/ndk-version.h>)
#if defined(__NDK_MAJOR__) && \
    ((__NDK_MAJOR__ < 12) || ((__NDK_MAJOR__ == 12) && (__NDK_MINOR__ < 1)))
#undef BT_HAVE_TLS
#endif
#endif  // defined(__ANDROID__) && defined(__clang__)
// clang-format on

unsigned int TinyGetCurrentThreadIndex2() {
  const unsigned int kNullIndex = ~0U;

#if BT_THREADSAFE
  return btGetCurrentThreadIndex();
#else
#if defined(BT_HAVE_TLS)
  static __thread unsigned int sThreadIndex = kNullIndex;
#elif defined(_WIN32)
  __declspec(thread) static unsigned int sThreadIndex = kNullIndex;
#else
  unsigned int sThreadIndex = 0;
  return -1;
#endif

  static int gThreadCounter = 0;

  if (sThreadIndex == kNullIndex) {
    sThreadIndex = gThreadCounter++;
  }
  return sThreadIndex;
#endif  // BT_THREADSAFE
}

void MyEnterProfileZoneFunc(const char* msg) {
  if (gProfileDisabled) return;

  int threadId = TinyGetCurrentThreadIndex2();
  if (threadId < 0 || threadId >= TINY_QUICKPROF_MAX_THREAD_COUNT) return;

  if (gStackDepths[threadId] >= MAX_NESTING) {
    assert(0);
    return;
  }
  gFuncNames[threadId][gStackDepths[threadId]] = msg;
  gStartTimes[threadId][gStackDepths[threadId]] = clk.get_time_nanoseconds();
  if (gStartTimes[threadId][gStackDepths[threadId]] <=
      gStartTimes[threadId][gStackDepths[threadId] - 1]) {
    gStartTimes[threadId][gStackDepths[threadId]] =
        1 + gStartTimes[threadId][gStackDepths[threadId] - 1];
  }
  gStackDepths[threadId]++;
}
void MyLeaveProfileZoneFunc() {
  if (gProfileDisabled) return;

  int threadId = TinyGetCurrentThreadIndex2();
  if (threadId < 0 || threadId >= TINY_QUICKPROF_MAX_THREAD_COUNT) return;

  if (gStackDepths[threadId] <= 0) {
    return;
  }

  gStackDepths[threadId]--;

  const char* name = gFuncNames[threadId][gStackDepths[threadId]];
  unsigned long long int startTime =
      gStartTimes[threadId][gStackDepths[threadId]];

  unsigned long long int endTime = clk.get_time_nanoseconds();
  gTimings[threadId].addTiming(name, threadId, startTime, endTime);
}

void TinyChromeUtilsStartTimings() {
  gFirstTiming = true;
  gProfileDisabled = false;
  TinySetCustomEnterProfileZoneFunc(MyEnterProfileZoneFunc);
  TinySetCustomLeaveProfileZoneFunc(MyLeaveProfileZoneFunc);
}

void TinyChromeUtilsStopTimingsAndWriteJsonFile(const char* fileNamePrefix) {
  TinySetCustomEnterProfileZoneFunc(MyDummyEnterProfileZoneFunc);
  TinySetCustomLeaveProfileZoneFunc(MyDummyLeaveProfileZoneFunc);
  char fileName[1024];
  static int fileCounter = 0;
  sprintf(fileName, "%s_%d.json", fileNamePrefix, fileCounter++);
  gTimingFile = fopen(fileName, "w");
  if (gTimingFile) {
    fprintf(gTimingFile, "{\"traceEvents\":[\n");
    // dump the content to file
    for (int i = 0; i < TINY_QUICKPROF_MAX_THREAD_COUNT; i++) {
      if (gTimings[i].m_numTimings) {
        printf("Writing %d timings for thread %d\n", gTimings[i].m_numTimings,
               i);
        gTimings[i].flush();
      }
    }
    fprintf(gTimingFile, "\n],\n\"displayTimeUnit\": \"ns\"}");
    fclose(gTimingFile);
  } else {
    TinyPrintf("Error opening file");
    TinyPrintf(fileName);
  }
  gProfileDisabled = true;
  gTimingFile = 0;
}

void TinyChromeUtilsEnableProfiling() { gProfileDisabled = false; }
