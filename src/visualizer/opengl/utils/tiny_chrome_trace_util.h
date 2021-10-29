
#ifndef TINY_CHROME_TRACE_UTIL_H
#define TINY_CHROME_TRACE_UTIL_H

#define TINY_QUICKPROF_MAX_THREAD_COUNT 256

void TinyChromeUtilsStartTimings();
void TinyChromeUtilsStopTimingsAndWriteJsonFile(const char* fileNamePrefix);
void TinyChromeUtilsEnableProfiling();
void MyEnterProfileZoneFunc(const char* msg);
void MyLeaveProfileZoneFunc();

#endif  // TINY_CHROME_TRACE_UTIL_H