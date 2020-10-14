

#include "tiny_clock.h"
#include <assert.h>

#ifdef __APPLE__
#include <TargetConditionals.h>
#include <mach/mach_time.h>
#endif

#if defined(WIN32) || defined(_WIN32)

#define BT_USE_WINDOWS_TIMERS
#define WIN32_LEAN_AND_MEAN
#define NOWINRES
#define NOMCX
#define NOIME

#ifdef _XBOX
#include <Xtl.h>
#else  //_XBOX
#include <windows.h>

#if WINVER < 0x0602
#define GetTickCount64 GetTickCount
#endif

#endif  //_XBOX

#include <time.h>

#else  //_WIN32
#include <sys/time.h>
#include <unistd.h>

#ifdef BT_LINUX_REALTIME
// required linking against rt (librt)
#include <time.h>
#endif  // BT_LINUX_REALTIME

#endif  //_WIN32

#define mymin(a, b) (a > b ? a : b)

struct TinyClockData {
#ifdef BT_USE_WINDOWS_TIMERS
  LARGE_INTEGER mClockFrequency;
  LONGLONG mStartTick;
  LARGE_INTEGER mStartTime;
#else
#ifdef __APPLE__
  uint64_t mStartTimeNano;
#endif
  struct timeval mStartTime;
#endif
};

/// The TinyClock is a portable basic clock that measures accurate time in
/// seconds, use for profiling.
TinyClock::TinyClock() {
  m_data = new TinyClockData;
#ifdef BT_USE_WINDOWS_TIMERS
  QueryPerformanceFrequency(&m_data->mClockFrequency);
#endif
  reset();
}

TinyClock::~TinyClock() { delete m_data; }

TinyClock::TinyClock(const TinyClock& other) {
  m_data = new TinyClockData;
  *m_data = *other.m_data;
}

TinyClock& TinyClock::operator=(const TinyClock& other) {
  *m_data = *other.m_data;
  return *this;
}

/// Resets the initial reference time.
void TinyClock::reset() {
#ifdef BT_USE_WINDOWS_TIMERS
  QueryPerformanceCounter(&m_data->mStartTime);
  m_data->mStartTick = GetTickCount64();
#else
#ifdef __APPLE__
  m_data->mStartTimeNano = mach_absolute_time();
#endif
  gettimeofday(&m_data->mStartTime, 0);
#endif
}

/// Returns the time in ms since the last call to reset or since
/// the TinyClock was created.
unsigned long long int TinyClock::get_time_milliseconds() {
#ifdef BT_USE_WINDOWS_TIMERS
  LARGE_INTEGER currentTime;
  QueryPerformanceCounter(&currentTime);
  LONGLONG elapsedTime = currentTime.QuadPart - m_data->mStartTime.QuadPart;
  // Compute the number of millisecond ticks elapsed.
  unsigned long msecTicks =
      (unsigned long)(1000 * elapsedTime / m_data->mClockFrequency.QuadPart);

  return msecTicks;
#else

  struct timeval currentTime;
  gettimeofday(&currentTime, 0);
  return (currentTime.tv_sec - m_data->mStartTime.tv_sec) * 1000 +
         (currentTime.tv_usec - m_data->mStartTime.tv_usec) / 1000;
#endif
}

/// Returns the time in us since the last call to reset or since
/// the Clock was created.
unsigned long long int TinyClock::get_time_microseconds() {
#ifdef BT_USE_WINDOWS_TIMERS
  // see
  // https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx
  LARGE_INTEGER currentTime, elapsedTime;

  QueryPerformanceCounter(&currentTime);
  elapsedTime.QuadPart = currentTime.QuadPart - m_data->mStartTime.QuadPart;
  elapsedTime.QuadPart *= 1000000;
  elapsedTime.QuadPart /= m_data->mClockFrequency.QuadPart;

  return (unsigned long long)elapsedTime.QuadPart;
#else

  struct timeval currentTime;
  gettimeofday(&currentTime, 0);
  return (currentTime.tv_sec - m_data->mStartTime.tv_sec) * 1000000 +
         (currentTime.tv_usec - m_data->mStartTime.tv_usec);
#endif
}

unsigned long long int TinyClock::get_time_nanoseconds() {
#ifdef BT_USE_WINDOWS_TIMERS
  // see
  // https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx
  LARGE_INTEGER currentTime, elapsedTime;

  QueryPerformanceCounter(&currentTime);
  elapsedTime.QuadPart = currentTime.QuadPart - m_data->mStartTime.QuadPart;
  elapsedTime.QuadPart *= 1000000000;
  elapsedTime.QuadPart /= m_data->mClockFrequency.QuadPart;

  return (unsigned long long)elapsedTime.QuadPart;
#else

#ifdef __APPLE__
  uint64_t ticks = mach_absolute_time() - m_data->mStartTimeNano;
  static long double conversion = 0.0L;
  if (0.0L == conversion) {
    // attempt to get conversion to nanoseconds
    mach_timebase_info_data_t info;
    int err = mach_timebase_info(&info);
    if (err) {
      assert(0);
      conversion = 1.;
    }
    conversion = info.numer / info.denom;
  }
  return (ticks * conversion);

#else  //__APPLE__

#ifdef BT_LINUX_REALTIME
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return 1000000000 * ts.tv_sec + ts.tv_nsec;
#else
  struct timeval currentTime;
  gettimeofday(&currentTime, 0);
  return (currentTime.tv_sec - m_data->mStartTime.tv_sec) * 1e9 +
         (currentTime.tv_usec - m_data->mStartTime.tv_usec) * 1000;
#endif  // BT_LINUX_REALTIME

#endif  //__APPLE__
#endif
}

/// Returns the time in s since the last call to reset or since
/// the Clock was created.
double TinyClock::get_time_seconds() {
  static const double microseconds_to_seconds = double(0.000001);
  return double(get_time_microseconds()) * microseconds_to_seconds;
}

void TinyClock::usleep(int microSeconds)
{
#ifdef _WIN32
	if (microSeconds == 0)
	{
		Sleep(0);
	}
	else
	{
		int millis = microSeconds / 1000;
		if (millis < 1)
			millis = 1;
		Sleep(millis);
	}
#else
	if (microSeconds > 0)
	{
		::usleep(microSeconds);
		//struct timeval tv;
		//tv.tv_sec = microSeconds/1000000L;
		//tv.tv_usec = microSeconds%1000000L;
		//return select(0, 0, 0, 0, &tv);
	}
#endif
}
