#ifndef TINY_CLOCK_H
#define TINY_CLOCK_H

/// The TinyClock is a portable basic clock that measures accurate time in
/// seconds, use for profiling.
class TinyClock {
 public:
  TinyClock();

  TinyClock(const TinyClock& other);
  TinyClock& operator=(const TinyClock& other);

  ~TinyClock();

  /// Resets the initial reference time.
  void reset();

  /// Returns the time in ms since the last call to reset or since
  /// the TinyClock was created.
  unsigned long long int get_time_milliseconds();

  /// Returns the time in us since the last call to reset or since
  /// the Clock was created.
  unsigned long long int get_time_microseconds();

  unsigned long long int get_time_nanoseconds();

  /// Returns the time in seconds since the last call to reset or since
  /// the Clock was created.
  double get_time_seconds();

  /// Sleep for 'microSeconds', to yield to other threads and not waste 100% CPU
  /// cycles. Note that some operating systems may sleep a longer time.
  static void usleep(int microSeconds);

 private:
  struct TinyClockData* m_data;
};

#endif  // TINY_CLOCK_H
