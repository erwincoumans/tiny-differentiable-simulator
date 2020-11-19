#pragma once

#include <chrono>

namespace tds {
/**
 * Stopwatch implementation to measure elapsed time.
 */
class Stopwatch {
  typedef std::chrono::high_resolution_clock clock_t;

 public:
  /**
   * Starts the timer.
   */
  void start() {
    start_ = clock_t::now();
    running_ = true;
  }

  /**
   * Stops the timer.
   * @return Elapsed time in seconds.
   */
  double stop() {
    elapsed_ = calcElapsed_();
    running_ = false;
    return elapsed_;
  }

  /**
   * Elapsed time in seconds.
   */
  double elapsed() const {
    if (!running_)
      return elapsed_;
    else
      return calcElapsed_();
  }

  /**
   * Continues the stop watch from when it was stopped.
   */
  void resume() {
    start_ = clock_t::now() -
             std::chrono::microseconds(static_cast<long>(elapsed_ * 1e6));
    running_ = true;
  }

 protected:
  double elapsed_{0};
  bool running_{false};

 private:
  std::chrono::time_point<clock_t> start_;

  double calcElapsed_() const {
    const auto end = clock_t::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start_)
               .count() /
           1e6;
  }
};
}  // namespace tds