/*
 * cmrTimer.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 *
 */

#include "cmrTimer.hpp"
namespace cmr {

//! sleep for ms millisecond
void cmrSleep(double ms) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_nsec += ms * NSEC_PER_MSEC;
  if (ts.tv_nsec > NSEC_PER_SEC) {
    ts.tv_sec++;
    ts.tv_nsec -= NSEC_PER_SEC;
  }

  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
}

} // namespace cmr