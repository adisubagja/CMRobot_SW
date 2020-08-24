
/*
 * cmrTimer.hpp
 *
 * Description: This file provides various timers definition
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 *
 */

#ifndef CMRTIMER_HPP_
#define CMRTIMER_HPP_

#include <chrono>
#include <time.h>

#define NSEC_PER_MSEC 1000000
#define NSEC_PER_SEC 1000000000
namespace cmr {

using namespace std::chrono;

//! sleep for ms millisecond
void cmrSleep(double ms);

//! template class for timer
template <typename timeUnit> class cmrTimer {
public:
  cmrTimer() { m_span = 0.0; };
  ~cmrTimer(){};

  //! reset timer
  void reset() {
    m_span = 0.0;
    m_startPoint = high_resolution_clock::now();
  }

  //! pause timer
  void pause() {
    high_resolution_clock::time_point pausePoint = high_resolution_clock::now();
    m_span += duration_cast<timeUnit>(pausePoint - m_startPoint).count();
    m_startPoint = pausePoint;
  }

  //! start timer
  void start() { m_startPoint = high_resolution_clock::now(); }

  //! get timer duration
  double getDuration() {
    high_resolution_clock::time_point end_tp = high_resolution_clock::now();
    return duration_cast<timeUnit>(end_tp - m_startPoint).count() + m_span;
  }

protected:
  //! start time point
  high_resolution_clock::time_point m_startPoint;

  //! total duration
  double m_span;
};

} // namespace cmr

#endif
