/*
 *ButterworthLPFilter.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.4.6
 */

#include "ButterworthLPFilter.hpp"

namespace cmr {
//! constructors and destructor
template <typename T>
ButterworthLPFilter<T>::ButterworthLPFilter(cmrFilterOrder filterOrder)
    : m_flagHistoryDataInit(false) {
  m_filterOrder = filterOrder;
}

template <typename T> ButterworthLPFilter<T>::~ButterworthLPFilter() {}

//！ init the filter
template <typename T>
cmrErrorType ButterworthLPFilter<T>::init(double cutoffFrequency,
                                          cmrFilterOrder filterOrder) {
  switch (filterOrder) {
  case first_order:
    // init first order low pass filter
    initFrstOrderLPF(cutoffFrequency);
    break;
  case second_order:
    // init second order low pass filter
    initSecondOrderLPF(cutoffFrequency);
  default:
    return CMR_OUT_RANGE;
    break;
  }
  m_filterOrder = filterOrder;
  return CMR_SUCCESS;
}

//! init first order low pass filter
template <typename T>
void ButterworthLPFilter<T>::initFrstOrderLPF(double cutoffFrequency) {
  // reset data
  reset();

  // compute numeric frequency
  double numericFre = 2 * g_cmrPI * cutoffFrequency * g_controlCycleTime;

  // compute analog with predistiortions
  double analogFre = 2 * tan(numericFre / 2);

  // compute filter coeffecients
  m_coeffY[0] = (analogFre - 2) / (analogFre + 2);
  m_coeffX[0] = m_coeffX[1] = analogFre / (analogFre + 2);
}

//! init second order low pass filter
template <typename T>
void ButterworthLPFilter<T>::initSecondOrderLPF(double cutoffFrequency) {
  // reset data
  reset();

  // compute numeric frequency
  double numericFre = 2 * g_cmrPI * cutoffFrequency * g_controlCycleTime;

  // compute analog with predistiortions
  double analogFre = 2 * tan(numericFre / 2);

  // compute filter coefficients
  double a = 4 / (analogFre * analogFre) + 2 * sqrt(2) / analogFre + 1;
  m_coeffY[0] = (4 / (analogFre * analogFre) - 2 * sqrt(2) / analogFre + 1) / a;
  m_coeffY[1] = 2 - 8 / (analogFre * analogFre);
  m_coeffX[0] = m_coeffX[2] = 1 / a;
  m_coeffX[1] = 2 / a;
}
//! filt the data
template <typename T> T ButterworthLPFilter<T>::runFilter(T newData) {
  // init history data
  if (!m_flagHistoryDataInit) {
    for (int i = 0; i < m_X.size(); i++) {
      m_Y[i] = m_X[i] = newData;
    }
  }
  // compute result
  m_filterResult = m_coeffX.back() * newData;
  for (int i = 0; i < m_Y.size(); i++) {
    m_filterResult = m_filterResult + m_coeffX[i] * m_X[i];
    m_filterResult = m_filterResult - m_coeffY[i] * m_Y[i];
  }

  // update history data
  m_Y.push_back(m_filterResult);
  m_Y.erase(m_Y.begin());
  m_X.push_back(newData);
  m_X.erase(m_X.begin());

  return m_filterResult;
}

} // namespace cmr