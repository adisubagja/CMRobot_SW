/*
 *tterworthLPFilterer.hpp
 *
 * Description: This file provide a rwo filterrthow pass filter template
 * Now this filter support first and second order filter
 *
 * Author: Feijian.Ni
 * Date: 2020.4.6
 */

#ifndef BUTTERWORTHLPFILTER_HPP_
#define BUTTERWORTHLPFILTER_HPP_
#include "cmrException.hpp"
#include "cmrMathDef.hpp"
#include "cmrMatrix.hpp"

namespace cmr {
enum cmrFilterOrder { first_order = 1, second_order };

template <typename T> class ButterworthLPFilter {
public:
  ButterworthLPFilter(cmrFilterOrder filterOrder);
  ~ButterworthLPFilter();

  //! init the filter
  cmrErrorType init(double cutoffFrequency);

  //! filt the data
  T runFilter(T newData);

  //! clear history data
  inline void clear() { m_flagHistoryDataInit = false; }

private:
  //! output history data
  std::vector<T> m_Y;

  //! input history data,including current data
  std::vector<T> m_X;

  //! coefficient of output data (Y^-1, Y^-2)
  std::vector<double> m_coeffY;

  //! coefficent of input data (X, X^-1,X^-2)
  std::vector<double> m_coeffX;

  //! flag if history data is initialized
  bool m_flagHistoryDataInit;

  //! filter order
  cmrFilterOrder m_filterOrder;

  //! filtered result data
  T m_filterResult;

  //! reset the filter data and coefficient
  void reset() {
    // clear data
    m_Y.clear();
    m_X.clear();
    m_coeffY.clear();
    m_coeffX.clear();

    // resize data
    m_Y.resize(m_filterOrder);
    m_coeffY.resize(m_filterOrder);
    m_X.resize(m_filterOrder);
    m_coeffX.resize(m_filterOrder + 1);
  }

  //! init first order low pass filter
  void initFrstOrderLPF(double cutoffFrequency);

  //! init second order low pass filter
  void initSecondOrderLPF(double cutoffFrequency);
};

//! constructors and destructor
template <typename T>
ButterworthLPFilter<T>::ButterworthLPFilter(cmrFilterOrder filterOrder)
    : m_flagHistoryDataInit(false) {
  m_filterOrder = filterOrder;
}

template <typename T> ButterworthLPFilter<T>::~ButterworthLPFilter() {}

//！ init the filter
template <typename T>
cmrErrorType ButterworthLPFilter<T>::init(double cutoffFrequency) {
  switch (m_filterOrder) {
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
  m_coeffY[1] = (2 - 8 / (analogFre * analogFre)) / a;
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
    m_flagHistoryDataInit = true;
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
#endif