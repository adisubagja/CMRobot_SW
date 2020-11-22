/*
 *cmrDoubleSTraj.hpp
 *
 * this file generate trapzoidal trajectory in joint space
 *
 * Author: Feijian.ni
 * Date: 2020.7.4
 */

#ifndef _CMRTRAPZOIDALTRAJ_HPP_
#define _CMRTRAPZOIDALTRAJ_HPP_

#include "cmrMatrix.hpp"

namespace cmr {

//------------------Double S trajectory class----------------------------
class cmrDoubleSTraj {
public:
  cmrDoubleSTraj();
  ~cmrDoubleSTraj();

  cmrDoubleSTraj(const cmrDoubleSTraj&) = delete;
  cmrDoubleSTraj& operator=(const cmrDoubleSTraj&) =delete; 
  //! generate velocity for waypoints

private:
  //! flag if trajectory is generated successfully
  bool m_trajGenerated;

  //! acceleration duration
  double m_accDuration;

  //! deceleration duration
  double m_decDuration;

  //! constant velocity duration;
  double m_constDuration;

  //! trajectory max velocity
  double m_maxTrajVel;

  //！ trajectory max acceleration
  double m_maxTrajAcc;
};

//------------------single dimension basic Double S class----------------------------
class cmrBasicDoubleS{
public:
  cmrBasicDoubleS();
  ~cmrBasicDoubleS();

  //! init
  void init(double q0, double q1, double v0, double v1, double maxV, double maxA, double maxJ);

private:
  //! limit acceleration 
  double m_limitAcc;

  //! limit deceleration
  double m_limitDec;

  //! limit velocity
  double m_limitVel;

  //! acceleration period
  double m_accT;

  //! constant jert time-interval during acceleration phase
  double m_accJerkT;

  //! constant velocity period
  double m_constVelT;
  
  //! deceleration period
  double m_decT;

  //! constant jert time-interval during deceleration phase
  double m_decJerkT;
};

} // namespace cmr
#endif