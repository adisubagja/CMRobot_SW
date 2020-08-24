/*
 *cmrTrapzoidalTraj.hpp
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

class cmrTapzoidalTraj {
public:
  cmrTapzoidalTraj();
  ~cmrTapzoidalTraj();

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
}

} // namespace cmr
#endif