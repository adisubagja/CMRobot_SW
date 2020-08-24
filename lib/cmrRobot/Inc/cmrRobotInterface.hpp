/*
 *cmrRobotInterface.hpp
 *
 * Description: This file provide all interface functions to get robot status
 *and send command to robot.
 *
 * Author: Feijian.Ni
 * Date: 2020.7.6
 */

#ifndef CMRROBOTINTERFACE_HPP_
#define CMRROBOTINTERFACE_HPP_

#include "cmrException.hpp"
#include "cmrMathDef.hpp"

namespace cmr {
//------------------robot interface base class----------------------------
class cmrRobotInterface {
public:
  cmrRobotInterface() = delete;
  cmrRobotInterface(int robotDofs);
  ~cmrRobotInterface();

  //! connect real/simulation robot
  virtual cmrErrorType robotConnect() { return CMR_SUCCESS; }

  /* cyclic step function
   * This funciton is called cyclic to communicate with real/simulation robot
   * to get current robot status and sent commands to robot
   */
  virtual cmrErrorType robotStep() = 0;

  //! check if robot is connected
  virtual bool isConnected() = 0;

  //! get robot joint position
  cmrVectorXd getJntCurPos() const { return m_jointCurPos; }

  //! get robot joint velocity
  cmrVectorXd getJntCurVel() const { return m_jointCurPos; }

  //! get robot joint torque
  cmrVectorXd getJntCurTrq() const { return m_jointCurTrq; }

  //! set robot command joint position
  cmrVectorXd setJntCmdPos(const cmrVectorXd &jntPos) {
    m_jointCmdPos = jntPos;
  }

  //! set robot command joint velocity
  cmrVectorXd setJntCmdVel(const cmrVectorXd &jntVel) {
    m_jointCmdVel = jntVel;
  }

  //! set robot command joint torque
  cmrVectorXd setJntCmdTrq(const cmrVectorXd &jntTrq) {
    m_jointCmdTrq = jntTrq;
  }

protected:
  //! robot degree of freedoms
  unsigned int m_robotDofs;

  //! flag is a real/simulation robot is connected
  bool m_isRobotConnected;

  //! robot joint current position
  cmrVectorXd m_jointCurPos;

  //! robot joint current velocity
  cmrVectorXd m_jointCurVel;

  //! robot joint current torque
  cmrVectorXd m_jointCurTrq;

  //! robot command joint position
  cmrVectorXd m_jointCmdPos;

  //! robot command joint velocity
  cmrVectorXd m_jointCmdVel;

  //! robot command joint torque
  cmrVectorXd m_jointCmdTrq;
};

} // namespace cmr
#endif