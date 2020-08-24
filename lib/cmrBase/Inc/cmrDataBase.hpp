/*
 *cmrDataBase.hpp
 *
 * Description: This file provide all base data for robot communication and
 *control
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 *
 */

#ifndef CMRROBOTDATABASE_HPP_
#define CMRROBOTDATABASE_HPP_

#include "baseStdLibs.hpp"
#include "cmrMathDef.hpp"
#include "cmrTransform.hpp"

namespace cmr {

//---------------------------Definition-----------------------------
//! data base
struct cmrDataBase {
  //！ robot status data
  cmrRobotStatus m_robotStatus;

  //! robot command data
  cmrRobotCommand m_robotCommands;
};

//! robot status data
struct cmrRobotStatus {
  // init
  void init(int robotDofs) {
    m_jntActPos = cmrVectorXd::Zero(robotDofs);
    m_jntActVel = cmrVectorXd::Zero(robotDofs);
    m_jntActTrq = cmrVectorXd::Zero(robotDofs);
  }

  //! robot actual joint position
  cmrVectorXd m_jntActPos;

  //! robot actual joint velocity
  cmrVectorXd m_jntActVel;

  //! robot actual joint torque
  cmrVectorXd m_jntActTrq;
};

//! robot command data
struct cmrRobotCommand {
  // init
  void init(int robotDofs) {
    m_jntCmdPos = cmrVectorXd::Zero(robotDofs);
    m_jntCmdVel = cmrVectorXd::Zero(robotDofs);
    m_jntCmdTrq = cmrVectorXd::Zero(robotDofs);
  }

  //! robot command joint position
  cmrVectorXd m_jntCmdPos;

  //! robot command joint velocity
  cmrVectorXd m_jntCmdVel;

  //! robot command joint torque
  cmrVectorXd m_jntCmdTrq;
};

} // namespace cmr

#endif