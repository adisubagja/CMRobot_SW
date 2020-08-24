/*
 *cmrRobotInterface.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.7.6
 */

#include "cmrRobotInterface.hpp"

namespace cmr {

//! construct/disconstruct funciton
cmrRobotInterface::cmrRobotInterface(int robotDofs)
    : m_isRobotConnected(false), m_robotDofs(robotDofs) {
  assert(robotDofs > 0);
  m_jointCurPos = cmrVectorXd::Zero(robotDofs);
  m_jointCurVel = cmrVectorXd::Zero(robotDofs);
  m_jointCurTrq = cmrVectorXd::Zero(robotDofs);
  m_jointCmdPos = cmrVectorXd::Zero(robotDofs);
  m_jointCmdVel = cmrVectorXd::Zero(robotDofs);
  m_jointCmdTrq = cmrVectorXd::Zero(robotDofs);
}

cmrRobotInterface::~cmrRobotInterface() {}

} // namespace cmr