/*
 *coppeliaSimRobot.cpp
 *
 * Author: Feijian.Ni
 * Date: 2020.7.6
 */

#include "coppeliaSimRobot.hpp"
#include "cmrTimer.hpp"

namespace cmr {
//--------------------Variables Definition------------------------
//! default remote server connection port
const int g_defaultRemotePort = 3000;

//! default time in milliseconds before stop trying to connect server[ms]
const int g_defaultTimeOutInMs = 2000;

//! defualt communication period in ms
const int g_defaultCommCycleInMs = 5;

//! joint name prefix for coppelia robot, all the joint name should named as
//! "jointX", X start from 1
const std::string g_jntNamePrefix = "joint";

//--------------------Class Function Definition--------------------
//! construct/disconstruct funciton
coppeliaSimRobot::coppeliaSimRobot(int robotDofs)
    : cmrRobotInterface(robotDofs) {
  m_remoteConnectPortNum = g_defaultRemotePort;
  m_timeOutInMs = g_defaultTimeOutInMs;
  m_commCycleInMs = g_defaultCommCycleInMs;
}

coppeliaSimRobot::~coppeliaSimRobot() {
  // stop data streaming
  float jointPos;
  float jointTrq;
  for (const auto handle : m_jointHandles) {
    simxGetJointPosition(m_clientID, handle, &jointPos,
                         simx_opmode_discontinue);
    simxGetJointForce(m_clientID, handle, &jointTrq, simx_opmode_discontinue);
  }
  simxInt pingTime;
  simxGetPingTime(m_clientID, &pingTime);

  // stop simulation
  simxStopSimulation(m_clientID, simx_opmode_oneshot);
  simxFinish(m_clientID);
}

//！ init coppelia simulaiton communication parameters
void coppeliaSimRobot::setCoppeliaCommuParams(
    const std::string &connectionAddress, int connectionPort, int timeOutInMs,
    int commThreadCycleInMs) {
  m_remoteConnectIP = connectionAddress;
  m_remoteConnectPortNum = connectionPort;
  m_timeOutInMs = timeOutInMs;
  m_commCycleInMs = commThreadCycleInMs;
}

//! connect real/simulation robot
cmrErrorType coppeliaSimRobot::robotConnect() {
  // connect remote server
  m_clientID = simxStart(m_remoteConnectIP.c_str(), m_remoteConnectPortNum,
                         true, true, m_timeOutInMs, m_commCycleInMs);
  if (-1 == m_clientID) {
    throw cmrException("Could not connect to coopelia simulation");
  }

  // get robot joint handle
  m_jointHandles.resize(m_robotDofs);
  for (unsigned int i = 0; i < m_robotDofs; i++) {
    std::string jointName = g_jntNamePrefix + std::to_string(i);
    if (simxGetObjectHandle(m_clientID, jointName.c_str(), &(m_jointHandles[i]),
                            simx_opmode_blocking) != simx_return_ok) {
      throw cmrException("Could no get handl of " + jointName);
    }
  }

  // enable synchronous mode
  simxSynchronous(m_clientID, true);
  simxStartSimulation(m_clientID, simx_opmode_oneshot);

  // sent robot status reading command for first time in streaming mode
  float jointPos;
  float jointTrq;
  for (const auto handle : m_jointHandles) {
    simxGetJointPosition(m_clientID, handle, &jointPos, simx_opmode_streaming);
    simxGetJointForce(m_clientID, handle, &jointTrq, simx_opmode_streaming);
  }

  // make sure the first data arrive
  double commuTimeMs = 0.0;
  while (true) {
    bool getData = true;
    simxFloat jntPos;
    simxFloat jntTrq;
    for (unsigned int i = 0; i < m_robotDofs; i++) {
      if (simxGetJointPosition(m_clientID, m_jointHandles[i], &jntPos,
                               simx_opmode_buffer) != simx_return_ok ||
          simxGetJointForce(m_clientID, m_jointHandles[i], &jntTrq,
                            simx_opmode_buffer) != simx_return_ok) {
        getData = false;
        commuTimeMs++;
        cmrSleep(1.0);
        break;
      }
      m_jointCurPos(i) = static_cast<double>(jntPos);
      m_jointCurTrq(i) = static_cast<double>(jntTrq);
    }

    if (commuTimeMs > m_timeOutInMs) {
      throw cmrException("Faild to read robot data");
    } else if (getData) {
      break;
    }
  }

  return CMR_SUCCESS;
}

/* cyclic step function
 * This funciton is called cyclic to communicate with real/simulation robot
 * to get current robot status and sent commands to robot
 */
cmrErrorType coppeliaSimRobot::robotStep() {
  // get/send command joint position/torque to robot
  simxFloat jntPos;
  simxFloat jntTrq;
  for (unsigned int i = 0; i < m_robotDofs; i++) {
    simxGetJointPosition(m_clientID, m_jointHandles[i], &(jntPos),
                         simx_opmode_buffer);
    m_jointCurPos(i) = static_cast<double>(jntPos);
    simxGetJointForce(m_clientID, m_jointHandles[i], &(jntTrq),
                      simx_opmode_buffer);
    m_jointCurTrq(i) = static_cast<double>(jntTrq);

    simxSetJointTargetPosition(m_clientID, m_jointHandles[i], m_jointCmdPos(i),
                               simx_opmode_oneshot);
    simxSetJointTargetVelocity(m_clientID, m_jointHandles[i], m_jointCmdVel(i),
                               simx_opmode_oneshot);
    simxSetJointMaxForce(m_clientID, m_jointHandles[i], m_jointCmdTrq(i),
                         simx_opmode_oneshot);
  }

  // trigger server simulation
  simxSynchronousTrigger(m_clientID);
  // simxGetPingTime(m_clientID, &simuPingTime);
  return CMR_SUCCESS;
}

//! check if robot is connected
bool coppeliaSimRobot::isConnected() {
  if (simxGetConnectionId(m_clientID) != -1) {
    return true;
  }
  return false;
}
} // namespace cmr