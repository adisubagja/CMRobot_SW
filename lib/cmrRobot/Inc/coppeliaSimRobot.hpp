/*
 * coppeliaSimRobot.hpp
 *
 * Description: This file provide an interface to remote control a simulation
 * robot in coppeliaSim software
 *
 * Author:Feijian.Ni
 * Date: 2020.5.7
 *
 */

#ifndef COOPELIASIMROBOT_HPP_
#define COOPELIASIMROBOT_HPP_

#include "CoppeliaRemoteAPI/extApi.h"
#include "CoppeliaRemoteAPI/extApiPlatform.h"
#include "cmrRobotInterface.hpp"

namespace cmr {
//------------------coppelia simulation robot class----------------------------
class coppeliaSimRobot : public cmrRobotInterface {
public:
  coppeliaSimRobot() = delete;
  coppeliaSimRobot(int robotDofs);
  ~coppeliaSimRobot();

  /* init coppelia simulaiton communication parameters
   * connectionAddress: the ip address where the server is located (i.e.
   * CoppeliaSim) connectionPort: the port number where to connect. Specify a
   * negative port number in order to use shared memory, instead of socket
   * communication. timeOutInMs: the connection time-out in milliseconds for the
   * first connection attempt. In that case, the time-out for blocking function
   * calls is 5000 milliseconds. commThreadCycleInMs: indicates how often data
   * packets are sent back and forth. Reducing this number improves
   * responsiveness, and a default value of 5 is recommended.
   */
  void setCoppeliaCommuParams(const std::string &connectionAddress,
                              int connectionPort, int timeOutInMs,
                              int commThreadCycleInMs);

  //! connect real/simulation robot
  virtual cmrErrorType robotConnect();

  /* cyclic step function
   * This funciton is called cyclic to communicate with real/simulation robot
   * to get current robot status and sent commands to robot
   */
  virtual cmrErrorType robotStep();

  //! check if robot is connected
  virtual bool isConnected();

protected:
  //! ip address of coppelia simulation
  std::string m_remoteConnectIP;

  //! port number to connect
  int m_remoteConnectPortNum;

  //! time in milliseconds before stop trying to connect server[ms]
  int m_timeOutInMs;

  //! communication period
  int m_commCycleInMs;

  //! connected client ID
  int m_clientID;

  //! coppelia robot joint handles
  std::vector<simxInt> m_jointHandles;
};
} // namespace cmr
#endif