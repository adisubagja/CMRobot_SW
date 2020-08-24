/*
 * CMRobotApp.cpp
 *
 * Description: Main function for the CMRobot
 *
 */

#include "baseStdLibs.hpp"
#include "cmrDynamics.hpp"
#include "cmrException.hpp"
#include "cmrMathDef.hpp"
#include "cmrMatrix.hpp"
#include "cmrParser.hpp"
#include "cmrTimer.hpp"
#include "cmrURDFParser.hpp"
#include "coppeliaSimRobot.hpp"
#include "multiThreadTest.hpp"
#include "myHello.h"
#include "tinyxml2/tinyxml2.h"
#include <chrono>
#include <fstream>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

using namespace cmr;
using namespace std::chrono;

const int COOPELIA_REMORT_PORT = 3000;
const std::string COOPELIA_REMORT_IP = "192.168.144.1";
const int COOPELIA_MAX_CONNETION_TIME = 2000;
const int COOPELIA_COMM_TIME = 5;

int main() {
  // coppelia robot interface
  coppeliaSimRobot cmrRobot = coppeliaSimRobot(7);
  cmrRobot.setCoppeliaCommuParams(COOPELIA_REMORT_IP, COOPELIA_REMORT_PORT,
                                  COOPELIA_MAX_CONNETION_TIME,
                                  COOPELIA_COMM_TIME);
  cmrRobot.robotConnect();

  cmrVectorXd robotJntVel;
  cmrTimer<seconds> timer;
  timer.start();
  while (cmrRobot.isConnected()) {
    high_resolution_clock::time_point loopstart = high_resolution_clock::now();

    // test trajectory
    double timePeriod = timer.getDuration();
    cmrVectorXd cmdJntVel =
        0.1 * cmrVectorXd::Ones(7) * sin(2.0 * g_cmrPI * 0.1 * timePeriod);
    cmrRobot.setJntCmdVel(cmdJntVel);

    robotJntVel = cmrRobot.getJntCurPos();
    std::cout << robotJntVel << std::endl;
    //
    // targetVlocity+=0.02*g_cmrPI/180;
    // simxSetJointTargetVelocity(clientID,jointhandle3,targetVlocity,simx_opmode_oneshot);
    // simxGetJointPosition(clientID,jointhandle3,&jointPos3,simx_opmode_buffer);
    // simxGetObjectVelocity(clientID,jointhandle4,jointLinearVel3,jointAngularVel3,simx_opmode_buffer);
    // std::cout << "joint3 position is " << jointPos3*180/g_cmrPI << std::endl;
    // std::cout << "joint3 velocity is0 " << jointLinearVel3[0] << std::endl;
    // std::cout << "joint3 velocity is1 " << jointLinearVel3[1] << std::endl;
    // std::cout << "joint3 velocity is2 " << jointLinearVel3[2] << std::endl;
    // std::cout << "joint3 velocity is4 " << jointAngularVel3[0] << std::endl;
    // std::cout << "joint3 velocity is5 " << jointAngularVel3[1] << std::endl;
    // std::cout << "joint3 velocity is5 " << jointAngularVel3[2] << std::endl;

    // assert(loopTime < 1e6 * g_controlCycleTime);

    cmrRobot.robotStep();
    // simxInt simuPingTime;

    high_resolution_clock::time_point loopend = high_resolution_clock::now();
    double loopTime = duration_cast<microseconds>(loopend - loopstart).count();
    double sleepTime = 1e3 * g_controlCycleTime - loopTime;
    if (sleepTime > 0) {
      cmrSleep(sleepTime);
    } else {
      std::cout << "looptime is " << loopTime << std::endl;
    }
  }
  // std::string urdffile = "../config/robotCfg/flexivRobot.urdf";
  // std::string configFile = "../config/CMRobotCfg.xml";
  // cmrParser cmrParser;
  // cmrRobotData *robotData = new cmrRobotData();
  // try {
  //   cmrParser.parseRobotCfgFile(configFile, robotData);
  //   // robotData->printInfo();
  // } catch (cmrException cmrError) {
  //   std::cout << cmrError.what();
  // }

  // cmrDynamics robotDynamics;
  // robotDynamics.createRobotModel(*robotData);

  // cmrVectorXd jointPos = cmrVectorXd::Zero(7);
  // jointPos(0) = 0.0;
  // jointPos(1) = _DegToRad(-40.06);
  // jointPos(2) = 0;
  // jointPos(3) = _DegToRad(-90.06);
  // jointPos(4) = 0;
  // jointPos(5) = _DegToRad(40.01);
  // jointPos(6) = 0;
  // cmrTransform zeroTrans;
  // unsigned int linkId = robotDynamics.getLinkId("Link4");
  // robotDynamics.updataRobotKinematic(jointPos);

  // // cmrTransform
  // // tcpTrans=robotDynamics.getPointTransform(jointPos,linkId,zeroTrans);
  // cmrTransform tcpTrans = robotDynamics.getTcpTransform(jointPos);
  // std::cout << tcpTrans.toStr();
  return 0;
}