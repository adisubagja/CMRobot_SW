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
#include "cmrURDFParser.hpp"
#include "multiThreadTest.hpp"
#include "myHello.h"
#include "tinyxml2/tinyxml2.h"
#include <chrono>
#include <fstream>
#include <math.h>
#include <unistd.h>

using namespace cmr;
using namespace std::chrono;

//!---------------------------Global
//!Variables-----------------------------------
//! robot control cycle time (s)
const double g_controlCycleTime = 0.001;

int main() {
  // while (true) {
  //   high_resolution_clock::time_point loopstart =
  //   high_resolution_clock::now();

  //   high_resolution_clock::time_point loopend = high_resolution_clock::now();
  //   double loopTime = duration_cast<microseconds>(loopend -
  //   loopstart).count(); assert(loopTime < 1e6 * g_controlCycleTime);

  //   double sleepTime = 1e6 * g_controlCycleTime - loopTime;
  //   usleep(sleepTime);

  //   std::cout << "looptime is " << sleepTime << std::endl;
  // }
  // std::string urdffile = "../config/robotCfg/flexivRobot.urdf";
  std::string configFile = "../config/CMRobotCfg.xml";
  cmrParser cmrParser;
  cmrRobotData *robotData = new cmrRobotData();
  try {
    cmrParser.parseRobotCfgFile(configFile, robotData);
    // robotData->printInfo();
  } catch (cmrException cmrError) {
    std::cout << cmrError.what();
  }

  cmrDynamics robotDynamics;
  robotDynamics.createRobotModel(*robotData);

  cmrVectorXd jointPos = cmrVectorXd::Zero(7);
  jointPos(0) = 0.0;
  jointPos(1) = _DegToRad(-40.06);
  jointPos(2) = 0;
  jointPos(3) = _DegToRad(-90.06);
  jointPos(4) = 0;
  jointPos(5) = _DegToRad(40.01);
  jointPos(6) = 0;
  cmrTransform zeroTrans;
  unsigned int linkId = robotDynamics.getLinkId("Link4");
  robotDynamics.updataRobotKinematic(jointPos);

  // cmrTransform
  // tcpTrans=robotDynamics.getPointTransform(jointPos,linkId,zeroTrans);
  cmrTransform tcpTrans = robotDynamics.getTcpTransform(jointPos);
  std::cout << tcpTrans.toStr();
  return 0;
}