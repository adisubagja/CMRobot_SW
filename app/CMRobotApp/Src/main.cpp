/*
 * CMRobotApp.cpp
 *
 * Description: Main function for the CMRobot
 *
 */

#include "baseStdLibs.hpp"
#include "cmrDynamics.hpp"
#include "cmrException.hpp"
#include "cmrMath.hpp"
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
  while (true) {
    high_resolution_clock::time_point loopstart = high_resolution_clock::now();

    high_resolution_clock::time_point loopend = high_resolution_clock::now();
    double loopTime = duration_cast<microseconds>(loopend - loopstart).count();
    assert(loopTime < 1e6 * g_controlCycleTime);

    double sleepTime = 1e6 * g_controlCycleTime - loopTime;
    usleep(sleepTime);

    std::cout << "looptime is " << sleepTime << std::endl;
  }
  // std::string urdffile="../../../config/robotCfg/flexivRobot.urdf";
  std::string urdffile = "../config/robotCfg/flexivRobot.urdf";
  cmrURDFParser urdfParser;
  cmrRobotData *robotData = new cmrRobotData();
  try {
    urdfParser.parseURDF(urdffile, robotData);
    robotData->printInfo();
  } catch (cmrException cmrError) {
    std::cout << cmrError.what();
  }

  return 0;
}