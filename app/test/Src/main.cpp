
#include "baseStdLibs.hpp"
#include "cmrDynamics.hpp"
#include "cmrException.hpp"
#include "cmrMath.hpp"
#include "cmrURDFParser.hpp"
#include "multiThreadTest.hpp"
#include "myHello.h"
#include "tinyxml2/tinyxml2.h"
#include <fstream>
#include <math.h>

int a =0;
using namespace cmr;
int main()
{
  // std::string urdffile="../../../config/robotCfg/flexivRobot.urdf";
  std::string urdffile = "../config/robotCfg/flexivRobot.urdf";
  cmrURDFParser urdfParser = cmrURDFParser();
  try {
    cmrRobotData *robotData = urdfParser.parseURDF(urdffile);
    robotData->printInfo();
  } catch (cmrException cmrError) {
    std::cout << cmrError.what();
  }

  // tinyxml2::XMLDocument doc;
  // cmrVector3d vec3;
  // cmrMatrix3d mat3;
  // cmrAngleAxis axis(M_PI / 4, cmrVector3d(1, 0, 0));
  // cmrQuat quat;
  // cmrDynamics dynamicModel = cmrDynamics();
  // const char *str1 = "abcd";
  // std::cout << str1 << std::endl;
  // cmrVector3d e0(30 * M_PI / 180, 45 * M_PI / 180, 60 * M_PI / 180);
  // cmrAngleAxis rollAng(e0(2), cmrVector3d::UnitX());
  // cmrAngleAxis pitchAng(e0(1), cmrVector3d::UnitY());
  // cmrAngleAxis yawAng(e0(0), cmrVector3d::UnitZ());

  // cmrMatrix3d rot = (yawAng * pitchAng * rollAng).matrix();

  // cmrVector3d e1 = rot.eulerAngles(0, 1, 2);
  // cmrAngleAxis yawAng1(e1(2), cmrVector3d::UnitZ());
  // cmrAngleAxis pitchAng1(e1(1), cmrVector3d::UnitY());
  // cmrAngleAxis rollAng1(e1(0), cmrVector3d::UnitX());
  // cmrMatrix3d rot1 = (rollAng1 * pitchAng1 * yawAng1).matrix();

  // cmrVector3d rotAxis(1, 2, 3);
  // rotAxis.normalize();
  // double angle = 60 * M_PI / 180;
  // cmrVector3d rotAngAxis = angle * rotAxis;

  // cmrAngleAxis rotAngle(angle, rotAxis);
  // cmrMatrix3d rot2 = rotAngle.matrix();

  // cmrAngleAxis yawAng3(rotAngAxis(2), cmrVector3d::UnitZ());
  // cmrAngleAxis pitchAng3(rotAngAxis(1), cmrVector3d::UnitY());
  // cmrAngleAxis rollAng3(rotAngAxis(0), cmrVector3d::UnitX());
  // cmrMatrix3d rot3 = (yawAng3 * pitchAng3 * rollAng3).matrix();
  // std::cout << rot2 << std::endl;
  // std::cout << rot3 << std::endl;
  // std::cout<<"axis is "<<axis.matrix()<<std::endl;
  // mat3=axis;
  // std::cout<<"mat is "<<mat3<<std::endl;
  // quat=axis;
  // std::cout<<"quat is "<<quat.coeffs()<<std::endl;
  // std::cout<<"quat is "<<quat.w()<<quat.x()<<quat.y()<<quat.z()<<std::endl;
  // std::thread t1(addNum,std::ref(a));
  // std::thread t2(minuNum,std::ref(a));
  // t1.join();
  // t2.join();
  // printA();
  // printB();
  printMyHello();
  return 0;
}