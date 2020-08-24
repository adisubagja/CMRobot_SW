/*
 * cmrParser.hpp
 *
 * Author: Feijian.Ni
 * Date: 2020.3.22
 *
 */

#include "cmrParser.hpp"
#include <sstream>

namespace cmr {

cmrParser::cmrParser() {}

cmrParser::~cmrParser() {}

//! parse robot config file, and relative config file
cmrErrorType cmrParser::parseRobotCfgFile(const std::string &robotCfgFile,
                                          cmrRobotCfgData *robotDataPtr) {
  //! init robot data struct
  robotDataPtr->init();

  // load config file
  XMLDocument configFile;
  if (XML_SUCCESS != configFile.LoadFile(robotCfgFile.c_str())) {
    std::string msg = "Could not open URDF file " + robotCfgFile;
    throw cmrException(msg);
  };

  // get rootroot element
  XMLElement *cfgFileRootElement = configFile.RootElement();

  // get robot config parameters
  XMLElement *robotElement = cfgFileRootElement->FirstChildElement("robot");

  // get robot name
  XMLElement *robotNameElement = robotElement->FirstChildElement("robot_name");
  _CHECK_NULLPOINTER(robotNameElement,
                     "Could not find robot_name element in config file");
  robotDataPtr->m_robotName = robotNameElement->FirstChild()->Value();

  // parse robot urdf file
  XMLElement *robotURDFElement = robotElement->FirstChildElement("robot_urdf");
  _CHECK_NULLPOINTER(robotURDFElement,
                     "Could not find robot_urdf element in config file");
  std::string urdfFile = robotURDFElement->FirstChild()->Value();
  m_urdfParser.parseURDF(urdfFile, robotDataPtr);

  // parse robot gravity
  XMLElement *gravityElement = robotElement->FirstChildElement("gravity");
  _CHECK_NULLPOINTER(gravityElement,
                     "Could not find gravity element in config file");
  std::istringstream gravityStr(gravityElement->FirstChild()->Value());
  gravityStr >> robotDataPtr->m_gravity[0] >> robotDataPtr->m_gravity[1] >>
      robotDataPtr->m_gravity[2];

  // get robot tcp element
  XMLElement *tcpElement = cfgFileRootElement->FirstChildElement("robot_tcp");

  // parse tcp parent link
  XMLElement *tcpParentLink = tcpElement->FirstChildElement("parent_link_name");
  _CHECK_NULLPOINTER(tcpParentLink, "Could not find tcp parent In config file");
  robotDataPtr->m_tcpData.m_parentLink = tcpParentLink->FirstChild()->Value();

  // parse tcp position in parent link
  XMLElement *tcpPosInParentLink =
      tcpElement->FirstChildElement("position_in_link");
  _CHECK_NULLPOINTER(tcpPosInParentLink,
                     "Could not find tcp position in link In config file");
  std::istringstream tcpPos(tcpPosInParentLink->FirstChild()->Value());
  tcpPos >> robotDataPtr->m_tcpData.m_posInParent(0) >>
      robotDataPtr->m_tcpData.m_posInParent(1) >>
      robotDataPtr->m_tcpData.m_posInParent(2);

  // parse tcp orientation in parent link
  XMLElement *tcpOriInParentLink =
      tcpElement->FirstChildElement("orientation_in_link");
  _CHECK_NULLPOINTER(tcpPosInParentLink,
                     "Could not find tcp orientation in link In config file");
  std::istringstream tcpOri(tcpPosInParentLink->FirstChild()->Value());
  double quat_w, quat_x, quat_y, quat_z;
  tcpPos >> quat_w >> quat_x >> quat_y >> quat_z;
  cmrQuat tcpQuat(quat_w, quat_x, quat_y, quat_z);
  robotDataPtr->m_tcpData.m_rotInParent = tcpQuat.matrix();

  return CMR_SUCCESS;
}

} // namespace cmr
