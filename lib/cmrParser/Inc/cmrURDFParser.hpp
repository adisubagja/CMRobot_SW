/*
 * cmrURDFParser.hpp
 *
 * Description: This file provide parser for robot URDF file
 *
 * WARNING: Now Only Seriel Robots Are Supported
 *
 * Author: Feijian.Ni
 * Date: 2020.2.20
 */

#ifndef CMRURDFPARSER_HPP_
#define CMRURDFPARSER_HPP_

#include "baseStdLibs.hpp"
#include "cmrException.hpp"
#include "cmrRobotCfgData.hpp"
#include "tinyxml2/tinyxml2.h"

using namespace tinyxml2;

namespace cmr {

class cmrURDFParser {
public:
  cmrURDFParser();
  ~cmrURDFParser();

  //! parse URDF file
  cmrErrorType parseURDF(const std::string &URDFFileName,
                         cmrRobotCfgData *robotDataPtr);

protected:
  //! parse link Data
  cmrErrorType parseLinkData(const XMLElement *curLinkElement,
                             cmrLinkData &linkData);

  //! parse joint Data
  cmrErrorType parseJointData(const XMLElement *curJointElement,
                              cmrJointData &jointData);

  //! converte rpy string to rot matrix
  void rpyStrToRotMat(const std::string rpyStr, cmrMatrix3d &rotMat);
};
} // namespace cmr

#endif
