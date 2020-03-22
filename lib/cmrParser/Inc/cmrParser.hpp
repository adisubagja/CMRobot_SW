/*
 * cmrParser.hpp
 *
 * Description: This file provide the parser of robot config file,
 * and parse all the other relative config files
 *
 * Author: Feijian.Ni
 * Date: 2020.3.22
 *
 */

#include "baseStdLibs.hpp"
#include "cmrException.hpp"
#include "cmrRobotData.hpp"
#include "cmrURDFParser.hpp"
#include "tinyxml2/tinyxml2.h"

using namespace tinyxml2;

namespace cmr {

class cmrParser {
public:
  cmrParser();
  ~cmrParser();

  //! parse robot config file, and relative config file
  cmrErrorType parseRobotCfgFile(const std::string &robotCfgFile,
                                 cmrRobotData *robotDataPtr);

private:
  cmrURDFParser m_urdfParser;
};

} // namespace cmr
